// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

/**
 * Predictive Tracking for shoot-on-the-move capability.
 *
 * ACCURACY-FIRST DESIGN:
 * - Only enables prediction when conditions are stable
 * - Automatically falls back to direct targeting if uncertain
 * - Uses conservative prediction windows
 *
 * @author Baichen Yu
 */
public class PredictiveTracker {

  // Prediction configuration - conservative for accuracy
  private static final double PREDICTION_TIME_SECONDS = 0.15;  // How far ahead to predict
  private static final double MAX_PREDICTION_VELOCITY = 3.0;   // m/s - don't predict beyond this
  private static final double MIN_VELOCITY_FOR_PREDICTION = 0.3;  // m/s - below this, use direct
  private static final double VELOCITY_STABILITY_THRESHOLD = 0.5;  // m/s change = unstable

  // Accuracy safeguards
  private static final int MIN_STABLE_READINGS = 5;  // Need stable velocity before predicting
  private static final double MAX_ACCELERATION = 2.0;  // m/s² - beyond this = too dynamic

  // State tracking
  private double m_lastVx = 0.0;
  private double m_lastVy = 0.0;
  private double m_lastTimestamp = 0.0;
  private int m_stableReadings = 0;
  private boolean m_predictionEnabled = false;

  // Target position (HUB)
  private Translation2d m_targetPosition;

  public PredictiveTracker(Translation2d hubPosition) {
    m_targetPosition = hubPosition;
    m_lastTimestamp = Timer.getFPGATimestamp();
  }

  /**
   * Update with current robot state.
   *
   * @param robotPose Current robot pose
   * @param chassisSpeeds Current robot velocity
   * @return Predicted aim point (or direct aim if prediction disabled)
   */
  public AimResult update(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - m_lastTimestamp;
    m_lastTimestamp = currentTime;

    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;
    double velocity = Math.hypot(vx, vy);

    // Check velocity stability
    double velocityChange = Math.hypot(vx - m_lastVx, vy - m_lastVy);
    double acceleration = dt > 0 ? velocityChange / dt : 0;

    // Update stability tracking
    if (velocityChange < VELOCITY_STABILITY_THRESHOLD && acceleration < MAX_ACCELERATION) {
      m_stableReadings = Math.min(m_stableReadings + 1, MIN_STABLE_READINGS + 5);
    } else {
      m_stableReadings = Math.max(0, m_stableReadings - 2);  // Faster decay on instability
    }

    m_lastVx = vx;
    m_lastVy = vy;

    // Determine if prediction should be enabled
    boolean shouldPredict = m_stableReadings >= MIN_STABLE_READINGS
        && velocity >= MIN_VELOCITY_FOR_PREDICTION
        && velocity <= MAX_PREDICTION_VELOCITY
        && acceleration < MAX_ACCELERATION;

    m_predictionEnabled = shouldPredict;

    // Calculate aim point
    Translation2d currentPos = robotPose.getTranslation();

    if (!shouldPredict) {
      // Direct targeting - most accurate when stationary or unstable
      return createDirectAimResult(currentPos);
    }

    // Predictive targeting for shoot-on-the-move
    return createPredictiveAimResult(currentPos, vx, vy, velocity);
  }

  private AimResult createDirectAimResult(Translation2d currentPos) {
    Translation2d toTarget = m_targetPosition.minus(currentPos);
    double distance = toTarget.getNorm();
    Rotation2d angle = new Rotation2d(toTarget.getX(), toTarget.getY());

    return new AimResult(
        angle,
        distance,
        0.0,  // No lead angle
        false,  // Not using prediction
        1.0  // High confidence
    );
  }

  private AimResult createPredictiveAimResult(Translation2d currentPos,
                                               double vx, double vy, double velocity) {
    // Predict where robot will be when shot reaches target
    // Uses iterative approach for accuracy

    // Initial estimate: direct distance
    double directDistance = m_targetPosition.minus(currentPos).getNorm();

    // Estimate time of flight (rough - based on typical ball speed)
    // Ball typically travels ~10-15 m/s
    double estimatedBallSpeed = 12.0;  // m/s
    double timeOfFlight = directDistance / estimatedBallSpeed;

    // Total prediction time = spinup + time of flight
    double totalPredictionTime = PREDICTION_TIME_SECONDS + timeOfFlight * 0.5;

    // Cap prediction time for safety
    totalPredictionTime = Math.min(totalPredictionTime, 0.3);

    // Predicted robot position
    double predictedX = currentPos.getX() + vx * totalPredictionTime;
    double predictedY = currentPos.getY() + vy * totalPredictionTime;
    Translation2d predictedPos = new Translation2d(predictedX, predictedY);

    // Calculate aim from predicted position
    Translation2d toTarget = m_targetPosition.minus(predictedPos);
    double predictedDistance = toTarget.getNorm();
    Rotation2d predictedAngle = new Rotation2d(toTarget.getX(), toTarget.getY());

    // Calculate lead angle (difference from direct aim)
    Translation2d directToTarget = m_targetPosition.minus(currentPos);
    Rotation2d directAngle = new Rotation2d(directToTarget.getX(), directToTarget.getY());
    double leadAngle = predictedAngle.minus(directAngle).getDegrees();

    // Confidence decreases with higher velocity and lead angle
    double confidence = 1.0 - (velocity / MAX_PREDICTION_VELOCITY) * 0.3;
    confidence -= Math.abs(leadAngle) / 30.0 * 0.2;  // Reduce confidence for large leads
    confidence = Math.max(0.5, confidence);  // Minimum 50% confidence

    return new AimResult(
        predictedAngle,
        predictedDistance,
        leadAngle,
        true,  // Using prediction
        confidence
    );
  }

  /**
   * Result of aim calculation.
   */
  public static class AimResult {
    public final Rotation2d aimAngle;
    public final double distance;
    public final double leadAngleDegrees;
    public final boolean usingPrediction;
    public final double confidence;

    public AimResult(Rotation2d angle, double dist, double lead,
                     boolean prediction, double conf) {
      this.aimAngle = angle;
      this.distance = dist;
      this.leadAngleDegrees = lead;
      this.usingPrediction = prediction;
      this.confidence = conf;
    }

    @Override
    public String toString() {
      return String.format("Aim: %.1f° dist=%.2fm lead=%.1f° %s conf=%.0f%%",
          aimAngle.getDegrees(), distance, leadAngleDegrees,
          usingPrediction ? "[PREDICT]" : "[DIRECT]",
          confidence * 100);
    }
  }

  /**
   * Check if prediction is currently active.
   */
  public boolean isPredictionEnabled() {
    return m_predictionEnabled;
  }

  /**
   * Get stability reading count (for telemetry).
   */
  public int getStabilityCount() {
    return m_stableReadings;
  }

  /**
   * Reset prediction state.
   * Call when stopping or when accuracy is critical.
   */
  public void reset() {
    m_stableReadings = 0;
    m_predictionEnabled = false;
    m_lastVx = 0;
    m_lastVy = 0;
  }

  /**
   * Update target position (if HUB position changes).
   */
  public void setTargetPosition(Translation2d position) {
    m_targetPosition = position;
  }
}
