// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Auto-Lead Compensation for robot velocity adjustment.
 *
 * Calculates how much to adjust aim based on robot movement
 * to compensate for the ball's trajectory while the robot is moving.
 *
 * ACCURACY-FIRST DESIGN:
 * - Returns zero compensation if conditions are uncertain
 * - Uses conservative multipliers
 * - Caps maximum compensation angle
 *
 * @author Baichen Yu
 */
public class AutoLeadCompensation {

  // Physics constants
  private static final double BALL_EXIT_VELOCITY = 12.0;  // m/s estimated

  // Compensation limits for accuracy
  private static final double MAX_LEAD_ANGLE_DEGREES = 8.0;  // Never exceed this
  private static final double MAX_ROBOT_VELOCITY = 3.0;  // m/s - beyond this, don't compensate
  private static final double MIN_ROBOT_VELOCITY = 0.2;  // Below this, no compensation needed

  // RPM adjustment
  private static final double MAX_RPM_ADJUSTMENT_PERCENT = 0.08;  // Max 8% RPM change

  /**
   * Calculate lead compensation for current robot state.
   *
   * @param chassisSpeeds Current robot velocity
   * @param targetAngleDegrees Angle to target (relative to robot)
   * @param distanceMeters Distance to target
   * @return Compensation result with angle and RPM adjustments
   */
  public static CompensationResult calculate(ChassisSpeeds chassisSpeeds,
                                              double targetAngleDegrees,
                                              double distanceMeters) {
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;
    double robotVelocity = Math.hypot(vx, vy);

    // No compensation needed if stationary or too fast (uncertain)
    if (robotVelocity < MIN_ROBOT_VELOCITY) {
      return CompensationResult.none();
    }
    if (robotVelocity > MAX_ROBOT_VELOCITY) {
      // Too fast - uncertain, return zero compensation
      return new CompensationResult(0, 0, false, "Too fast - no compensation");
    }

    // Calculate time of flight
    double timeOfFlight = distanceMeters / BALL_EXIT_VELOCITY;

    // Calculate how far robot will move during flight
    // Only consider lateral velocity component (perpendicular to target direction)
    double targetAngleRad = Math.toRadians(targetAngleDegrees);

    // Robot velocity components relative to target direction
    double velocityTowardTarget = vx * Math.cos(targetAngleRad) + vy * Math.sin(targetAngleRad);
    double velocityLateral = -vx * Math.sin(targetAngleRad) + vy * Math.cos(targetAngleRad);

    // Lateral offset during time of flight
    double lateralOffset = velocityLateral * timeOfFlight;

    // Convert lateral offset to angle compensation
    // Using small angle approximation: angle ≈ offset / distance
    double compensationAngleRad = Math.atan2(lateralOffset, distanceMeters);
    double compensationAngleDeg = Math.toDegrees(compensationAngleRad);

    // Cap the compensation for safety
    compensationAngleDeg = Math.max(-MAX_LEAD_ANGLE_DEGREES,
        Math.min(MAX_LEAD_ANGLE_DEGREES, compensationAngleDeg));

    // Calculate RPM adjustment for forward/backward motion
    // Moving toward target = ball effectively travels less distance
    // Moving away = ball travels more distance
    double effectiveDistanceChange = -velocityTowardTarget * timeOfFlight;
    double percentDistanceChange = effectiveDistanceChange / distanceMeters;

    // Cap RPM adjustment
    double rpmAdjustmentPercent = Math.max(-MAX_RPM_ADJUSTMENT_PERCENT,
        Math.min(MAX_RPM_ADJUSTMENT_PERCENT, percentDistanceChange * 0.5));

    // Build status message
    String status = String.format("v=%.1fm/s lat=%.2f comp=%.1f°",
        robotVelocity, velocityLateral, compensationAngleDeg);

    return new CompensationResult(compensationAngleDeg, rpmAdjustmentPercent, true, status);
  }

  /**
   * Result of lead compensation calculation.
   */
  public static class CompensationResult {
    /** Angle to add to target angle (degrees, positive = aim more right) */
    public final double angleOffsetDegrees;

    /** RPM multiplier adjustment (e.g., 0.05 = increase RPM by 5%) */
    public final double rpmAdjustmentPercent;

    /** Whether compensation is being applied */
    public final boolean isCompensating;

    /** Status message for telemetry */
    public final String status;

    public CompensationResult(double angleOffset, double rpmAdjust,
                               boolean compensating, String statusMsg) {
      this.angleOffsetDegrees = angleOffset;
      this.rpmAdjustmentPercent = rpmAdjust;
      this.isCompensating = compensating;
      this.status = statusMsg;
    }

    /**
     * Create a "no compensation" result.
     */
    public static CompensationResult none() {
      return new CompensationResult(0, 0, false, "Stationary");
    }

    /**
     * Apply RPM adjustment to base RPM.
     *
     * @param baseRPM The RPM calculated from distance
     * @return Adjusted RPM accounting for robot velocity
     */
    public double applyToRPM(double baseRPM) {
      return baseRPM * (1.0 + rpmAdjustmentPercent);
    }

    /**
     * Apply angle offset to base angle.
     *
     * @param baseAngleDegrees The angle to target
     * @return Adjusted angle accounting for robot velocity
     */
    public double applyToAngle(double baseAngleDegrees) {
      return baseAngleDegrees + angleOffsetDegrees;
    }

    @Override
    public String toString() {
      if (!isCompensating) {
        return "No compensation";
      }
      return String.format("Lead: %.1f° RPM: %+.1f%% [%s]",
          angleOffsetDegrees, rpmAdjustmentPercent * 100, status);
    }
  }

  /**
   * Check if compensation should be applied based on confidence.
   * Use this to decide whether to trust the compensation values.
   *
   * @param result The compensation result to evaluate
   * @param minConfidence Minimum confidence threshold (0-1)
   * @return true if compensation should be applied
   */
  public static boolean shouldApplyCompensation(CompensationResult result,
                                                  double minConfidence) {
    if (!result.isCompensating) {
      return false;
    }

    // If lead angle is very small, just apply it
    if (Math.abs(result.angleOffsetDegrees) < 1.0) {
      return true;
    }

    // For larger angles, we need more confidence
    // (This could be enhanced with velocity stability data)
    return Math.abs(result.angleOffsetDegrees) < MAX_LEAD_ANGLE_DEGREES * minConfidence;
  }
}
