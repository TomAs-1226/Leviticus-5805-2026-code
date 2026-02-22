// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

/**
 * Simple Kalman Filter for vision data smoothing.
 *
 * ACCURACY-FIRST DESIGN:
 * - Only smooths when we have consistent data
 * - Resets when measurements are inconsistent (avoids lag)
 * - Never over-smooths - prefers raw data when uncertain
 * - Tuned for FRC AprilTag detection characteristics
 *
 * @author Baichen Yu
 */
public class VisionKalmanFilter {

  // State estimate
  private double m_estimate = 0.0;
  private double m_errorCovariance = 1.0;

  // Tuning parameters (ACCURACY-FOCUSED)
  private final double m_processNoise;      // Q: How much we expect the real value to change
  private final double m_measurementNoise;  // R: How noisy our sensor is

  // Accuracy safeguards
  private boolean m_initialized = false;
  private double m_lastMeasurement = 0.0;
  private int m_consistentReadings = 0;
  private static final int MIN_READINGS_TO_TRUST = 3;  // Need 3 consistent readings before smoothing
  private static final double RESET_THRESHOLD = 0.5;   // Reset if measurement jumps too much

  /**
   * Create a Kalman filter for vision data.
   *
   * @param processNoise How much we expect the real value to change between frames (lower = smoother)
   * @param measurementNoise How noisy the sensor is (higher = more smoothing)
   */
  public VisionKalmanFilter(double processNoise, double measurementNoise) {
    m_processNoise = processNoise;
    m_measurementNoise = measurementNoise;
  }

  /**
   * Create filter with default parameters tuned for AprilTag distance (meters).
   */
  public static VisionKalmanFilter forDistance() {
    // Tuned for distance measurements:
    // - Process noise: 0.01 (target shouldn't move much between frames)
    // - Measurement noise: 0.05 (AprilTag distance has some noise)
    return new VisionKalmanFilter(0.01, 0.05);
  }

  /**
   * Create filter with default parameters tuned for AprilTag yaw (degrees).
   */
  public static VisionKalmanFilter forYaw() {
    // Tuned for yaw measurements:
    // - Process noise: 0.5 (robot can turn quickly)
    // - Measurement noise: 1.0 (yaw has moderate noise)
    return new VisionKalmanFilter(0.5, 1.0);
  }

  /**
   * Update filter with new measurement and get filtered estimate.
   *
   * ACCURACY SAFEGUARDS:
   * - Returns raw measurement until we have enough consistent readings
   * - Resets if measurement jumps significantly (avoids lag)
   * - Never returns stale data
   *
   * @param measurement Raw sensor measurement
   * @return Filtered estimate (may be raw if accuracy would be compromised)
   */
  public double update(double measurement) {
    // Check for significant jump - reset filter to avoid lag
    if (m_initialized && Math.abs(measurement - m_estimate) > RESET_THRESHOLD) {
      reset();
      System.out.println("[KALMAN] Reset due to jump: " +
          String.format("%.2f -> %.2f", m_estimate, measurement));
    }

    // Track consistency
    if (Math.abs(measurement - m_lastMeasurement) < 0.1) {
      m_consistentReadings++;
    } else {
      m_consistentReadings = 0;
    }
    m_lastMeasurement = measurement;

    // Don't smooth until we have consistent readings (accuracy first!)
    if (!m_initialized || m_consistentReadings < MIN_READINGS_TO_TRUST) {
      m_estimate = measurement;
      m_initialized = true;
      return measurement;  // Return raw - not enough data to smooth safely
    }

    // === KALMAN FILTER UPDATE ===

    // Predict step
    // (No control input, so estimate stays the same)
    m_errorCovariance = m_errorCovariance + m_processNoise;

    // Update step
    double kalmanGain = m_errorCovariance / (m_errorCovariance + m_measurementNoise);
    m_estimate = m_estimate + kalmanGain * (measurement - m_estimate);
    m_errorCovariance = (1 - kalmanGain) * m_errorCovariance;

    return m_estimate;
  }

  /**
   * Reset filter state.
   * Call this when target is lost or when switching targets.
   */
  public void reset() {
    m_initialized = false;
    m_estimate = 0.0;
    m_errorCovariance = 1.0;
    m_consistentReadings = 0;
  }

  /**
   * Get current estimate without updating.
   */
  public double getEstimate() {
    return m_estimate;
  }

  /**
   * Check if filter is initialized with enough data.
   */
  public boolean isInitialized() {
    return m_initialized && m_consistentReadings >= MIN_READINGS_TO_TRUST;
  }

  /**
   * Get confidence level (0-1) in current estimate.
   * Higher = more consistent readings.
   */
  public double getConfidence() {
    if (!m_initialized) return 0.0;
    // Confidence increases with consistent readings, maxes at 1.0
    return Math.min(1.0, m_consistentReadings / 10.0);
  }
}
