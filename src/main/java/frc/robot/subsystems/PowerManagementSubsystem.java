// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PowerConstants;
import java.util.Map;

/**
 * Watches the battery and protects against brownouts.
 * Runs silently in the background - driver shouldn't notice it until really needed.
 *
 * THROTTLE PRIORITY (what gets cut during brownout):
 * 1. NEVER THROTTLED: Swerve, Shooter, Feeder (critical for mobility + scoring)
 * 2. EMERGENCY ONLY: Vision (CPU/power saver), Intake (75% power)
 *
 * Vision is cut completely at EMERGENCY to save CPU and camera bandwidth.
 * This keeps the robot drivable and able to shoot even when battery is critical.
 *
 * @author Baichen Yu
 */
public class PowerManagementSubsystem extends SubsystemBase {

  public enum PowerState {
    NOMINAL,    // >11.5V - all good
    WARNING,    // 11.0-11.5V - watch it
    CRITICAL,   // 10.5-11.0V - save where we can
    EMERGENCY   // <10.5V - save everything now
  }

  private PowerState m_currentState = PowerState.NOMINAL;
  private PowerState m_previousState = PowerState.NOMINAL;

  // Voltage tracking
  private double m_currentVoltage = 12.5;
  private double m_voltageSlope = 0.0;  // V/s (negative = dropping) - short-term
  private double m_longTermSlope = 0.0;  // V/s - session average (more stable)
  private double m_smoothedSlope = 0.0;  // V/s - smoothed for predictions
  private double m_minVoltageSession = 12.5;
  private double m_startVoltage = 12.5;
  private double m_startTime = 0.0;

  // Voltage history for slope calculation (rolling average over ~10 samples)
  private static final int HISTORY = 10;
  private final double[] m_voltageHistory = new double[HISTORY];
  private final double[] m_timeHistory = new double[HISTORY];
  private int m_historyIndex = 0;
  private boolean m_historyFilled = false;

  // Brownout prediction
  private boolean m_brownoutImminent = false;
  private double m_predictedTimeToBrownout = Double.MAX_VALUE;

  // External motor data (updated by other subsystems calling us)
  private double m_maxMotorTemp = 0.0;
  private boolean m_thermalWarning = false;
  private double m_estimatedTotalCurrent = 0.0;
  private double m_peakCurrentSession = 0.0;

  // State change counter
  private int m_warningCount = 0;
  private int m_criticalCount = 0;
  private int m_emergencyCount = 0;

  // NetworkTables
  private final NetworkTable m_powerTable;
  private final DoublePublisher m_voltagePub;
  private final DoublePublisher m_voltageSlopePub;
  private final DoublePublisher m_minVoltagePub;
  private final DoublePublisher m_totalCurrentPub;
  private final DoublePublisher m_maxTempPub;
  private final DoublePublisher m_brownoutTimePub;
  private final StringPublisher m_powerStatePub;

  // Shuffleboard
  private ShuffleboardTab m_powerTab;
  private boolean m_sbInit = false;

  public PowerManagementSubsystem() {
    m_powerTable = NetworkTableInstance.getDefault().getTable("Telemetry/Power");
    m_voltagePub = m_powerTable.getDoubleTopic("voltage").publish();
    m_voltageSlopePub = m_powerTable.getDoubleTopic("voltage_slope").publish();
    m_minVoltagePub = m_powerTable.getDoubleTopic("min_voltage_session").publish();
    m_totalCurrentPub = m_powerTable.getDoubleTopic("total_current_est").publish();
    m_maxTempPub = m_powerTable.getDoubleTopic("max_motor_temp").publish();
    m_brownoutTimePub = m_powerTable.getDoubleTopic("brownout_time_est").publish();
    m_powerStatePub = m_powerTable.getStringTopic("state").publish();

    double now = Timer.getFPGATimestamp();
    for (int i = 0; i < HISTORY; i++) {
      m_voltageHistory[i] = 12.5;
      m_timeHistory[i] = now;
    }
    m_startTime = now;
    m_startVoltage = RobotController.getBatteryVoltage();

    System.out.println("[POWER] Ready - critical at " + PowerConstants.kCriticalVoltage + "V");
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    // ===== OVERVIEW TAB - Power section (cols 0-4, rows 0-1) =====
    ShuffleboardTab overview = Shuffleboard.getTab("Overview");

    // Row 0: Battery voltage bar + power state + brownout warning
    overview.addDouble("Battery V", () -> m_currentVoltage)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 10.0, "max", 13.0))
        .withPosition(0, 0).withSize(2, 1);

    overview.addString("Power State", () -> m_currentState.toString())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 0).withSize(2, 1);

    // Red when brownout risk, green when safe
    overview.addBoolean("Brownout!", () -> m_brownoutImminent)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "green"))
        .withPosition(4, 0).withSize(1, 1);

    // Row 1: Safe to shoot + battery %
    overview.addBoolean("Safe to Shoot", this::isSafeToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "orange"))
        .withPosition(0, 1).withSize(1, 1);

    overview.addDouble("Batt %", this::getBatteryHealthPercent)
        .withPosition(1, 1).withSize(1, 1);

    // ===== DETAILED POWER TAB =====
    m_powerTab = Shuffleboard.getTab("Power");

    m_powerTab.addDouble("Battery V", () -> m_currentVoltage)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 10.0, "max", 13.0, "show value", true))
        .withPosition(0, 0).withSize(3, 3);

    m_powerTab.addString("State", () -> m_currentState.toString())
        .withPosition(3, 0).withSize(2, 1);

    m_powerTab.addBoolean("Brownout Risk", () -> m_brownoutImminent)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "green"))
        .withPosition(5, 0).withSize(2, 1);

    m_powerTab.addBoolean("Safe to Shoot", this::isSafeToShoot)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "orange"))
        .withPosition(7, 0).withSize(2, 1);

    m_powerTab.addDouble("V/s (smoothed)", () -> m_smoothedSlope)
        .withPosition(3, 1).withSize(2, 1);

    m_powerTab.addDouble("Batt Life (s)", this::getEstimatedBatteryLife)
        .withPosition(5, 1).withSize(2, 1);

    m_powerTab.addDouble("V/s (instant)", () -> m_voltageSlope)
        .withPosition(7, 1).withSize(2, 1);

    m_powerTab.addDouble("Min V Session", () -> m_minVoltageSession)
        .withPosition(3, 2).withSize(2, 1);

    m_powerTab.addDouble("V Drop", this::getVoltageDropSinceStart)
        .withPosition(5, 2).withSize(2, 1);

    m_powerTab.addDouble("Battery %", this::getBatteryHealthPercent)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .withPosition(0, 3).withSize(5, 1);

    m_powerTab.addDouble("Total Amps", () -> m_estimatedTotalCurrent)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 200.0))
        .withPosition(5, 3).withSize(4, 1);

    m_powerTab.addDouble("Peak Amps", () -> m_peakCurrentSession)
        .withPosition(0, 4).withSize(2, 1);
    m_powerTab.addInteger("Warnings", () -> m_warningCount)
        .withPosition(2, 4).withSize(2, 1);
    m_powerTab.addInteger("Criticals", () -> m_criticalCount)
        .withPosition(4, 4).withSize(2, 1);
    m_powerTab.addDouble("Session (s)", this::getSessionDuration)
        .withPosition(6, 4).withSize(3, 1);

    m_sbInit = true;
  }

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    double now = Timer.getFPGATimestamp();
    m_currentVoltage = RobotController.getBatteryVoltage();

    if (m_currentVoltage < m_minVoltageSession) m_minVoltageSession = m_currentVoltage;
    if (m_estimatedTotalCurrent > m_peakCurrentSession) m_peakCurrentSession = m_estimatedTotalCurrent;

    updateVoltageSlope(now);
    predictBrownout();
    updatePowerState();
    publishTelemetry();

    if (m_currentState != m_previousState) {
      System.out.println("[POWER] " + m_previousState + " -> " + m_currentState +
          " @ " + String.format("%.2f", m_currentVoltage) + "V");

      // Count state entries for analysis
      if (m_currentState == PowerState.WARNING) m_warningCount++;
      else if (m_currentState == PowerState.CRITICAL) m_criticalCount++;
      else if (m_currentState == PowerState.EMERGENCY) m_emergencyCount++;

      SmartDashboard.putString("Power State", m_currentState.toString());
      m_previousState = m_currentState;
    }

    // Always push key values to SmartDashboard for easy access
    SmartDashboard.putNumber("Battery V", m_currentVoltage);
    SmartDashboard.putNumber("Discharge V/s", m_voltageSlope);
    SmartDashboard.putNumber("Discharge V/s Long-Term", m_longTermSlope);
    SmartDashboard.putNumber("Discharge V/s Smoothed", m_smoothedSlope);
    SmartDashboard.putBoolean("Brownout Risk", m_brownoutImminent);
    SmartDashboard.putNumber("Battery %", getBatteryHealthPercent());
    SmartDashboard.putBoolean("Safe to Shoot", isSafeToShoot());
  }

  private void updateVoltageSlope(double now) {
    m_voltageHistory[m_historyIndex] = m_currentVoltage;
    m_timeHistory[m_historyIndex] = now;
    m_historyIndex = (m_historyIndex + 1) % HISTORY;
    if (m_historyIndex == 0) m_historyFilled = true;

    // Short-term slope (last ~10 samples, ~0.2s at 50Hz)
    if (m_historyFilled) {
      int oldest = m_historyIndex;
      int newest = (m_historyIndex - 1 + HISTORY) % HISTORY;
      double dV = m_voltageHistory[newest] - m_voltageHistory[oldest];
      double dT = m_timeHistory[newest] - m_timeHistory[oldest];
      if (dT > 0.01) m_voltageSlope = dV / dT;
    }

    // Long-term slope (entire session from start)
    double sessionTime = now - m_startTime;
    if (sessionTime > 5.0) {  // Wait 5 seconds to stabilize
      m_longTermSlope = (m_currentVoltage - m_startVoltage) / sessionTime;
    }

    // Smoothed slope for battery life prediction:
    // Use the more optimistic (slower discharge) of short-term vs long-term
    // This prevents over-reaction to brief current spikes
    if (sessionTime > 5.0) {
      // Both slopes are negative during discharge
      // More optimistic = less negative (closer to 0)
      double optimisticSlope = Math.max(m_voltageSlope, m_longTermSlope);

      // Apply exponential smoothing: 80% old + 20% new (dampens spikes)
      m_smoothedSlope = 0.8 * m_smoothedSlope + 0.2 * optimisticSlope;
    } else {
      m_smoothedSlope = m_voltageSlope;  // Use short-term until session stabilizes
    }
  }

  private void predictBrownout() {
    // Don't warn unless:
    // 1. Voltage is already at or below WARNING level (not just a transient sag)
    // 2. Slope is steep enough to be a real concern (not just noise)
    if (m_currentVoltage >= PowerConstants.kWarningVoltage || m_voltageSlope >= -0.3) {
      m_brownoutImminent = false;
      m_predictedTimeToBrownout = Double.MAX_VALUE;
      return;
    }

    double voltageToEmergency = m_currentVoltage - PowerConstants.kEmergencyVoltage;
    m_predictedTimeToBrownout = voltageToEmergency / (-m_voltageSlope);
    // 2 seconds is a reasonable warning window (was 0.5s - too reactive)
    m_brownoutImminent = m_predictedTimeToBrownout < 2.0;
  }

  private void updatePowerState() {
    if (m_currentVoltage > PowerConstants.kWarningVoltage) {
      m_currentState = PowerState.NOMINAL;
    } else if (m_currentVoltage > PowerConstants.kCriticalVoltage) {
      m_currentState = PowerState.WARNING;
    } else if (m_currentVoltage > PowerConstants.kEmergencyVoltage) {
      m_currentState = PowerState.CRITICAL;
    } else {
      m_currentState = PowerState.EMERGENCY;
    }

    if (m_brownoutImminent && m_currentState == PowerState.WARNING) {
      m_currentState = PowerState.CRITICAL;
    }
  }

  private void publishTelemetry() {
    m_voltagePub.set(m_currentVoltage);
    m_voltageSlopePub.set(m_voltageSlope);
    m_minVoltagePub.set(m_minVoltageSession);
    m_totalCurrentPub.set(m_estimatedTotalCurrent);
    m_maxTempPub.set(m_maxMotorTemp);
    m_brownoutTimePub.set(m_brownoutImminent ? m_predictedTimeToBrownout : -1.0);
    m_powerStatePub.set(m_currentState.toString());
  }

  // ===== GETTERS =====

  public PowerState getPowerState() { return m_currentState; }
  public boolean isNominal() { return m_currentState == PowerState.NOMINAL; }
  public boolean shouldConservePower() {
    return m_currentState == PowerState.CRITICAL || m_currentState == PowerState.EMERGENCY;
  }
  public boolean isBrownoutImminent() { return m_brownoutImminent; }
  public double getVoltage() { return m_currentVoltage; }
  public double getVoltageSlope() { return m_voltageSlope; }
  public double getLongTermSlope() { return m_longTermSlope; }
  public double getSmoothedSlope() { return m_smoothedSlope; }

  // Dashboard convenience methods
  public double getBatteryVoltage() { return m_currentVoltage; }
  public double getDischargeRate() { return m_voltageSlope; }
  public double getDischargeRateSmoothed() { return m_smoothedSlope; }
  public boolean isBrownoutRisk() { return m_brownoutImminent; }
  public double getMinVoltage() { return m_minVoltageSession; }
  public double getVoltageDrop() { return getVoltageDropSinceStart(); }
  public double getTotalCurrent() { return m_estimatedTotalCurrent; }
  public double getPeakCurrent() { return m_peakCurrentSession; }
  public double getMaxMotorTemp() { return m_maxMotorTemp; }
  public int getWarningCount() { return m_warningCount; }
  public int getCriticalCount() { return m_criticalCount; }

  /**
   * Rough estimate of remaining battery life in seconds.
   * Uses smoothed discharge rate (more optimistic, filters out spikes).
   */
  public double getEstimatedBatteryLife() {
    // Use smoothed slope (more stable, optimistic)
    if (m_smoothedSlope >= -0.001) return 999.0;  // Voltage stable or rising
    double voltageLeft = m_currentVoltage - PowerConstants.kEmergencyVoltage;
    return voltageLeft / (-m_smoothedSlope);
  }

  /**
   * Battery health as a percentage (13V=100%, 10.5V=0%).
   */
  public double getBatteryHealthPercent() {
    double full = 13.0;
    double empty = PowerConstants.kEmergencyVoltage;
    return Math.max(0, Math.min(100, (m_currentVoltage - empty) / (full - empty) * 100.0));
  }

  /**
   * True if battery voltage is high enough to safely fire a full-power shot.
   * If borderline, the shot could cause a brownout.
   */
  public boolean isSafeToShoot() {
    return m_currentVoltage > PowerConstants.kCriticalVoltage && !m_brownoutImminent;
  }

  /**
   * True if we're drawing dangerously high current.
   */
  public boolean isPowerBudgetExceeded() {
    return m_estimatedTotalCurrent > PowerConstants.kMaxSafeCurrentDraw;
  }

  /**
   * Voltage drop since this session started.
   */
  public double getVoltageDropSinceStart() {
    return m_startVoltage - m_currentVoltage;
  }

  /**
   * How long this session has been running (seconds).
   */
  public double getSessionDuration() {
    return Timer.getFPGATimestamp() - m_startTime;
  }

  /**
   * Session stats string for logging.
   */
  public String getSessionStats() {
    return String.format(
        "Min V: %.2fV | Drop: %.2fV | Peak Amps: %.1fA | Warnings: %d | Criticals: %d | State: %s",
        m_minVoltageSession, getVoltageDropSinceStart(), m_peakCurrentSession,
        m_warningCount, m_criticalCount, m_currentState);
  }

  // ===== POWER SCALING =====

  /** Intake power scale - reduced only in EMERGENCY. */
  public double getIntakePowerScale() {
    // Only throttle intake in EMERGENCY state
    if (m_currentState == PowerState.EMERGENCY) {
      return PowerConstants.kEmergencyModeIntakeScale;  // 75%
    }
    return 1.0;
  }

  /** Shooter is NEVER throttled. */
  public double getShooterPowerScale() { return 1.0; }

  /** Feeder is NEVER throttled. */
  public double getFeederPowerScale() { return 1.0; }

  /**
   * Vision is cut during EMERGENCY to save power/CPU.
   * Returns true if vision should be disabled.
   */
  public boolean shouldThrottleVision() {
    // Only cut vision in EMERGENCY state
    return m_currentState == PowerState.EMERGENCY;
  }

  /**
   * Partial vision throttle for lower half of CRITICAL state.
   * Returns true when voltage is between 10.5V and 10.75V.
   * In this mode, all cameras except one are disabled to save power/CPU.
   */
  public boolean shouldPartiallyThrottleVision() {
    // Lower half of CRITICAL: 10.5V to 10.75V (midpoint of 10.5-11.0 range)
    double criticalMidpoint = (PowerConstants.kCriticalVoltage + PowerConstants.kEmergencyVoltage) / 2.0;
    return m_currentVoltage >= PowerConstants.kEmergencyVoltage
        && m_currentVoltage < criticalMidpoint;
  }

  /** Swerve drive is NEVER throttled (critical for mobility). */
  public double getSwervePowerScale() { return 1.0; }

  // ===== UPDATERS (called by other subsystems) =====

  /** Pass in motor temps to track the hottest motor. */
  public void updateMotorTemp(double tempC) {
    if (tempC > m_maxMotorTemp) m_maxMotorTemp = tempC;
    m_thermalWarning = tempC > PowerConstants.kMotorTempWarning;
  }

  /** Pass in total current draw estimate. */
  public void updateTotalCurrent(double amps) {
    m_estimatedTotalCurrent = amps;
    if (amps > PowerConstants.kHighCurrentWarning) {
      System.out.println("[POWER] High current: " + String.format("%.1f", amps) + "A");
    }
  }

  /** Reset peak tracking (e.g. between matches). */
  public void resetPeaks() {
    m_peakCurrentSession = 0.0;
    m_maxMotorTemp = 0.0;
    m_warningCount = 0;
    m_criticalCount = 0;
    m_emergencyCount = 0;
    m_minVoltageSession = m_currentVoltage;
    m_startVoltage = m_currentVoltage;
    m_startTime = Timer.getFPGATimestamp();
    System.out.println("[POWER] Session stats reset");
  }
}
