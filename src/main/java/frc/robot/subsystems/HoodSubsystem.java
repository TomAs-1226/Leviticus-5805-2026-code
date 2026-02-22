// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

import java.util.Map;

/**
 * Adjustable hood subsystem for changing shot angle.
 * Uses Kraken X44 with CANCoder (mag encoder) for absolute position.
 * Features:
 * - Motion Magic for smooth, jitter-free movement
 * - Hardstop calibration
 * - Anti-jitter filtering and deadband
 * - Vision-based automatic angle adjustment
 *
 * @author FRC 5805
 */
public class HoodSubsystem extends SubsystemBase {

  public enum HoodState {
    CALIBRATING_RETRACTED,  // Finding retracted hardstop
    CALIBRATING_EXTENDED,   // Finding extended hardstop
    CALIBRATED,             // Ready for use
    UNCALIBRATED,           // Needs calibration
    MOVING,                 // Actively moving to target
    AT_POSITION,            // At target position
    ERROR                   // Something wrong
  }

  private final TalonFX m_motor;
  private final CANcoder m_encoder;
  private final MotionMagicVoltage m_positionControl;
  private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

  // State tracking
  private HoodState m_state = HoodState.UNCALIBRATED;
  private boolean m_isCalibrated = false;

  // Calibrated position limits (set during calibration)
  private double m_retractedPosition = MotorConstants.kHoodRetractedPosition;
  private double m_extendedPosition = MotorConstants.kHoodExtendedPosition;

  // Current target and filtered target (anti-jitter)
  private double m_targetPosition = MotorConstants.kHoodDefaultPosition;
  private double m_filteredTarget = MotorConstants.kHoodDefaultPosition;
  private double m_lastAcceptedTarget = MotorConstants.kHoodDefaultPosition;
  private int m_settlingCounter = 0;
  private boolean m_visionControlEnabled = false;

  // Calibration state
  private double m_calibrationStartTime = 0.0;
  private boolean m_calibrationStallDetected = false;

  // Telemetry
  private double m_peakCurrent = 0.0;
  private double m_totalMovements = 0;

  // Shuffleboard
  private ShuffleboardTab m_tab;
  private boolean m_sbInit = false;

  public HoodSubsystem() {
    // Initialize motor and encoder
    if (MotorConstants.kHoodCANBus.isEmpty()) {
      m_motor = new TalonFX(MotorConstants.kHoodMotorID);
      m_encoder = new CANcoder(MotorConstants.kHoodEncoderID);
    } else {
      m_motor = new TalonFX(MotorConstants.kHoodMotorID, MotorConstants.kHoodCANBus);
      m_encoder = new CANcoder(MotorConstants.kHoodEncoderID, MotorConstants.kHoodCANBus);
    }

    // Configure encoder first
    configureEncoder();

    // Configure motor with Motion Magic
    configureMotor();

    // Position control with Motion Magic for smooth movement
    m_positionControl = new MotionMagicVoltage(0)
        .withSlot(0)
        .withEnableFOC(true);

    // Initialize target to current position
    m_filteredTarget = getPosition();
    m_targetPosition = m_filteredTarget;
    m_lastAcceptedTarget = m_filteredTarget;

    System.out.println("===========================================");
    System.out.println("[HOOD] Adjustable hood initialized");
    System.out.println("[HOOD] Motor CAN ID: " + MotorConstants.kHoodMotorID);
    System.out.println("[HOOD] Encoder CAN ID: " + MotorConstants.kHoodEncoderID);
    System.out.println("[HOOD] Range: " + MotorConstants.kHoodMinAngleDegrees + "° to " +
                       MotorConstants.kHoodMaxAngleDegrees + "°");
    System.out.println("[HOOD] Anti-jitter: deadband=" + MotorConstants.kHoodUpdateDeadband +
                       ", filter=" + MotorConstants.kHoodFilterAlpha);
    System.out.println("[HOOD] STATUS: UNCALIBRATED - Run calibration before use!");
    System.out.println("===========================================");
  }

  private void configureEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();

    // Mag encoder configuration
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    // Apply with retries
    for (int i = 0; i < 3; i++) {
      if (m_encoder.getConfigurator().apply(config).isOK()) {
        System.out.println("[HOOD] CANCoder configured successfully");
        break;
      }
    }
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limiting
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kHoodCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = MotorConstants.kHoodUseBrakeMode ?
        NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    // Use remote CANCoder as feedback source
    FeedbackConfigs feedback = new FeedbackConfigs();
    feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    feedback.FeedbackRemoteSensorID = MotorConstants.kHoodEncoderID;
    feedback.RotorToSensorRatio = 1.0;  // 1:1 if encoder is on output shaft
    config.Feedback = feedback;

    // Motion Magic configuration - smooth trapezoid motion profile
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicCruiseVelocity = MotorConstants.kHoodMaxVelocity;
    motionMagic.MotionMagicAcceleration = MotorConstants.kHoodMaxAcceleration;
    motionMagic.MotionMagicJerk = MotorConstants.kHoodMaxAcceleration * 10;  // Fast jerk limit
    config.MotionMagic = motionMagic;

    // PID + Feedforward for slot 0 (position control)
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = MotorConstants.kHoodKp;
    slot0.kI = MotorConstants.kHoodKi;
    slot0.kD = MotorConstants.kHoodKd;
    slot0.kV = MotorConstants.kHoodKv;
    slot0.kS = MotorConstants.kHoodKs;
    slot0.kA = MotorConstants.kHoodKa;
    slot0.kG = MotorConstants.kHoodKg;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;  // Gravity depends on angle
    config.Slot0 = slot0;

    // Apply with retries
    for (int i = 0; i < 3; i++) {
      if (m_motor.getConfigurator().apply(config).isOK()) {
        System.out.println("[HOOD] Motor configured with Motion Magic");
        break;
      }
    }
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    m_tab = Shuffleboard.getTab("Hood");

    // Position display
    m_tab.addDouble("Position", this::getPosition)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", -0.1, "max", 0.4, "showValue", true))
        .withPosition(0, 0).withSize(2, 2);

    m_tab.addDouble("Target", () -> m_targetPosition)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", -0.1, "max", 0.4, "showValue", true))
        .withPosition(2, 0).withSize(2, 2);

    // Status
    m_tab.addString("State", () -> m_state.name())
        .withPosition(4, 0).withSize(2, 1);

    m_tab.addBoolean("Calibrated", () -> m_isCalibrated)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
        .withPosition(4, 1).withSize(1, 1);

    m_tab.addBoolean("At Target", this::isAtTarget)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FFFF00"))
        .withPosition(5, 1).withSize(1, 1);

    // Angle display
    m_tab.addDouble("Angle (deg)", this::getAngleDegrees)
        .withPosition(0, 2).withSize(2, 1);

    m_tab.addDouble("Target Angle", this::getTargetAngleDegrees)
        .withPosition(2, 2).withSize(2, 1);

    // Motor stats
    m_tab.addDouble("Current (A)", this::getCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 40.0))
        .withPosition(0, 3).withSize(4, 1);

    m_tab.addDouble("Velocity (RPS)", this::getVelocity)
        .withPosition(4, 2).withSize(2, 1);

    // Calibration limits
    m_tab.addDouble("Retracted Pos", () -> m_retractedPosition)
        .withPosition(0, 4).withSize(2, 1);

    m_tab.addDouble("Extended Pos", () -> m_extendedPosition)
        .withPosition(2, 4).withSize(2, 1);

    m_tab.addBoolean("Vision Control", () -> m_visionControlEnabled)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#00FFFF", "colorWhenFalse", "#404040"))
        .withPosition(4, 3).withSize(2, 1);

    m_sbInit = true;
  }

  // ==================== POSITION CONTROL ====================

  /**
   * Set target position with anti-jitter filtering.
   * Small changes are filtered out to prevent oscillation.
   *
   * @param position Target position in rotations
   */
  public void setTargetPosition(double position) {
    // Clamp to valid range
    position = Math.max(m_retractedPosition, Math.min(m_extendedPosition, position));

    // Deadband check - ignore tiny changes
    double delta = Math.abs(position - m_lastAcceptedTarget);
    if (delta < MotorConstants.kHoodUpdateDeadband) {
      // Change too small, ignore
      return;
    }

    // Start settling counter for new target
    m_settlingCounter = MotorConstants.kHoodSettlingCycles;
    m_targetPosition = position;
  }

  /**
   * Set target angle in degrees (more intuitive).
   * @param angleDegrees Target angle (15° to 45°)
   */
  public void setTargetAngle(double angleDegrees) {
    // Clamp angle to valid range
    angleDegrees = Math.max(MotorConstants.kHoodMinAngleDegrees,
                            Math.min(MotorConstants.kHoodMaxAngleDegrees, angleDegrees));

    // Convert angle to position
    double position = angleToPosition(angleDegrees);
    setTargetPosition(position);
  }

  /**
   * Enable/disable vision-based automatic control.
   * When enabled, hood will automatically adjust based on vision distance.
   */
  public void setVisionControlEnabled(boolean enabled) {
    m_visionControlEnabled = enabled;
    if (enabled) {
      System.out.println("[HOOD] Vision control ENABLED");
    } else {
      System.out.println("[HOOD] Vision control DISABLED");
    }
  }

  /**
   * Set hood position based on distance from target.
   * Uses linear interpolation between min/max angles.
   *
   * @param distanceMeters Distance to target in meters
   */
  public void setPositionForDistance(double distanceMeters) {
    // Example mapping - adjust based on testing
    // Close (1m) = low angle (15°), Far (5m) = high angle (45°)
    double minDistance = 1.0;
    double maxDistance = 5.0;

    // Normalize distance to 0-1 range
    double t = (distanceMeters - minDistance) / (maxDistance - minDistance);
    t = Math.max(0.0, Math.min(1.0, t));

    // Interpolate angle
    double angle = MotorConstants.kHoodMinAngleDegrees +
                   t * (MotorConstants.kHoodMaxAngleDegrees - MotorConstants.kHoodMinAngleDegrees);

    setTargetAngle(angle);
  }

  // ==================== CALIBRATION ====================

  /**
   * Start calibration sequence. Finds both hardstops.
   * Call this on robot enable or button press.
   */
  public void startCalibration() {
    m_state = HoodState.CALIBRATING_RETRACTED;
    m_isCalibrated = false;
    m_calibrationStallDetected = false;
    m_calibrationStartTime = Timer.getFPGATimestamp();

    // Move slowly toward retracted position
    m_motor.setControl(m_dutyCycleControl.withOutput(-MotorConstants.kHoodCalibrationPower));

    System.out.println("[HOOD] Calibration started - finding retracted hardstop...");
  }

  /**
   * Cancel calibration and stop motor.
   */
  public void cancelCalibration() {
    if (m_state == HoodState.CALIBRATING_RETRACTED ||
        m_state == HoodState.CALIBRATING_EXTENDED) {
      m_motor.setControl(m_dutyCycleControl.withOutput(0));
      m_state = HoodState.UNCALIBRATED;
      System.out.println("[HOOD] Calibration cancelled");
    }
  }

  private void updateCalibration() {
    double current = getCurrentAmps();
    double velocity = Math.abs(getVelocity());
    double elapsed = Timer.getFPGATimestamp() - m_calibrationStartTime;

    // Check for stall (high current + low velocity)
    boolean stalling = current > MotorConstants.kHoodCalibrationStallCurrent &&
                       velocity < MotorConstants.kHoodCalibrationStallVelocity &&
                       elapsed > 0.3;  // Wait at least 0.3s before detecting stall

    if (m_state == HoodState.CALIBRATING_RETRACTED) {
      if (stalling) {
        // Found retracted hardstop
        m_motor.setControl(m_dutyCycleControl.withOutput(0));
        m_retractedPosition = getPosition();
        System.out.println("[HOOD] Retracted hardstop found at " +
                           String.format("%.4f", m_retractedPosition) + " rotations");

        // Now find extended hardstop
        m_state = HoodState.CALIBRATING_EXTENDED;
        m_calibrationStartTime = Timer.getFPGATimestamp();
        m_motor.setControl(m_dutyCycleControl.withOutput(MotorConstants.kHoodCalibrationPower));
        System.out.println("[HOOD] Finding extended hardstop...");
      }
    } else if (m_state == HoodState.CALIBRATING_EXTENDED) {
      if (stalling) {
        // Found extended hardstop
        m_motor.setControl(m_dutyCycleControl.withOutput(0));
        m_extendedPosition = getPosition();
        System.out.println("[HOOD] Extended hardstop found at " +
                           String.format("%.4f", m_extendedPosition) + " rotations");

        // Calibration complete!
        m_isCalibrated = true;
        m_state = HoodState.CALIBRATED;

        // Set motor position to match encoder (sync internal position)
        m_motor.setPosition(getPosition());

        // Move to default position
        double defaultPos = (m_retractedPosition + m_extendedPosition) / 2.0;
        m_targetPosition = defaultPos;
        m_filteredTarget = defaultPos;
        m_lastAcceptedTarget = defaultPos;

        System.out.println("===========================================");
        System.out.println("[HOOD] CALIBRATION COMPLETE");
        System.out.println("[HOOD] Range: " + String.format("%.4f", m_retractedPosition) +
                           " to " + String.format("%.4f", m_extendedPosition) + " rotations");
        System.out.println("[HOOD] Travel: " + String.format("%.4f",
                           m_extendedPosition - m_retractedPosition) + " rotations");
        System.out.println("===========================================");
      }
    }

    // Timeout safety
    if (elapsed > 5.0) {
      m_motor.setControl(m_dutyCycleControl.withOutput(0));
      m_state = HoodState.ERROR;
      System.err.println("[HOOD] Calibration TIMEOUT - check mechanism!");
    }
  }

  // ==================== GETTERS ====================

  public double getPosition() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getVelocity() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public double getCurrentAmps() {
    return m_motor.getSupplyCurrent().getValueAsDouble();
  }

  public double getMotorTemp() {
    return m_motor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Convert position to angle in degrees.
   */
  public double getAngleDegrees() {
    return positionToAngle(getPosition());
  }

  /**
   * Get target angle in degrees.
   */
  public double getTargetAngleDegrees() {
    return positionToAngle(m_targetPosition);
  }

  /**
   * Check if hood is at target position (within tolerance).
   */
  public boolean isAtTarget() {
    return Math.abs(getPosition() - m_targetPosition) < MotorConstants.kHoodPositionTolerance;
  }

  public boolean isCalibrated() {
    return m_isCalibrated;
  }

  public HoodState getState() {
    return m_state;
  }

  public boolean isVisionControlEnabled() {
    return m_visionControlEnabled;
  }

  public double getRetractedPosition() {
    return m_retractedPosition;
  }

  public double getExtendedPosition() {
    return m_extendedPosition;
  }

  // Dashboard getters
  public boolean isMotorAlive() {
    return m_motor.isAlive();
  }

  public boolean isEncoderConnected() {
    return m_encoder.isConnected();
  }

  // ==================== CONVERSION HELPERS ====================

  private double positionToAngle(double position) {
    // Linear interpolation from position to angle
    double range = m_extendedPosition - m_retractedPosition;
    if (Math.abs(range) < 0.001) return MotorConstants.kHoodMinAngleDegrees;

    double t = (position - m_retractedPosition) / range;
    t = Math.max(0.0, Math.min(1.0, t));

    return MotorConstants.kHoodMinAngleDegrees +
           t * (MotorConstants.kHoodMaxAngleDegrees - MotorConstants.kHoodMinAngleDegrees);
  }

  private double angleToPosition(double angleDegrees) {
    // Linear interpolation from angle to position
    double angleRange = MotorConstants.kHoodMaxAngleDegrees - MotorConstants.kHoodMinAngleDegrees;
    if (Math.abs(angleRange) < 0.01) return m_retractedPosition;

    double t = (angleDegrees - MotorConstants.kHoodMinAngleDegrees) / angleRange;
    t = Math.max(0.0, Math.min(1.0, t));

    return m_retractedPosition + t * (m_extendedPosition - m_retractedPosition);
  }

  // ==================== MANUAL CONTROL ====================

  /**
   * Nudge hood up (more arc).
   */
  public void nudgeUp() {
    double newTarget = m_targetPosition + 0.01;  // Small increment
    setTargetPosition(newTarget);
  }

  /**
   * Nudge hood down (flatter shot).
   */
  public void nudgeDown() {
    double newTarget = m_targetPosition - 0.01;  // Small decrement
    setTargetPosition(newTarget);
  }

  /**
   * Move to retracted position (flat shot).
   */
  public void goToRetracted() {
    setTargetPosition(m_retractedPosition);
  }

  /**
   * Move to extended position (high arc).
   */
  public void goToExtended() {
    setTargetPosition(m_extendedPosition);
  }

  /**
   * Move to middle position.
   */
  public void goToMiddle() {
    setTargetPosition((m_retractedPosition + m_extendedPosition) / 2.0);
  }

  /**
   * Stop motor (emergency).
   */
  public void stop() {
    m_motor.setControl(m_dutyCycleControl.withOutput(0));
    m_state = HoodState.AT_POSITION;
  }

  // ==================== PERIODIC ====================

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    // Update calibration if in progress
    if (m_state == HoodState.CALIBRATING_RETRACTED ||
        m_state == HoodState.CALIBRATING_EXTENDED) {
      updateCalibration();
      return;  // Don't do normal control during calibration
    }

    // Skip control if not calibrated
    if (!m_isCalibrated) {
      return;
    }

    // Anti-jitter: Settling counter
    if (m_settlingCounter > 0) {
      m_settlingCounter--;
      if (m_settlingCounter == 0) {
        // Accept the new target after settling
        m_lastAcceptedTarget = m_targetPosition;
        m_totalMovements++;
      }
    }

    // Anti-jitter: Low-pass filter on target
    m_filteredTarget = m_filteredTarget +
        MotorConstants.kHoodFilterAlpha * (m_lastAcceptedTarget - m_filteredTarget);

    // Apply position control with Motion Magic
    m_motor.setControl(m_positionControl.withPosition(m_filteredTarget));

    // Update state
    if (isAtTarget()) {
      m_state = HoodState.AT_POSITION;
    } else {
      m_state = HoodState.MOVING;
    }

    // Track peak current
    double current = getCurrentAmps();
    if (current > m_peakCurrent) m_peakCurrent = current;
  }
}
