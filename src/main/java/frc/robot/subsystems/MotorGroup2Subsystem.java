// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.ShooterConfig;
import java.util.Map;
import java.util.concurrent.atomic.DoubleAccumulator;

/**
 * Shooter - 4 Kraken X60 motors working together.
 * Once you start shooting, it idles at 500 RPM to save power on spinup.
 *
 * @author Baichen Yu
 */
public class MotorGroup2Subsystem extends SubsystemBase {

  private final TalonFX m_motor1; // Leader
  private final TalonFX m_motor2; // Follower
  private final TalonFX m_motor3; // Follower
  private final TalonFX m_motor4; // Follower
  
  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);
  // Shooter configuration (distance-to-RPM lookup table)
  private final ShooterConfig m_config = new ShooterConfig();
  private boolean m_configLoaded = false;

  private double m_targetRPM = MotorConstants.kMotorGroup2TargetRPM;
  private boolean m_idleActive = false;

  // Telemetry stats
  private double m_peakRPM = 0.0;
  private double m_peakCurrent = 0.0;
  private int m_spinupCount = 0;
  private boolean m_isReversing = false;

  // Shuffleboard
  private ShuffleboardTab m_tab;
  private boolean m_sbInit = false;

  public MotorGroup2Subsystem() {
    CANBus canBus = new CANBus(MotorConstants.kMotorGroup2CANBus);
    
    m_motor1 = new TalonFX(MotorConstants.kMotorGroup2Motor1ID, canBus);
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup2Motor2ID, canBus);
    m_motor3 = new TalonFX(MotorConstants.kMotorGroup2Motor3ID, canBus);
    m_motor4 = new TalonFX(MotorConstants.kMotorGroup2Motor4ID, canBus);

    configureTalonFX(m_motor1, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor2, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor3, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor4, InvertedValue.CounterClockwise_Positive);

    m_motor2.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));
    m_motor3.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));
    m_motor4.setControl(new Follower(MotorConstants.kMotorGroup2Motor1ID, MotorAlignmentValue.Aligned));

    // Load shooter configuration from JSON (happens once at startup)
    m_configLoaded = m_config.loadConfig();
    if (!m_configLoaded) {
      System.err.println("[SHOOTER] WARNING: Config not loaded, using fallback defaults");
    }

    SmartDashboard.putNumber("Shooter Target RPM", m_targetRPM);
    System.out.println("[SHOOTER] 4 motors ready - idle at " + MotorConstants.kShooterIdleRPM + " RPM");
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    // ===== OVERVIEW TAB - Shooter section at columns 5-9 =====
    ShuffleboardTab overview = Shuffleboard.getTab("Overview");

    // Row 0: Full-width avg RPM bar
    overview.addDouble("Avg RPM", this::getAverageRPM)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 6000.0))
        .withPosition(5, 0).withSize(5, 1);

    // Row 1: Status indicators
    // GREEN = under 3000, RED = over 3000
    overview.addBoolean("RPM HIGH", () -> getAverageRPM() >= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "green"))
        .withPosition(5, 1).withSize(1, 1);

    // Big red warning when shooter is running backwards
    overview.addBoolean("REVERSE!!", () -> m_isReversing)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "gray"))
        .withPosition(6, 1).withSize(2, 1);

    overview.addBoolean("Idle", () -> m_idleActive)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "yellow", "color when false", "gray"))
        .withPosition(8, 1).withSize(1, 1);

    overview.addBoolean("Motors OK", () ->
            m_motor1.isAlive() && m_motor2.isAlive() && m_motor3.isAlive() && m_motor4.isAlive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(9, 1).withSize(1, 1);

    // Row 2: Individual motor RPMs + target RPM
    overview.addDouble("M1", () -> m_motor1.getVelocity().getValueAsDouble() * 60.0)
        .withPosition(5, 2).withSize(1, 1);
    overview.addDouble("M2", () -> m_motor2.getVelocity().getValueAsDouble() * 60.0)
        .withPosition(6, 2).withSize(1, 1);
    overview.addDouble("M3", () -> m_motor3.getVelocity().getValueAsDouble() * 60.0)
        .withPosition(7, 2).withSize(1, 1);
    overview.addDouble("M4", () -> m_motor4.getVelocity().getValueAsDouble() * 60.0)
        .withPosition(8, 2).withSize(1, 1);
    overview.addDouble("Target RPM", () -> m_targetRPM)
        .withPosition(9, 2).withSize(1, 1);

    // Row 3: Per-motor color indicators (green = under 3k, red = over 3k) + RPM error
    overview.addBoolean("M1 OK", () -> m_motor1.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(5, 3).withSize(1, 1);
    overview.addBoolean("M2 OK", () -> m_motor2.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(6, 3).withSize(1, 1);
    overview.addBoolean("M3 OK", () -> m_motor3.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(7, 3).withSize(1, 1);
    overview.addBoolean("M4 OK", () -> m_motor4.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(8, 3).withSize(1, 1);
    overview.addDouble("RPM Err", () -> m_targetRPM - getAverageRPM())
        .withPosition(9, 3).withSize(1, 1);

    // Row 4: Full-width shooter amps bar
    overview.addDouble("Shooter Amps", this::getTotalCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 480.0))
        .withPosition(5, 4).withSize(5, 1);

    // ===== DETAILED SHOOTER TAB =====
    m_tab = Shuffleboard.getTab("Shooter");

    // Row 0-1: 4 RPM dials (2x2 each) + reverse warning
    m_tab.addDouble("M1 RPM", () -> m_motor1.getVelocity().getValueAsDouble() * 60.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", 6000.0, "show value", true))
        .withPosition(0, 0).withSize(2, 2);
    m_tab.addDouble("M2 RPM", () -> m_motor2.getVelocity().getValueAsDouble() * 60.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", 6000.0, "show value", true))
        .withPosition(2, 0).withSize(2, 2);
    m_tab.addDouble("M3 RPM", () -> m_motor3.getVelocity().getValueAsDouble() * 60.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", 6000.0, "show value", true))
        .withPosition(4, 0).withSize(2, 2);
    m_tab.addDouble("M4 RPM", () -> m_motor4.getVelocity().getValueAsDouble() * 60.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0.0, "max", 6000.0, "show value", true))
        .withPosition(6, 0).withSize(2, 2);

    // Reverse warning - big red box when running backwards
    m_tab.addBoolean("REVERSE!!", () -> m_isReversing)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "gray"))
        .withPosition(8, 0).withSize(2, 2);

    // Row 2: Avg RPM bar + color indicator (green = normal, red = over 3k) + per-motor colors
    m_tab.addDouble("Avg RPM", this::getAverageRPM)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 6000.0))
        .withPosition(0, 2).withSize(6, 1);

    m_tab.addBoolean("RPM HIGH", () -> getAverageRPM() >= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "red", "color when false", "green"))
        .withPosition(6, 2).withSize(2, 1);

    // Per-motor color indicators (green = under 3k, red = over 3k)
    m_tab.addBoolean("M1 OK", () -> m_motor1.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(0, 3).withSize(2, 1);
    m_tab.addBoolean("M2 OK", () -> m_motor2.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(2, 3).withSize(2, 1);
    m_tab.addBoolean("M3 OK", () -> m_motor3.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(4, 3).withSize(2, 1);
    m_tab.addBoolean("M4 OK", () -> m_motor4.getVelocity().getValueAsDouble() * 60.0 <= 3000)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(6, 3).withSize(2, 1);

    // Row 4: Per-motor amps
    m_tab.addDouble("M1 Amps", () -> m_motor1.getSupplyCurrent().getValueAsDouble())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 120.0))
        .withPosition(0, 4).withSize(2, 1);
    m_tab.addDouble("M2 Amps", () -> m_motor2.getSupplyCurrent().getValueAsDouble())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 120.0))
        .withPosition(2, 4).withSize(2, 1);
    m_tab.addDouble("M3 Amps", () -> m_motor3.getSupplyCurrent().getValueAsDouble())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 120.0))
        .withPosition(4, 4).withSize(2, 1);
    m_tab.addDouble("M4 Amps", () -> m_motor4.getSupplyCurrent().getValueAsDouble())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 120.0))
        .withPosition(6, 4).withSize(2, 1);

    // Row 5: Temps
    m_tab.addDouble("M1 Temp C", () -> m_motor1.getDeviceTemp().getValueAsDouble())
        .withPosition(0, 5).withSize(2, 1);
    m_tab.addDouble("M2 Temp C", () -> m_motor2.getDeviceTemp().getValueAsDouble())
        .withPosition(2, 5).withSize(2, 1);
    m_tab.addDouble("M3 Temp C", () -> m_motor3.getDeviceTemp().getValueAsDouble())
        .withPosition(4, 5).withSize(2, 1);
    m_tab.addDouble("M4 Temp C", () -> m_motor4.getDeviceTemp().getValueAsDouble())
        .withPosition(6, 5).withSize(2, 1);

    // Row 6: Summary
    m_tab.addDouble("Total Amps", this::getTotalCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 480.0))
        .withPosition(0, 6).withSize(4, 1);
    m_tab.addDouble("Target RPM", () -> m_targetRPM).withPosition(4, 6).withSize(2, 1);
    m_tab.addDouble("Peak RPM", () -> m_peakRPM).withPosition(6, 6).withSize(2, 1);

    // Row 7: Status
    m_tab.addBoolean("Idle Active", () -> m_idleActive)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "yellow", "color when false", "gray"))
        .withPosition(0, 7).withSize(2, 1);
    m_tab.addBoolean("All Alive", () ->
            m_motor1.isAlive() && m_motor2.isAlive() && m_motor3.isAlive() && m_motor4.isAlive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(2, 7).withSize(2, 1);
    m_tab.addDouble("Peak Amps", () -> m_peakCurrent).withPosition(4, 7).withSize(2, 1);
    m_tab.addInteger("Spinups", () -> m_spinupCount).withPosition(6, 7).withSize(2, 1);

    // Row 8: Config status (verify JSON loaded correctly)
    m_tab.addBoolean("Config OK", () -> m_configLoaded)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(0, 8).withSize(2, 1);
    m_tab.addString("Config Ver", () -> m_config.getVersion())
        .withPosition(2, 8).withSize(2, 1);
    m_tab.addInteger("Config Entries", () -> m_config.getEntryCount())
        .withPosition(4, 8).withSize(2, 1);
    m_tab.addString("Config Range", () ->
        String.format("%.1f-%.1fm", m_config.getMinDistance(), m_config.getMaxDistance()))
        .withPosition(6, 8).withSize(2, 1);

    m_sbInit = true;
    System.out.println("[SHOOTER] Shuffleboard initialized");
  }

  private void configureTalonFX(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.getMotorGroup2CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = inverted;
    motorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = MotorConstants.getMotorGroup2Kp();
    slot0.kV = MotorConstants.getMotorGroup2Kv();
    slot0.kS = MotorConstants.getMotorGroup2Ks();
    slot0.kA = MotorConstants.getMotorGroup2Ka();
    config.Slot0 = slot0;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = MotorConstants.kRampRate;

    for (int i = 0; i < 3; i++) {
      if (motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  /** Spin up shooter to dashboard RPM. */
  public void runForward() {
    m_targetRPM = SmartDashboard.getNumber("Shooter Target RPM", MotorConstants.kMotorGroup2TargetRPM);
    setShooterRPM(m_targetRPM);
  }

  /** Spin shooter backwards. */
  public void runReverse() {
    m_targetRPM = SmartDashboard.getNumber("Shooter Target RPM", MotorConstants.kMotorGroup2TargetRPM);
    setShooterRPM(-m_targetRPM);
  }

  /** Run at specific RPM (for vision control). */
  public void runAtRPM(double rpm) {
    setShooterRPM(rpm);
  }

  /**
   * Get recommended shooter RPM for a given distance.
   * Uses the loaded JSON config with linear interpolation.
   *
   * @param distanceMeters Distance to target in meters
   * @return Recommended RPM for this distance
   */
  public double getRPMForDistance(double distanceMeters) {
    if (!m_configLoaded) {
      System.err.println("[SHOOTER] Config not loaded, using default RPM");
      return MotorConstants.kMotorGroup2TargetRPM;
    }
    return m_config.getRPMForDistance(distanceMeters);
  }

  /**
   * Get recommended hood angle for a given distance.
   *
   * @param distanceMeters Distance to target in meters
   * @return Recommended hood angle in degrees
   */
  public double getHoodAngleForDistance(double distanceMeters) {
    if (!m_configLoaded) {
      return 25.0;  // Default angle
    }
    return m_config.getHoodAngleForDistance(distanceMeters);
  }

  /**
   * Run shooter at RPM calculated from distance.
   * This is the primary method for distance-based shooting.
   *
   * @param distanceMeters Distance to target in meters
   */
  public void runAtDistance(double distanceMeters) {
    double rpm = getRPMForDistance(distanceMeters);
    setShooterRPM(rpm);
    SmartDashboard.putNumber("Shooter/Distance", distanceMeters);
    SmartDashboard.putNumber("Shooter/CalculatedRPM", rpm);
  }

  /**
   * Get the shooter config (for debugging/display).
   */
  public ShooterConfig getConfig() {
    return m_config;
  }

  private void setShooterRPM(double rpm) {
    if (Math.abs(rpm) > 0 && !m_idleActive) {
      m_idleActive = true;
      m_spinupCount++;
    }

    m_isReversing = rpm < 0;
    m_targetRPM = Math.abs(rpm);
    m_motor1.setControl(m_velocityControl.withVelocity(rpm / 60.0));
  }

  /** Stop (goes to idle if active). */
  public void stopMotors() {
    if (m_idleActive && MotorConstants.kUseShooterIdle) {
      m_motor1.setControl(m_velocityControl.withVelocity(MotorConstants.kShooterIdleRPM / 60.0));
      SmartDashboard.putString("Shooter", "IDLE");
    } else {
      m_motor1.setControl(m_velocityControl.withVelocity(0));
      SmartDashboard.putString("Shooter", "STOPPED");
    }
  }

  /** Force full stop - bypasses idle. */
  public void forceStop() {
    m_motor1.setControl(m_velocityControl.withVelocity(0));
    m_idleActive = false;
    SmartDashboard.putString("Shooter", "FORCE STOPPED");
  }

  public boolean isReversing() { return m_isReversing; }

  public boolean isIdling() {
    return m_idleActive && getAverageRPM() < MotorConstants.kShooterIdleRPM + 100;
  }

  // Dashboard getters
  public double getTargetRPM() { return m_targetRPM; }
  public double getPeakRPM() { return m_peakRPM; }
  public double getPeakAmps() { return m_peakCurrent; }
  public int getSpinupCount() { return m_spinupCount; }
  public boolean isIdleActive() { return m_idleActive; }
  public boolean areAllMotorsAlive() {
    return m_motor1.isAlive() && m_motor2.isAlive() && m_motor3.isAlive() && m_motor4.isAlive();
  }
  public double getRPMError() { return m_targetRPM - getAverageRPM(); }

  /** Check if shooter is at target RPM (within 5% tolerance). */
  public boolean isAtTargetRPM() {
    if (m_targetRPM < 100) return false; // Not really targeting anything
    return Math.abs(getRPMError()) < (m_targetRPM * 0.05);
  }

  // Individual motor alive status
  public boolean isMotor1Alive() { return m_motor1.isAlive(); }
  public boolean isMotor2Alive() { return m_motor2.isAlive(); }
  public boolean isMotor3Alive() { return m_motor3.isAlive(); }
  public boolean isMotor4Alive() { return m_motor4.isAlive(); }

  public double[] getMotorRPMs() {
    return new double[] {
      m_motor1.getVelocity().getValueAsDouble() * 60.0,
      m_motor2.getVelocity().getValueAsDouble() * 60.0,
      m_motor3.getVelocity().getValueAsDouble() * 60.0,
      m_motor4.getVelocity().getValueAsDouble() * 60.0
    };
  }

  public double[] getMotorAmps() {
    return new double[] {
      m_motor1.getSupplyCurrent().getValueAsDouble(),
      m_motor2.getSupplyCurrent().getValueAsDouble(),
      m_motor3.getSupplyCurrent().getValueAsDouble(),
      m_motor4.getSupplyCurrent().getValueAsDouble()
    };
  }

  public double[] getMotorTemps() {
    return new double[] {
      m_motor1.getDeviceTemp().getValueAsDouble(),
      m_motor2.getDeviceTemp().getValueAsDouble(),
      m_motor3.getDeviceTemp().getValueAsDouble(),
      m_motor4.getDeviceTemp().getValueAsDouble()
    };
  }

  public double getAverageRPM() {
    return (m_motor1.getVelocity().getValueAsDouble() +
            m_motor2.getVelocity().getValueAsDouble() +
            m_motor3.getVelocity().getValueAsDouble() +
            m_motor4.getVelocity().getValueAsDouble()) * 15.0; // *60/4
  }

  public double getTotalCurrentAmps() {
    return m_motor1.getSupplyCurrent().getValueAsDouble() +
           m_motor2.getSupplyCurrent().getValueAsDouble() +
           m_motor3.getSupplyCurrent().getValueAsDouble() +
           m_motor4.getSupplyCurrent().getValueAsDouble();
  }

  public double getTotalAmps() { return getTotalCurrentAmps(); }

  public double getLeaderRPM() {
    return m_motor1.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    double avgRPM = getAverageRPM();
    double totalAmps = getTotalCurrentAmps();

    // Track peaks
    if (avgRPM > m_peakRPM) m_peakRPM = avgRPM;
    if (totalAmps > m_peakCurrent) m_peakCurrent = totalAmps;

    // Individual motors
    SmartDashboard.putNumber("Shooter M1 RPM", m_motor1.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter M2 RPM", m_motor2.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter M3 RPM", m_motor3.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter M4 RPM", m_motor4.getVelocity().getValueAsDouble() * 60.0);

    // Individual currents
    SmartDashboard.putNumber("Shooter M1 Amps", m_motor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M2 Amps", m_motor2.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M3 Amps", m_motor3.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M4 Amps", m_motor4.getSupplyCurrent().getValueAsDouble());

    // Temps
    SmartDashboard.putNumber("Shooter M1 Temp", m_motor1.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M2 Temp", m_motor2.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M3 Temp", m_motor3.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter M4 Temp", m_motor4.getDeviceTemp().getValueAsDouble());

    // Summary stats
    SmartDashboard.putNumber("Shooter Avg RPM", avgRPM);
    SmartDashboard.putNumber("Shooter Total Amps", totalAmps);
    SmartDashboard.putNumber("Shooter Target RPM", m_targetRPM);
    SmartDashboard.putNumber("Shooter Peak RPM", m_peakRPM);
    SmartDashboard.putNumber("Shooter Peak Amps", m_peakCurrent);
    SmartDashboard.putNumber("Shooter Spinup Count", m_spinupCount);
    SmartDashboard.putBoolean("Shooter Reversing", m_isReversing);
    SmartDashboard.putBoolean("Shooter Idle Active", m_idleActive);
    SmartDashboard.putBoolean("Shooter All Alive",
        m_motor1.isAlive() && m_motor2.isAlive() && m_motor3.isAlive() && m_motor4.isAlive());

    // RPM error (how far off from target)
    SmartDashboard.putNumber("Shooter RPM Error", m_targetRPM - avgRPM);

    // Config status (verify JSON loaded correctly)
    SmartDashboard.putBoolean("Shooter Config Loaded", m_configLoaded);
    SmartDashboard.putString("Shooter Config Version", m_config.getVersion());
    SmartDashboard.putNumber("Shooter Config Entries", m_config.getEntryCount());
  }
}
