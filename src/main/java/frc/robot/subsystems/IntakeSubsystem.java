// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import java.util.Map;

/**
 * Intake rollers - spins to grab or spit out game pieces.
 * Square = suck in, Circle = spit out. Hold the button to run.
 *
 * @author Baichen Yu
 */
public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_motor;
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  private boolean m_isRunning = false;
  private double m_currentPower = 0.0;
  private double m_totalRotations = 0.0;
  private double m_peakCurrent = 0.0;
  private double m_sessionStartTime = 0.0;

  private PowerManagementSubsystem m_powerManagement = null;

  // Shuffleboard
  private ShuffleboardTab m_tab;
  private boolean m_sbInit = false;

  public IntakeSubsystem() {
    if (MotorConstants.kMotorGroup3CANBus.isEmpty()) {
      m_motor = new TalonFX(MotorConstants.kMotorGroup3MotorID);
    } else {
      m_motor = new TalonFX(MotorConstants.kMotorGroup3MotorID, MotorConstants.kMotorGroup3CANBus);
    }

    configureMotor();
    m_sessionStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    System.out.println("[INTAKE] Rollers ready - CAN ID " + MotorConstants.kMotorGroup3MotorID);
  }

  public void setPowerManagement(PowerManagementSubsystem pm) {
    m_powerManagement = pm;
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    // ===== OVERVIEW TAB - Intake section (cols 0-4) =====
    ShuffleboardTab overview = Shuffleboard.getTab("Overview");

    // Row 1: Is it running?
    overview.addBoolean("Intake Running", () -> m_isRunning)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(2, 1).withSize(2, 1);

    // Row 3: Roller speed
    overview.addDouble("Roller RPM", this::getRPM)
        .withPosition(0, 3).withSize(2, 1);

    // Row 4: Amps bar
    overview.addDouble("Intake Amps", this::getCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 60.0))
        .withPosition(0, 4).withSize(5, 1);

    // ===== DETAILED INTAKE TAB =====
    m_tab = Shuffleboard.getTab("Intake");

    // RPM dial
    m_tab.addDouble("Roller RPM", this::getRPM)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", -6000.0, "max", 6000.0, "show value", true))
        .withPosition(0, 0).withSize(3, 3);

    m_tab.addDouble("Amps", this::getCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 60.0))
        .withPosition(3, 0).withSize(3, 1);

    m_tab.addBoolean("Running", () -> m_isRunning)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(6, 0).withSize(2, 1);

    m_tab.addBoolean("Motor OK", () -> m_motor.isAlive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(6, 1).withSize(2, 1);

    m_tab.addDouble("Voltage", () -> m_motor.getMotorVoltage().getValueAsDouble())
        .withPosition(3, 1).withSize(3, 1);

    m_tab.addDouble("Temp C", this::getMotorTemp)
        .withPosition(3, 2).withSize(3, 1);

    m_tab.addDouble("Power %", () -> m_currentPower * 100)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -100.0, "max", 100.0))
        .withPosition(0, 3).withSize(4, 1);

    m_tab.addDouble("Peak Amps", () -> m_peakCurrent)
        .withPosition(4, 3).withSize(2, 1);

    m_tab.addDouble("Total Rotations", () -> m_totalRotations)
        .withPosition(6, 3).withSize(2, 1);

    m_sbInit = true;
    System.out.println("[INTAKE] Shuffleboard initialized");
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kKraken44CurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorOutput;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

    for (int i = 0; i < 3; i++) {
      if (m_motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  /** Spin intake to grab game pieces. */
  public void runIntake() {
    double power = MotorConstants.kIntakePowerPercent / 100.0;
    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    m_motor.setControl(m_dutyCycle.withOutput(power));
    m_isRunning = true;
    SmartDashboard.putString("Intake", "SUCKING @ " + (int)(power * 100) + "%");
  }

  /** Spin intake backwards to spit out game pieces. */
  public void runReverse() {
    double power = -MotorConstants.kIntakeReversePowerPercent / 100.0;
    if (m_powerManagement != null) {
      power *= m_powerManagement.getIntakePowerScale();
    }

    m_currentPower = power;
    m_motor.setControl(m_dutyCycle.withOutput(power));
    m_isRunning = true;
    SmartDashboard.putString("Intake", "SPITTING @ " + (int)(Math.abs(power) * 100) + "%");
  }

  /** Run at custom power. Positive = intake, negative = eject. */
  public void runAtPower(double power) {
    m_currentPower = Math.max(-1.0, Math.min(1.0, power));
    m_motor.setControl(m_dutyCycle.withOutput(m_currentPower));
    m_isRunning = Math.abs(power) > 0.01;
  }

  /** Stop the rollers. */
  public void stop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isRunning = false;
    m_currentPower = 0.0;
    SmartDashboard.putString("Intake", "STOPPED");
  }

  public boolean isRunning() { return m_isRunning; }
  public double getCurrentAmps() { return m_motor.getSupplyCurrent().getValueAsDouble(); }
  public double getRPM() { return m_motor.getVelocity().getValueAsDouble() * 60.0; }
  public double getMotorTemp() { return m_motor.getDeviceTemp().getValueAsDouble(); }
  public double getTotalRotations() { return m_totalRotations; }
  public double getPeakCurrent() { return m_peakCurrent; }

  // Dashboard getters
  public boolean isMotorAlive() { return m_motor.isAlive(); }
  public double getPowerPercent() { return m_currentPower * 100.0; }
  public double getUptime() { return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_sessionStartTime; }

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    double amps = getCurrentAmps();
    double rpm = getRPM();
    double temp = getMotorTemp();
    double voltage = m_motor.getMotorVoltage().getValueAsDouble();

    // Track stats
    m_totalRotations += Math.abs(m_motor.getVelocity().getValueAsDouble()) * 0.02;
    if (amps > m_peakCurrent) m_peakCurrent = amps;

    // Telemetry
    SmartDashboard.putNumber("Intake RPM", rpm);
    SmartDashboard.putNumber("Intake Amps", amps);
    SmartDashboard.putNumber("Intake Temp C", temp);
    SmartDashboard.putNumber("Intake Voltage", voltage);
    SmartDashboard.putNumber("Intake Power %", m_currentPower * 100);
    SmartDashboard.putNumber("Intake Peak Amps", m_peakCurrent);
    SmartDashboard.putNumber("Intake Total Rotations", m_totalRotations);
    SmartDashboard.putBoolean("Intake Running", m_isRunning);
    SmartDashboard.putBoolean("Intake Alive", m_motor.isAlive());
    SmartDashboard.putNumber("Intake Uptime",
        edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - m_sessionStartTime);
  }
}
