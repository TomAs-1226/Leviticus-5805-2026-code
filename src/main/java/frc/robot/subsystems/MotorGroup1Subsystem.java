// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * Motor Group 1 subsystem: Two motors for toggle operation (X button) - Kraken X60
 * CAN ID 1 spins clockwise, CAN ID 2 spins counter-clockwise.
 * Uses open-loop DutyCycle control for direct power - no PID stalling.
 */
public class MotorGroup1Subsystem extends SubsystemBase {

  private final TalonFX m_motor1;
  private final TalonFX m_motor2;

  // Open-loop duty cycle control - direct power, no PID fighting
  // EnableFOC for better torque at low speeds
  private final DutyCycleOut m_motor1DutyCycle = new DutyCycleOut(0).withEnableFOC(true);
  private final DutyCycleOut m_motor2DutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  // Adjustable power from SmartDashboard (0.0 to 1.0 = 0% to 100%)
  private double m_powerPercent = MotorConstants.kMotorGroup1PowerPercent / 100.0;

  public MotorGroup1Subsystem() {
    m_motor1 = new TalonFX(MotorConstants.kMotorGroup1Motor1ID, new CANBus(MotorConstants.kMotorGroup2CANBus));
    m_motor2 = new TalonFX(MotorConstants.kMotorGroup1Motor2ID, new CANBus(MotorConstants.kMotorGroup2CANBus));

    // Reversed directions - Motor 1 clockwise, Motor 2 counter-clockwise
    configureTalonFX(m_motor1, InvertedValue.CounterClockwise_Positive);
    configureTalonFX(m_motor2, InvertedValue.CounterClockwise_Positive);

    // Initialize SmartDashboard controls - default from Constants
    SmartDashboard.putNumber("Group1 Power %", MotorConstants.kMotorGroup1PowerPercent);

    System.out.println("Motor Group 1 initialized with DutyCycle control");
    System.out.println("Default Power: " + MotorConstants.kMotorGroup1PowerPercent + "%");
    System.out.println("Current Limit: " + MotorConstants.getMotorGroup1CurrentLimit() + "A");
  }

  private void configureTalonFX(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limiting - cranked to 120A
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.getMotorGroup1CurrentLimit();
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    // Motor output configuration
    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = inverted;
    motorOutput.NeutralMode = MotorConstants.kUseBrakeMode ?
        NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput = motorOutput;

    // Open-loop ramp rate for smooth acceleration
    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = MotorConstants.kRampRate;

    for (int i = 0; i < 3; i++) {
      var status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        System.out.println("Config applied to motor " + motor.getDeviceID() + " on attempt " + (i + 1));
        break;
      }
      System.out.println("Config FAILED for motor " + motor.getDeviceID() + " attempt " + (i + 1) + ": " + status);
    }
  }

  public void runMotors() {
    // Read power from SmartDashboard (0-100%) and convert to 0.0-1.0
    m_powerPercent = SmartDashboard.getNumber("Group1 Power %", MotorConstants.kMotorGroup1PowerPercent) / 100.0;

    // Clamp to valid range
    m_powerPercent = Math.max(0.0, Math.min(1.0, m_powerPercent));

    // Apply duty cycle directly - full power, no PID fighting
    m_motor1.setControl(m_motor1DutyCycle.withOutput(m_powerPercent));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(m_powerPercent));

    // Show that motors are being commanded
    SmartDashboard.putBoolean("Group1 Running", true);
  }

  public void stopMotors() {
    m_motor1.setControl(m_motor1DutyCycle.withOutput(0));
    m_motor2.setControl(m_motor2DutyCycle.withOutput(0));
    SmartDashboard.putBoolean("Group1 Running", false);
  }

  // Dashboard getters
  public boolean isRunning() { return m_powerPercent > 0.01; }
  public boolean isMotor1Alive() { return m_motor1.isAlive(); }
  public boolean isMotor2Alive() { return m_motor2.isAlive(); }
  public double getMotor1RPM() { return m_motor1.getVelocity().getValueAsDouble() * 60.0; }
  public double getMotor2RPM() { return m_motor2.getVelocity().getValueAsDouble() * 60.0; }
  public double getMotor1Amps() { return m_motor1.getSupplyCurrent().getValueAsDouble(); }
  public double getMotor2Amps() { return m_motor2.getSupplyCurrent().getValueAsDouble(); }
  public double getPowerPercent() { return m_powerPercent; }

  @Override
  public void periodic() {
    // Display actual RPM
    SmartDashboard.putNumber("Group1 Motor1 RPM", m_motor1.getVelocity().getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Group1 Motor2 RPM", m_motor2.getVelocity().getValueAsDouble() * 60.0);

    // Display current draw (amps)
    SmartDashboard.putNumber("Group1 Motor1 Amps", m_motor1.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Group1 Motor2 Amps", m_motor2.getSupplyCurrent().getValueAsDouble());

    // Status
    SmartDashboard.putBoolean("Group1 Motor1 Alive", m_motor1.isAlive());
    SmartDashboard.putBoolean("Group1 Motor2 Alive", m_motor2.isAlive());
    SmartDashboard.putNumber("Group1 Power Applied", m_powerPercent * 100.0);
  }
}
