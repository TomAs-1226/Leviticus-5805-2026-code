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
import com.ctre.phoenix6.signals.System_StateValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import java.util.Map;

/**
 * Controls the intake arm that swings down to grab game pieces.
 * Uses stall detection - when it hits the floor or top, it stops automatically.
 *
 * @author Baichen Yu
 */
public class IntakeDeploySubsystem extends SubsystemBase {

  public enum DeployState {
    UP,
    DOWN,
    MOVING_UP,
    MOVING_DOWN,
    UNKNOWN
  }

  private final TalonFX m_motor;
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0).withEnableFOC(true);

  private DeployState m_state = DeployState.UP;
  private boolean m_isMoving = false;

  // Stall detection stuff
  private boolean m_stallDetected = false;
  private double m_stallStartTime = 0.0;
  private boolean m_highCurrentDetected = false;

  // Shuffleboard
  private ShuffleboardTab m_tab;
  private boolean m_sbInit = false;

  public IntakeDeploySubsystem() {
    if (MotorConstants.kMotorGroup4CANBus.isEmpty()) {
      m_motor = new TalonFX(MotorConstants.kMotorGroup4MotorID);
    } else {
      m_motor = new TalonFX(MotorConstants.kMotorGroup4MotorID, MotorConstants.kMotorGroup4CANBus);
    }

    configureMotor();
    m_motor.setPosition(0);

    System.out.println("[DEPLOY] Intake arm ready - CAN ID " + MotorConstants.kMotorGroup4MotorID);
  }

  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = MotorConstants.kKrakenCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    MotorOutputConfigs motorOutput = new MotorOutputConfigs();
    motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput = motorOutput;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;

    for (int i = 0; i < 3; i++) {
      if (m_motor.getConfigurator().apply(config).isOK()) break;
    }
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    // ===== OVERVIEW TAB - Arm section (cols 3-4, rows 2-3) =====
    ShuffleboardTab overview = Shuffleboard.getTab("Overview");

    // Row 2: Current arm position state + up indicator
    overview.addString("Arm", () -> m_state.toString())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(3, 2).withSize(1, 1);

    overview.addBoolean("Arm UP", this::isUp)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(4, 2).withSize(1, 1);

    // Row 3: Down indicator + moving indicator
    overview.addBoolean("Arm DOWN", this::isDown)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(2, 3).withSize(1, 1);

    overview.addBoolean("Arm Moving", () -> m_isMoving)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "yellow", "color when false", "gray"))
        .withPosition(3, 3).withSize(1, 1);

    // ===== DETAILED INTAKE TAB - Deploy arm at bottom =====
    m_tab = Shuffleboard.getTab("Intake");

    // Arm position state - most important thing
    m_tab.addString("Arm State", () -> m_state.toString())
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 4).withSize(3, 1);

    // Moving indicator
    m_tab.addBoolean("Arm Moving", () -> m_isMoving)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "yellow", "color when false", "gray"))
        .withPosition(3, 4).withSize(2, 1);

    // Stall (means it hit the stop)
    m_tab.addBoolean("Stalled", () -> m_stallDetected)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "orange", "color when false", "gray"))
        .withPosition(5, 4).withSize(2, 1);

    // Up/down status boxes
    m_tab.addBoolean("Arm UP", this::isUp)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(0, 5).withSize(2, 1);

    m_tab.addBoolean("Arm DOWN", this::isDown)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(2, 5).withSize(2, 1);

    // Deploy amps bar
    m_tab.addDouble("Deploy Amps", this::getCurrentAmps)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0.0, "max", 80.0))
        .withPosition(4, 5).withSize(4, 1);

    // Deploy RPM
    m_tab.addDouble("Deploy RPM", this::getRPM)
        .withPosition(0, 6).withSize(3, 1);

    // Motor health
    m_tab.addBoolean("Deploy Motor OK", () -> m_motor.isAlive())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "red"))
        .withPosition(3, 6).withSize(2, 1);

    m_sbInit = true;
    System.out.println("[DEPLOY] Shuffleboard added to Intake tab");
  }

  /** Slap the intake down to the floor. Auto-stops when it hits. */
  public void deployDown() {
    if (m_state == DeployState.DOWN) return;

    m_state = DeployState.MOVING_DOWN;
    m_isMoving = true;
    m_stallDetected = false;
    m_highCurrentDetected = false;

    m_motor.setControl(m_dutyCycle.withOutput(MotorConstants.kDeployDownPowerPercent / 100.0));
    SmartDashboard.putString("Deploy", "GOING DOWN");
  }

  /** Bring the intake back up. Auto-stops when it hits the top. */
  public void deployUp() {
    if (m_state == DeployState.UP) return;

    m_state = DeployState.MOVING_UP;
    m_isMoving = true;
    m_stallDetected = false;
    m_highCurrentDetected = false;

    m_motor.setControl(m_dutyCycle.withOutput(-MotorConstants.kDeployUpPowerPercent / 100.0));
    SmartDashboard.putString("Deploy", "GOING UP");
  }

  /** Stop and lock position. */
  public void stop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isMoving = false;
  }

  /** Emergency stop - resets state. */
  public void forceStop() {
    m_motor.setControl(m_dutyCycle.withOutput(0));
    m_isMoving = false;
    m_state = DeployState.UNKNOWN;
  }

  public DeployState getState() { return m_state; }
  public boolean isDown() { return m_state == DeployState.DOWN; }
  public boolean isUp() { return m_state == DeployState.UP; }
  public boolean isMoving() { return m_isMoving; }
  public double getCurrentAmps() { return m_motor.getSupplyCurrent().getValueAsDouble(); }
  public double getRPM() { return Math.abs(m_motor.getVelocity().getValueAsDouble() * 60.0); }

  // Dashboard getters
  public boolean isStalled() { return m_stallDetected; }
  public boolean isMotorAlive() { return m_motor.isAlive(); }

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    double amps = getCurrentAmps();
    double rpm = getRPM();

    // Check for stall while moving
    if (m_isMoving) {
      boolean stalling = amps > MotorConstants.kDeployStallCurrentThreshold &&
                         rpm < MotorConstants.kDeployStallVelocityThreshold;

      if (stalling) {
        if (!m_highCurrentDetected) {
          m_highCurrentDetected = true;
          m_stallStartTime = Timer.getFPGATimestamp();
        } else if (Timer.getFPGATimestamp() - m_stallStartTime >= MotorConstants.kDeployStallTimeThreshold) {
          // Hit the stop!
          m_stallDetected = true;
          stop();

          if (m_state == DeployState.MOVING_DOWN) {
            m_state = DeployState.DOWN;
            System.out.println("[DEPLOY] Hit floor - locked DOWN");
          } else if (m_state == DeployState.MOVING_UP) {
            m_state = DeployState.UP;
            m_motor.setPosition(0);
            System.out.println("[DEPLOY] Hit top - locked UP");
          }
        }
      } else {
        m_highCurrentDetected = false;
      }
    }

    SmartDashboard.putString("Deploy State", m_state.toString());
    SmartDashboard.putNumber("Deploy Amps", amps);
    SmartDashboard.putBoolean("Deploy Moving", m_isMoving);
  }
}
