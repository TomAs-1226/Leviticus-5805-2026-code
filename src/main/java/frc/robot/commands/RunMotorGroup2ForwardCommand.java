// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorGroup2Subsystem;

/**
 * Command to run Motor Group 2 forward at full speed while L1 trigger is held.
 */
public class RunMotorGroup2ForwardCommand extends Command {
  private final MotorGroup2Subsystem m_subsystem;

  public RunMotorGroup2ForwardCommand(MotorGroup2Subsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_subsystem.runForward();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
