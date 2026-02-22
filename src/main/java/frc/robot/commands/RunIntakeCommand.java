// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run intake forward while Square button is held.
 * Collects game pieces into the robot.
 */
public class RunIntakeCommand extends Command {
  private final IntakeSubsystem m_intake;

  public RunIntakeCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("[INTAKE] Intake started");
  }

  @Override
  public void execute() {
    m_intake.runIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    System.out.println("[INTAKE] Intake stopped" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
