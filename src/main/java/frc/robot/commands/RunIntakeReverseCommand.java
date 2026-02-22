// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to run intake in reverse while Circle button is held.
 * Ejects game pieces from the robot.
 */
public class RunIntakeReverseCommand extends Command {
  private final IntakeSubsystem m_intake;

  public RunIntakeReverseCommand(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    System.out.println("[INTAKE] Eject started");
  }

  @Override
  public void execute() {
    m_intake.runReverse();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    System.out.println("[INTAKE] Eject stopped" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
