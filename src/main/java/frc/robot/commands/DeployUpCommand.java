// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;

/**
 * D-pad Up - retract intake arm.
 *
 * @author Baichen Yu
 */
public class DeployUpCommand extends Command {

  private final IntakeDeploySubsystem m_deploy;

  public DeployUpCommand(IntakeDeploySubsystem deploy) {
    m_deploy = deploy;
    addRequirements(deploy);
  }

  @Override
  public void initialize() {
    m_deploy.deployUp();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted && m_deploy.isMoving()) {
      m_deploy.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return m_deploy.isUp();
  }
}
