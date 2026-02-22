// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Auto-deploys intake arm then runs rollers. One button does it all.
 * Hold Square to deploy and intake, release to stop (arm stays down).
 * 
 * @author Baichen Yu
 */
public class AutoDeployIntakeCommand extends Command {

  private final IntakeSubsystem m_intake;
  private final IntakeDeploySubsystem m_deploy;
  private boolean m_deployStarted = false;

  public AutoDeployIntakeCommand(IntakeSubsystem intake, IntakeDeploySubsystem deploy) {
    m_intake = intake;
    m_deploy = deploy;
    addRequirements(intake, deploy);
  }

  @Override
  public void initialize() {
    m_deployStarted = false;

    // If arm isn't down, deploy it first
    if (!m_deploy.isDown()) {
      m_deploy.deployDown();
      m_deployStarted = true;
      SmartDashboard.putString("Auto Intake", "DEPLOYING...");
    } else {
      // Already down, just run intake
      m_intake.runIntake();
      SmartDashboard.putString("Auto Intake", "INTAKING");
    }
  }

  @Override
  public void execute() {
    // Wait for deploy to finish, then start intake
    if (m_deployStarted && m_deploy.isDown()) {
      m_intake.runIntake();
      m_deployStarted = false;
      SmartDashboard.putString("Auto Intake", "INTAKING");
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    // Don't retract - leave arm down for quick re-intake
    SmartDashboard.putString("Auto Intake", "STOPPED");
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until button released
  }
}
