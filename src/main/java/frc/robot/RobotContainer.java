// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDeployIntakeCommand;
import frc.robot.commands.DeployDownCommand;
import frc.robot.commands.DeployUpCommand;
import frc.robot.commands.RunIntakeReverseCommand;
import frc.robot.commands.RunMotorGroup1Command;
import frc.robot.commands.RunMotorGroup2ForwardCommand;
import frc.robot.commands.RunMotorGroup2ReverseCommand;
import frc.robot.commands.VisionControlCommand;
import frc.robot.shuffleboard.RobotDashboard;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeDeploySubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MotorGroup1Subsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;
import frc.robot.subsystems.PowerManagementSubsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Main robot container - all subsystems and button bindings live here.
 *
 * @author Baichen Yu
 */
public class RobotContainer {

  // Subsystems
  private final MotorGroup1Subsystem m_feeder = new MotorGroup1Subsystem();
  private final MotorGroup2Subsystem m_shooter = new MotorGroup2Subsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final IntakeDeploySubsystem m_intakeDeploy = new IntakeDeploySubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final TelemetrySubsystem m_telemetry = new TelemetrySubsystem();
  private final PowerManagementSubsystem m_power = new PowerManagementSubsystem();

  // Controller
  private final CommandPS5Controller m_controller =
      new CommandPS5Controller(OperatorConstants.kDriverControllerPort);

  // Commands that need to persist
  private final VisionControlCommand m_visionCommand;

  // Custom Shuffleboard dashboard
  private final RobotDashboard m_dashboard;

  public RobotContainer() {
    // Wire up subsystems
    m_intake.setPowerManagement(m_power);
    m_vision.setPowerManagement(m_power);  // Vision throttles first during brownout
    m_telemetry.setSubsystems(m_feeder, m_shooter, m_vision, m_controller);
    m_telemetry.setPowerManagement(m_power);  // Enable battery/power telemetry

    m_visionCommand = new VisionControlCommand(
        m_vision, m_feeder, m_shooter, m_hood, m_controller, m_telemetry);

    // Initialize custom dashboard with all critical stats on one page
    m_dashboard = new RobotDashboard();

    configureBindings();
  }

  /**
   * Updates the custom Shuffleboard dashboard.
   * Call this from Robot.robotPeriodic() for real-time updates.
   */
  public void updateDashboard() {
    m_dashboard.update(m_intake, m_intakeDeploy, m_feeder, m_shooter, m_hood, m_vision, m_power);
  }

  private void configureBindings() {
    // Cross (X) - Feeder
    m_controller.cross()
        .whileTrue(new RunMotorGroup1Command(m_feeder));

    // L1/R1 - Shooter forward/reverse
    m_controller.L1()
        .whileTrue(new RunMotorGroup2ForwardCommand(m_shooter));
    m_controller.R1()
        .whileTrue(new RunMotorGroup2ReverseCommand(m_shooter));

    // Triangle - Vision auto-aim and shoot
    m_controller.triangle()
        .whileTrue(m_visionCommand);

    // Square - Auto-deploy + intake (deploys arm automatically!)
    m_controller.square()
        .whileTrue(new AutoDeployIntakeCommand(m_intake, m_intakeDeploy));

    // Circle - Eject (just runs rollers backwards)
    m_controller.circle()
        .whileTrue(new RunIntakeReverseCommand(m_intake));

    // D-pad - Manual arm control
    m_controller.povUp()
        .whileTrue(new DeployUpCommand(m_intakeDeploy));
    m_controller.povDown()
        .whileTrue(new DeployDownCommand(m_intakeDeploy));

    // L3 - Toggle shot profile (line drive vs lob)
    m_controller.L3()
        .onTrue(new InstantCommand(() -> m_visionCommand.toggleSpeedProfile()));

    // Options - Reset shot counter
    m_controller.options()
        .onTrue(new InstantCommand(() -> m_visionCommand.resetShotCount()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
