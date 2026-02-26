// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.MotorGroup1Subsystem;
import frc.robot.subsystems.MotorGroup2Subsystem;
import frc.robot.subsystems.TelemetrySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.AutoLeadCompensation;
import frc.robot.vision.PredictiveTracker;
import frc.robot.vision.ShotLogger;
import frc.robot.vision.VisionKalmanFilter;
import java.util.Map;
import java.util.function.Supplier;

/**
 * Controls motor groups based on AprilTag detection for automated shooting.
 *
 * Features:
 * - Distance-based shooter RPM from JSON config (area-based fallback)
 * - Prespin with velocity verification before feeding
 * - Tag memory system for brief occlusions
 * - Controller rumble feedback on target lock
 * - Shot counting and logging
 * - Shuffleboard alignment HUD
 * - Lob Shot and Line Drive speed profiles
 */
public class VisionControlCommand extends Command {

  private final VisionSubsystem m_vision;
  private final MotorGroup1Subsystem m_motorGroup1;
  private final MotorGroup2Subsystem m_motorGroup2;
  private final HoodSubsystem m_hood;
  private final CommandPS5Controller m_controller;
  private final TelemetrySubsystem m_telemetry;

  private final Supplier<ChassisSpeeds> m_velocitySupplier;
  private final Supplier<Pose2d> m_poseSupplier;

  private boolean m_group1Running = false;
  private boolean m_group2Running = false;

  // Kalman filters for smoothing distance/yaw readings
  private final VisionKalmanFilter m_distanceFilter = VisionKalmanFilter.forDistance();
  private final VisionKalmanFilter m_yawFilter = VisionKalmanFilter.forYaw();

  private final ShotLogger m_shotLogger = new ShotLogger();
  private final PredictiveTracker m_predictiveTracker;

  // Feature toggles
  private boolean m_useKalmanFilter = true;
  private boolean m_usePredictiveTracking = true;
  private boolean m_useLeadCompensation = true;

  // Prespin state
  private final Timer m_prespinTimer = new Timer();
  private boolean m_prespinComplete = false;

  // Tag memory state
  private double m_lastTagSeenTime = 0.0;
  private double m_lastKnownRPM = 0.0;
  private double m_lastKnownDistance = 0.0;
  private double m_lastKnownYaw = 0.0;
  private boolean m_hasEverSeenTag = false;

  // Shooting state machine
  private enum ShootingState { IDLE, SPINUP, SHOOTING, COOLDOWN }
  private ShootingState m_shootingState = ShootingState.IDLE;
  private double m_stateChangeTime = 0.0;

  // Speed profile
  private boolean m_lobShotMode = false;

  // Shot counter
  private int m_shotCount = 0;
  private boolean m_wasShooting = false;

  // Rumble feedback
  private final Timer m_rumbleTimer = new Timer();
  private boolean m_isRumbling = false;

  // Shuffleboard
  private ShuffleboardTab m_visionTab;
  private GenericEntry m_alignmentArrowEntry;
  private GenericEntry m_alignmentStatusEntry;
  private GenericEntry m_shotCountEntry;
  private GenericEntry m_speedProfileEntry;
  private GenericEntry m_targetLockEntry;
  private GenericEntry m_shooterRPMEntry;
  private GenericEntry m_distanceEntry;
  private boolean m_shuffleboardInitialized = false;

  /**
   * Constructor without robot velocity/pose suppliers.
   * Predictive tracking and lead compensation will be disabled.
   */
  public VisionControlCommand(
      VisionSubsystem vision,
      MotorGroup1Subsystem motorGroup1,
      MotorGroup2Subsystem motorGroup2,
      HoodSubsystem hood,
      CommandPS5Controller controller,
      TelemetrySubsystem telemetry) {
    this(vision, motorGroup1, motorGroup2, hood, controller, telemetry, null, null);
  }

  /**
   * Full constructor with robot velocity/pose suppliers for predictive features.
   *
   * @param vision Vision subsystem
   * @param motorGroup1 Feeder motor group
   * @param motorGroup2 Shooter motor group
   * @param hood Hood subsystem for angle adjustment
   * @param controller PS5 controller for feedback
   * @param telemetry Telemetry subsystem
   * @param velocitySupplier Supplier for current robot velocity
   * @param poseSupplier Supplier for current robot pose
   */
  public VisionControlCommand(
      VisionSubsystem vision,
      MotorGroup1Subsystem motorGroup1,
      MotorGroup2Subsystem motorGroup2,
      HoodSubsystem hood,
      CommandPS5Controller controller,
      TelemetrySubsystem telemetry,
      Supplier<ChassisSpeeds> velocitySupplier,
      Supplier<Pose2d> poseSupplier) {
    m_vision = vision;
    m_motorGroup1 = motorGroup1;
    m_motorGroup2 = motorGroup2;
    m_hood = hood;
    m_controller = controller;
    m_telemetry = telemetry;
    m_velocitySupplier = velocitySupplier;
    m_poseSupplier = poseSupplier;

    // HUB position at field center for 2026 REBUILT
    Translation2d hubPosition = new Translation2d(8.23, 4.11);
    m_predictiveTracker = new PredictiveTracker(hubPosition);

    if (velocitySupplier == null) {
      m_usePredictiveTracking = false;
      m_useLeadCompensation = false;
    }

    if (hood != null) {
      addRequirements(vision, motorGroup1, motorGroup2, hood);
    } else {
      addRequirements(vision, motorGroup1, motorGroup2);
    }
  }

  private void initializeShuffleboard() {
    if (m_shuffleboardInitialized) return;

    m_visionTab = Shuffleboard.getTab("AutoShoot");

    m_alignmentArrowEntry = m_visionTab.add("Alignment", "---")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 0)
        .withSize(2, 1)
        .getEntry();

    m_alignmentStatusEntry = m_visionTab.add("Status", "WAITING")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(2, 0)
        .withSize(2, 1)
        .getEntry();

    m_targetLockEntry = m_visionTab.add("TARGET LOCK", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();

    m_shotCountEntry = m_visionTab.add("Shots Fired", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", VisionConstants.kMaxShotsPerSession))
        .withPosition(2, 1)
        .withSize(2, 1)
        .getEntry();

    m_speedProfileEntry = m_visionTab.add("Speed Profile", "LINE DRIVE")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(2, 1)
        .getEntry();

    m_shooterRPMEntry = m_visionTab.add("Shooter RPM", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 5000))
        .withPosition(2, 2)
        .withSize(2, 2)
        .getEntry();

    m_distanceEntry = m_visionTab.add("Distance (m)", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", 6))
        .withPosition(0, 3)
        .withSize(2, 1)
        .getEntry();

    m_shuffleboardInitialized = true;
    DriverStation.reportWarning("[AUTOSHOOT] Shuffleboard initialized", false);
  }

  @Override
  public void initialize() {
    m_group1Running = false;
    m_group2Running = false;
    m_prespinComplete = false;
    m_prespinTimer.reset();
    m_prespinTimer.stop();
    m_lastTagSeenTime = 0.0;
    m_lastKnownRPM = 0.0;
    m_hasEverSeenTag = false;
    m_rumbleTimer.reset();
    m_rumbleTimer.stop();
    m_isRumbling = false;

    m_distanceFilter.reset();
    m_yawFilter.reset();
    m_predictiveTracker.reset();

    initializeShuffleboard();

    DriverStation.reportWarning("[AUTOSHOOT] Started - Profile: " +
        (m_lobShotMode ? "LOB" : "LINE") + " | Shots: " + m_shotCount, false);

    updateShuffleboardStatus("ACTIVE - Waiting");
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();

    // Process vision data
    VisionData visionData = processVisionData(currentTime);

    // Update alignment HUD and feedback
    updateAlignmentHUD(visionData.tagVisible, visionData.yaw);
    updateTargetLockFeedback(visionData.tagVisible, visionData.yaw);

    // Determine if we should run motors
    boolean shouldRun = shouldRunMotors(currentTime, visionData.tagVisible);

    // Update shooting state machine
    updateShootingState(visionData.tagVisible, currentTime);

    // Control shooter and feeder
    controlShooter(shouldRun);
    boolean currentlyShooting = controlFeeder(shouldRun);

    // Track shots
    trackShots(currentlyShooting, visionData.yaw);

    // Update dashboard
    updateDashboard(shouldRun, currentlyShooting);
  }

  /**
   * Processes vision data including filtering and predictive tracking.
   */
  private VisionData processVisionData(double currentTime) {
    boolean tagVisible = m_vision.hasHubTags() || m_vision.hasMotorGroup2Tags();
    double area = m_vision.getBestArea();
    double ambiguity = m_vision.getBestAmbiguity();
    double rawYaw = m_vision.getBestYaw();
    double rawDistance = m_vision.getClosestTagDistance();

    double distance = rawDistance;
    double yaw = rawYaw;

    // Apply Kalman filtering
    if (tagVisible && m_useKalmanFilter) {
      distance = m_distanceFilter.update(rawDistance);
      yaw = m_yawFilter.update(rawYaw);
    } else if (!tagVisible) {
      m_distanceFilter.reset();
      m_yawFilter.reset();
    }

    // Apply predictive tracking
    double leadAngleOffset = 0.0;
    double rpmAdjustment = 1.0;

    if (tagVisible && m_usePredictiveTracking && m_velocitySupplier != null && m_poseSupplier != null) {
      ChassisSpeeds velocity = m_velocitySupplier.get();
      Pose2d pose = m_poseSupplier.get();

      PredictiveTracker.AimResult aimResult = m_predictiveTracker.update(pose, velocity);

      if (aimResult.usingPrediction && aimResult.confidence > VisionConstants.kPredictiveConfidenceThreshold) {
        leadAngleOffset = aimResult.leadAngleDegrees;
        distance = aimResult.distance;
      }

      if (m_useLeadCompensation) {
        AutoLeadCompensation.CompensationResult leadComp =
            AutoLeadCompensation.calculate(velocity, yaw, distance);

        if (leadComp.isCompensating) {
          leadAngleOffset += leadComp.angleOffsetDegrees;
          rpmAdjustment = 1.0 + leadComp.rpmAdjustmentPercent;
        }
      }
    }

    // Calculate RPM if tag visible with good quality
    if (tagVisible && (ambiguity <= VisionConstants.kMaxAmbiguityThreshold || ambiguity == 0.0)) {
      m_lastTagSeenTime = currentTime;
      m_hasEverSeenTag = true;

      double shooterRPM;
      if (distance > VisionConstants.kMinDistanceThreshold) {
        shooterRPM = m_motorGroup2.getRPMForDistance(distance) * rpmAdjustment;
        m_lastKnownDistance = distance;

        if (m_hood != null && m_hood.isCalibrated()) {
          double hoodAngle = m_motorGroup2.getHoodAngleForDistance(distance);
          m_hood.setTargetAngle(hoodAngle);
          m_hood.setVisionControlEnabled(true);
        }

        if (m_shuffleboardInitialized) {
          m_distanceEntry.setDouble(distance);
        }
      } else {
        shooterRPM = calculateShooterRPMFromArea(area);
        if (m_shuffleboardInitialized) {
          m_distanceEntry.setDouble(area);
        }
      }

      m_lastKnownRPM = shooterRPM;
      m_lastKnownYaw = yaw + leadAngleOffset;
    } else if (tagVisible) {
      DriverStation.reportWarning("[AUTOSHOOT] Tag ignored - ambiguity: " +
          String.format("%.3f", ambiguity), false);
    }

    return new VisionData(tagVisible, yaw);
  }

  /**
   * Determines if motors should run based on tag visibility and memory.
   */
  private boolean shouldRunMotors(double currentTime, boolean tagVisible) {
    double timeSinceLastSeen = currentTime - m_lastTagSeenTime;
    double holdTime = getContextAwareHoldTime();
    boolean withinHoldTime = m_hasEverSeenTag && (timeSinceLastSeen < holdTime);
    return tagVisible || withinHoldTime;
  }

  /**
   * Controls the shooter motors.
   */
  private void controlShooter(boolean shouldRun) {
    if (shouldRun && m_lastKnownRPM > 0) {
      if (!m_group2Running) {
        m_prespinTimer.reset();
        m_prespinTimer.start();
        m_prespinComplete = false;
        DriverStation.reportWarning("[AUTOSHOOT] Spinup @ " +
            String.format("%.0f", m_lastKnownRPM) + " RPM", false);
      }
      m_motorGroup2.runAtRPM(m_lastKnownRPM);
      m_group2Running = true;

      if (m_shuffleboardInitialized) {
        m_shooterRPMEntry.setDouble(m_lastKnownRPM);
      }

      // Check prespin complete with velocity verification
      if (!m_prespinComplete && m_prespinTimer.hasElapsed(VisionConstants.kPrespinDelaySeconds)) {
        if (isShooterAtSpeed()) {
          m_prespinComplete = true;
          DriverStation.reportWarning("[AUTOSHOOT] Prespin complete - shooter at speed", false);
        }
      }
    } else {
      if (m_group2Running) {
        m_motorGroup2.stopMotors();
        m_group2Running = false;
        m_prespinComplete = false;
        m_prespinTimer.reset();
        m_prespinTimer.stop();
        DriverStation.reportWarning("[AUTOSHOOT] Target lost - stopping", false);
      }
      if (m_shuffleboardInitialized) {
        m_shooterRPMEntry.setDouble(0.0);
      }
    }
  }

  /**
   * Controls the feeder motors. Returns true if currently shooting.
   */
  private boolean controlFeeder(boolean shouldRun) {
    boolean currentlyShooting = shouldRun && m_prespinComplete;

    if (currentlyShooting) {
      if (!m_group1Running) {
        DriverStation.reportWarning("[AUTOSHOOT] Feeder running - SHOOTING", false);
      }
      m_motorGroup1.runMotors();
      m_group1Running = true;
    } else {
      if (m_group1Running) {
        m_motorGroup1.stopMotors();
        m_group1Running = false;
      }
    }

    return currentlyShooting;
  }

  /**
   * Checks if shooter has reached target RPM within tolerance.
   */
  private boolean isShooterAtSpeed() {
    if (!VisionConstants.kRequireShooterAtSpeed) {
      return true;
    }

    double currentRPM = m_motorGroup2.getAverageRPM();
    double targetRPM = m_lastKnownRPM;
    double requiredRPM = targetRPM * VisionConstants.kShooterRPMTolerance;

    return currentRPM >= requiredRPM;
  }

  /**
   * Tracks shot events for telemetry and logging.
   */
  private void trackShots(boolean currentlyShooting, double yaw) {
    if (currentlyShooting && !m_wasShooting) {
      m_shotCount++;
      DriverStation.reportWarning("[AUTOSHOOT] Shot #" + m_shotCount + " fired", false);

      if (m_telemetry != null) {
        m_telemetry.recordShot(m_lastKnownRPM, m_vision.getBestArea(), yaw);
      }

      double robotVelocity = 0.0;
      if (m_velocitySupplier != null) {
        ChassisSpeeds speeds = m_velocitySupplier.get();
        robotVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      }
      m_shotLogger.recordShot(m_lastKnownDistance, m_lastKnownRPM, m_lastKnownYaw, robotVelocity);
    }
    m_wasShooting = currentlyShooting;
  }

  /**
   * Updates dashboard displays.
   */
  private void updateDashboard(boolean shouldRun, boolean currentlyShooting) {
    if (m_shuffleboardInitialized) {
      m_shotCountEntry.setInteger(m_shotCount);
    }

    String status;
    if (currentlyShooting) {
      status = "SHOOTING";
    } else if (m_group2Running) {
      status = isShooterAtSpeed() ? "READY" : "PRESPIN";
    } else {
      status = "WAITING";
    }
    updateShuffleboardStatus(status);
  }

  private void updateAlignmentHUD(boolean tagVisible, double yaw) {
    if (!m_shuffleboardInitialized) return;

    String arrow;
    String status;

    if (!tagVisible) {
      arrow = "---";
      status = "NO TARGET";
    } else if (Math.abs(yaw) <= VisionConstants.kAlignedYawThreshold) {
      arrow = ">>> LOCKED <<<";
      status = "ALIGNED";
    } else if (Math.abs(yaw) <= VisionConstants.kCloseYawThreshold) {
      if (yaw > 0) {
        arrow = "-> CLOSE";
        status = "TURN RIGHT";
      } else {
        arrow = "<- CLOSE";
        status = "TURN LEFT";
      }
    } else {
      if (yaw > 0) {
        arrow = "-->>";
        status = "TURN RIGHT (" + String.format("%.1f", yaw) + ")";
      } else {
        arrow = "<<--";
        status = "TURN LEFT (" + String.format("%.1f", Math.abs(yaw)) + ")";
      }
    }

    m_alignmentArrowEntry.setString(arrow);
    m_alignmentStatusEntry.setString(status);
  }

  private void updateTargetLockFeedback(boolean tagVisible, double yaw) {
    boolean isAligned = tagVisible && Math.abs(yaw) <= VisionConstants.kAlignedYawThreshold;

    if (m_shuffleboardInitialized) {
      m_targetLockEntry.setBoolean(isAligned);
    }

    updateRumbleFeedback(isAligned);
  }

  private void updateRumbleFeedback(boolean isAligned) {
    if (m_controller == null) return;

    if (isAligned) {
      if (!m_isRumbling) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, VisionConstants.kRumbleIntensity);
        m_rumbleTimer.reset();
        m_rumbleTimer.start();
        m_isRumbling = true;
      } else if (m_rumbleTimer.hasElapsed(VisionConstants.kRumbleDuration)) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_rumbleTimer.reset();
        m_isRumbling = false;
      }
    } else {
      if (m_isRumbling) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_rumbleTimer.stop();
        m_isRumbling = false;
      }
    }
  }

  /**
   * Toggle between Lob Shot and Line Drive speed profiles.
   */
  public void toggleSpeedProfile() {
    m_lobShotMode = !m_lobShotMode;
    String profile = m_lobShotMode ? "LOB SHOT" : "LINE DRIVE";
    DriverStation.reportWarning("[AUTOSHOOT] Profile: " + profile, false);

    if (m_shuffleboardInitialized) {
      m_speedProfileEntry.setString(profile);
    }
  }

  /**
   * Set speed profile directly.
   */
  public void setSpeedProfile(boolean lobShot) {
    m_lobShotMode = lobShot;
    String profile = m_lobShotMode ? "LOB SHOT" : "LINE DRIVE";
    DriverStation.reportWarning("[AUTOSHOOT] Profile set: " + profile, false);

    if (m_shuffleboardInitialized) {
      m_speedProfileEntry.setString(profile);
    }
  }

  public int getShotCount() {
    return m_shotCount;
  }

  public void resetShotCount() {
    m_shotCount = 0;
    DriverStation.reportWarning("[AUTOSHOOT] Shot counter reset", false);

    if (m_shuffleboardInitialized) {
      m_shotCountEntry.setInteger(0);
    }
  }

  private double getContextAwareHoldTime() {
    switch (m_shootingState) {
      case SHOOTING:
      case COOLDOWN:
        return VisionConstants.kTagHoldTimeShooting;
      case SPINUP:
        return VisionConstants.kTagHoldTimeSpinup;
      case IDLE:
      default:
        return VisionConstants.kTagHoldTimeIdle;
    }
  }

  private void updateShootingState(boolean tagVisible, double currentTime) {
    ShootingState previousState = m_shootingState;

    if (m_group1Running && m_prespinComplete) {
      m_shootingState = ShootingState.SHOOTING;
    } else if (m_group2Running && !m_prespinComplete) {
      m_shootingState = ShootingState.SPINUP;
    } else if (previousState == ShootingState.SHOOTING && !m_group1Running) {
      m_shootingState = ShootingState.COOLDOWN;
      m_stateChangeTime = currentTime;
    } else if (m_shootingState == ShootingState.COOLDOWN) {
      if (currentTime - m_stateChangeTime > VisionConstants.kCooldownDuration) {
        m_shootingState = ShootingState.IDLE;
      }
    } else if (!m_group2Running && !m_group1Running) {
      m_shootingState = ShootingState.IDLE;
    }

    if (previousState != m_shootingState) {
      DriverStation.reportWarning("[AUTOSHOOT] State: " + previousState + " -> " + m_shootingState, false);
    }
  }

  private double calculateShooterRPMFromArea(double area) {
    double minRPM, maxRPM;
    if (m_lobShotMode) {
      minRPM = VisionConstants.kLobShotMinRPM;
      maxRPM = VisionConstants.kLobShotMaxRPM;
    } else {
      minRPM = VisionConstants.kLineDriveMinRPM;
      maxRPM = VisionConstants.kLineDriveMaxRPM;
    }

    double clampedArea = Math.max(VisionConstants.kMinArea, Math.min(VisionConstants.kMaxArea, area));
    double areaRange = VisionConstants.kMaxArea - VisionConstants.kMinArea;
    double normalizedArea = (clampedArea - VisionConstants.kMinArea) / areaRange;
    double rpmRange = maxRPM - minRPM;

    return maxRPM - (normalizedArea * rpmRange);
  }

  private void updateShuffleboardStatus(String status) {
    if (m_shuffleboardInitialized) {
      m_alignmentStatusEntry.setString(status);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_motorGroup1.stopMotors();
    m_motorGroup2.stopMotors();
    m_group1Running = false;
    m_group2Running = false;
    m_prespinComplete = false;
    m_prespinTimer.stop();
    m_hasEverSeenTag = false;

    if (m_hood != null) {
      m_hood.setVisionControlEnabled(false);
    }

    if (m_controller != null) {
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
    m_isRumbling = false;
    m_rumbleTimer.stop();

    DriverStation.reportWarning("[AUTOSHOOT] Ended" + (interrupted ? " (interrupted)" : "") +
        " | Shots: " + m_shotCount, false);

    if (m_shotLogger.getTotalShots() > 0) {
      DriverStation.reportWarning(m_shotLogger.getSessionSummary(), false);
    }

    m_distanceFilter.reset();
    m_yawFilter.reset();
    m_predictiveTracker.reset();

    updateShuffleboardStatus("STOPPED");
  }

  public void setKalmanFilterEnabled(boolean enabled) {
    m_useKalmanFilter = enabled;
    DriverStation.reportWarning("[AUTOSHOOT] Kalman: " + (enabled ? "ON" : "OFF"), false);
  }

  public void setPredictiveTrackingEnabled(boolean enabled) {
    m_usePredictiveTracking = enabled && m_velocitySupplier != null;
    DriverStation.reportWarning("[AUTOSHOOT] Predictive: " +
        (m_usePredictiveTracking ? "ON" : "OFF"), false);
  }

  public void setLeadCompensationEnabled(boolean enabled) {
    m_useLeadCompensation = enabled && m_velocitySupplier != null;
    DriverStation.reportWarning("[AUTOSHOOT] Lead comp: " +
        (m_useLeadCompensation ? "ON" : "OFF"), false);
  }

  public ShotLogger getShotLogger() {
    return m_shotLogger;
  }

  public int getMostUsedShootingZone() {
    return m_shotLogger.getMostUsedZone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Container for processed vision data.
   */
  private static class VisionData {
    final boolean tagVisible;
    final double yaw;

    VisionData(boolean tagVisible, double yaw) {
      this.tagVisible = tagVisible;
      this.yaw = yaw;
    }
  }
}
