// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
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
 * Command that controls motor groups based on AprilVision 3.2 AprilTag detection.
 * Features:
 * - Distance-based shooter RPM from JSON config (with area-based fallback)
 * - Prespin: Shooter spins up first, then feeder starts
 * - Anti-flicker: Keeps running for a short time if tag is briefly lost
 * - Target Lock Feedback: Controller rumbles when aligned
 * - Shot Counter/Limiter: Tracks shots fired
 * - Alignment HUD: Direction indicators on Shuffleboard
 * - Speed Profiles: Toggle between Lob Shot and Line Drive
 * - Best Tag Selection: Uses ambiguity and area for selection
 */
public class VisionControlCommand extends Command {

  private final VisionSubsystem m_vision;
  private final MotorGroup1Subsystem m_motorGroup1;
  private final MotorGroup2Subsystem m_motorGroup2;
  private final HoodSubsystem m_hood;
  private final CommandPS5Controller m_controller;
  private final TelemetrySubsystem m_telemetry;

  // Optional robot velocity supplier for predictive tracking
  private final Supplier<ChassisSpeeds> m_velocitySupplier;
  private final Supplier<Pose2d> m_poseSupplier;

  private boolean m_group1Running = false;
  private boolean m_group2Running = false;

  // ===== ADVANCED TECHNOLOGIES =====
  // Kalman filter for smoothing distance/yaw readings (accuracy-first design)
  private final VisionKalmanFilter m_distanceFilter = VisionKalmanFilter.forDistance();
  private final VisionKalmanFilter m_yawFilter = VisionKalmanFilter.forYaw();

  // Shot logger for match analysis and strategy optimization
  private final ShotLogger m_shotLogger = new ShotLogger();

  // Predictive tracker for shoot-on-the-move (only active when stable)
  private final PredictiveTracker m_predictiveTracker;

  // Enable/disable advanced features (can be toggled for safety)
  private boolean m_useKalmanFilter = true;
  private boolean m_usePredictiveTracking = true;
  private boolean m_useLeadCompensation = true;

  // Prespin timer
  private final Timer m_prespinTimer = new Timer();
  private boolean m_prespinComplete = false;

  // ===== SMART TAG MEMORY SYSTEM =====
  // Context-aware tag retention - holds last known target data when tag briefly disappears
  private static final double TAG_HOLD_TIME_SHOOTING = 0.25;  // 250ms when actively shooting (fast recovery)
  private static final double TAG_HOLD_TIME_SPINUP = 0.15;    // 150ms during spinup (tighter tolerance)
  private static final double TAG_HOLD_TIME_IDLE = 0.10;      // 100ms when idle (quick release)
  private double m_lastTagSeenTime = 0.0;
  private double m_lastKnownRPM = 0.0;
  private double m_lastKnownDistance = 0.0;
  private double m_lastKnownYaw = 0.0;
  private boolean m_hasEverSeenTag = false;

  // Shooting state awareness
  private enum ShootingState { IDLE, SPINUP, SHOOTING, COOLDOWN }
  private ShootingState m_shootingState = ShootingState.IDLE;
  private double m_stateChangeTime = 0.0;

  // ===== SPEED PROFILES =====
  private boolean m_lobShotMode = false;  // false = Line Drive (default), true = Lob Shot

  // ===== SHOT COUNTER =====
  private int m_shotCount = 0;
  private boolean m_wasShooting = false;  // Track shot transitions

  // ===== TARGET LOCK FEEDBACK =====
  private final Timer m_rumbleTimer = new Timer();
  private boolean m_isRumbling = false;

  // ===== SHUFFLEBOARD =====
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
   * Full constructor with robot velocity/pose suppliers for advanced features.
   *
   * @param vision Vision subsystem
   * @param motorGroup1 Feeder motor group
   * @param motorGroup2 Shooter motor group
   * @param hood Hood subsystem for angle adjustment
   * @param controller PS5 controller for feedback
   * @param telemetry Telemetry subsystem
   * @param velocitySupplier Supplier for current robot velocity (for lead compensation)
   * @param poseSupplier Supplier for current robot pose (for predictive tracking)
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

    // Initialize predictive tracker with HUB position (center of field for 2026 REBUILT)
    // HUB is at field center: ~8.2m x ~4.1m on a 16.5m x 8.2m field
    Translation2d hubPosition = new Translation2d(8.23, 4.11);
    m_predictiveTracker = new PredictiveTracker(hubPosition);

    // Disable predictive features if no velocity supplier
    if (velocitySupplier == null) {
      m_usePredictiveTracking = false;
      m_useLeadCompensation = false;
    }

    // Add requirements (hood is optional)
    if (hood != null) {
      addRequirements(vision, motorGroup1, motorGroup2, hood);
    } else {
      addRequirements(vision, motorGroup1, motorGroup2);
    }
  }

  private void initializeShuffleboard() {
    if (m_shuffleboardInitialized) return;

    m_visionTab = Shuffleboard.getTab("AutoShoot");

    // Alignment HUD - large text showing direction
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

    // Target lock indicator
    m_targetLockEntry = m_visionTab.add("TARGET LOCK", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
        .withPosition(0, 1)
        .withSize(2, 1)
        .getEntry();

    // Shot counter
    m_shotCountEntry = m_visionTab.add("Shots Fired", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", VisionConstants.kMaxShotsPerSession))
        .withPosition(2, 1)
        .withSize(2, 1)
        .getEntry();

    // Speed profile display
    m_speedProfileEntry = m_visionTab.add("Speed Profile", "LINE DRIVE")
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(2, 1)
        .getEntry();

    // Shooter RPM gauge
    m_shooterRPMEntry = m_visionTab.add("Shooter RPM", 0.0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 5000))
        .withPosition(2, 2)
        .withSize(2, 2)
        .getEntry();

    // Distance (from 3D transform) or Tag Area % (fallback)
    m_distanceEntry = m_visionTab.add("Distance (m)", 0.0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", 0, "max", 6))
        .withPosition(0, 3)
        .withSize(2, 1)
        .getEntry();

    m_shuffleboardInitialized = true;
    System.out.println("[AUTOSHOOT] Shuffleboard 'AutoShoot' tab initialized");
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

    // Reset advanced technology state
    m_distanceFilter.reset();
    m_yawFilter.reset();
    m_predictiveTracker.reset();

    // Initialize Shuffleboard on first run
    initializeShuffleboard();

    System.out.println("[AUTOSHOOT] Command started - waiting for HUB tags");
    System.out.println("[AUTOSHOOT] Speed Profile: " + (m_lobShotMode ? "LOB SHOT" : "LINE DRIVE"));
    System.out.println("[AUTOSHOOT] Session shots: " + m_shotCount);
    System.out.println("[AUTOSHOOT] Advanced features: Kalman=" + m_useKalmanFilter +
        " Predict=" + m_usePredictiveTracking + " Lead=" + m_useLeadCompensation);

    updateShuffleboardStatus("ACTIVE - Waiting");
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    boolean tagCurrentlyVisible = m_vision.hasHubTags() || m_vision.hasMotorGroup2Tags();
    double shooterRPM = m_lastKnownRPM;

    // ===== BEST TAG SELECTION (using distance or fallback to area) =====
    double area = m_vision.getBestArea();
    double ambiguity = m_vision.getBestAmbiguity();
    double rawYaw = m_vision.getBestYaw();
    double rawDistance = m_vision.getClosestTagDistance();

    // ===== KALMAN FILTER SMOOTHING (Accuracy-first) =====
    // Only smooths when data is consistent, resets on jumps
    double distance = rawDistance;
    double yaw = rawYaw;

    if (tagCurrentlyVisible && m_useKalmanFilter) {
      distance = m_distanceFilter.update(rawDistance);
      yaw = m_yawFilter.update(rawYaw);
    } else if (!tagCurrentlyVisible) {
      // Reset filters when target lost to avoid stale data
      m_distanceFilter.reset();
      m_yawFilter.reset();
    }

    // ===== PREDICTIVE TRACKING (when robot velocity available) =====
    double leadAngleOffset = 0.0;
    double rpmAdjustment = 1.0;

    if (tagCurrentlyVisible && m_usePredictiveTracking && m_velocitySupplier != null && m_poseSupplier != null) {
      ChassisSpeeds velocity = m_velocitySupplier.get();
      Pose2d pose = m_poseSupplier.get();

      // Update predictive tracker
      PredictiveTracker.AimResult aimResult = m_predictiveTracker.update(pose, velocity);

      // Only apply prediction if confidence is high enough
      if (aimResult.usingPrediction && aimResult.confidence > 0.7) {
        leadAngleOffset = aimResult.leadAngleDegrees;
        // Use predicted distance for RPM calculation
        distance = aimResult.distance;
      }

      // ===== AUTO-LEAD COMPENSATION =====
      if (m_useLeadCompensation) {
        AutoLeadCompensation.CompensationResult leadComp =
            AutoLeadCompensation.calculate(velocity, yaw, distance);

        if (leadComp.isCompensating) {
          leadAngleOffset += leadComp.angleOffsetDegrees;
          rpmAdjustment = 1.0 + leadComp.rpmAdjustmentPercent;
        }
      }
    }

    // Check if tag is currently visible and has good quality
    if (tagCurrentlyVisible) {
      // Only use tags with acceptable ambiguity
      if (ambiguity <= VisionConstants.kMaxAmbiguityThreshold || ambiguity == 0.0) {
        m_lastTagSeenTime = currentTime;
        m_hasEverSeenTag = true;

        // Prefer distance-based RPM (from JSON config) if distance is available
        // Otherwise fallback to area-based calculation
        if (distance > 0.01) {
          // Use distance-based shooting from JSON config
          shooterRPM = m_motorGroup2.getRPMForDistance(distance);
          // Apply RPM adjustment from lead compensation
          shooterRPM *= rpmAdjustment;
          m_lastKnownDistance = distance;  // Store for memory system

          // Auto-adjust hood angle based on distance (if hood is calibrated)
          if (m_hood != null && m_hood.isCalibrated()) {
            double hoodAngle = m_motorGroup2.getHoodAngleForDistance(distance);
            m_hood.setTargetAngle(hoodAngle);
            m_hood.setVisionControlEnabled(true);
          }

          if (m_shuffleboardInitialized) {
            m_distanceEntry.setDouble(distance);  // Show actual distance
          }
        } else {
          // Fallback to area-based calculation (legacy method)
          shooterRPM = calculateShooterRPMFromArea(area);
          if (m_shuffleboardInitialized) {
            m_distanceEntry.setDouble(area);  // Show area as fallback
          }
        }

        // Store last known values for smart tag memory system
        m_lastKnownRPM = shooterRPM;
        m_lastKnownYaw = yaw + leadAngleOffset;  // Include lead angle in stored yaw
      } else {
        // High ambiguity - log but don't use
        System.out.println("[AUTOSHOOT] Tag ignored - ambiguity too high: " +
            String.format("%.3f", ambiguity));
      }
    }

    // ===== ALIGNMENT HUD =====
    updateAlignmentHUD(tagCurrentlyVisible, yaw);

    // ===== TARGET LOCK FEEDBACK (Rumble) =====
    boolean isAligned = tagCurrentlyVisible &&
        Math.abs(yaw) <= VisionConstants.kAlignedYawThreshold;
    updateRumbleFeedback(isAligned);

    // Update target lock indicator
    if (m_shuffleboardInitialized) {
      m_targetLockEntry.setBoolean(isAligned);
    }

    // ===== SMART TAG MEMORY - Context-aware hold time =====
    // Use different hold times based on current shooting state
    double timeSinceLastSeen = currentTime - m_lastTagSeenTime;
    double contextHoldTime = getContextAwareHoldTime();
    boolean withinHoldTime = m_hasEverSeenTag && (timeSinceLastSeen < contextHoldTime);
    boolean shouldRun = tagCurrentlyVisible || withinHoldTime;

    // Update shooting state for context awareness
    updateShootingState(tagCurrentlyVisible, currentTime);

    // Control Motor Group 2 (Shooter)
    if (shouldRun && m_lastKnownRPM > 0) {
      if (!m_group2Running) {
        m_prespinTimer.reset();
        m_prespinTimer.start();
        m_prespinComplete = false;
        System.out.println("[AUTOSHOOT] Shooter spinning up @ " +
            String.format("%.0f", m_lastKnownRPM) + " RPM (" +
            (m_lobShotMode ? "LOB" : "LINE") + ")");
      }
      m_motorGroup2.runAtRPM(m_lastKnownRPM);
      m_group2Running = true;

      if (m_shuffleboardInitialized) {
        m_shooterRPMEntry.setDouble(m_lastKnownRPM);
      }

      // Check if prespin delay has elapsed
      if (m_prespinTimer.hasElapsed(VisionConstants.kPrespinDelaySeconds)) {
        if (!m_prespinComplete) {
          System.out.println("[AUTOSHOOT] Prespin complete - starting feeder!");
        }
        m_prespinComplete = true;
      }
    } else {
      if (m_group2Running) {
        m_motorGroup2.stopMotors();
        m_group2Running = false;
        m_prespinComplete = false;
        m_prespinTimer.reset();
        m_prespinTimer.stop();
        System.out.println("[AUTOSHOOT] Target lost - stopping");
      }
      if (m_shuffleboardInitialized) {
        m_shooterRPMEntry.setDouble(0.0);
      }
    }

    // Control Motor Group 1 (Feeder) - only after prespin complete
    boolean currentlyShooting = shouldRun && m_prespinComplete;
    if (currentlyShooting) {
      if (!m_group1Running) {
        System.out.println("[AUTOSHOOT] Feeder running - SHOOTING!");
      }
      m_motorGroup1.runMotors();
      m_group1Running = true;
    } else {
      if (m_group1Running) {
        m_motorGroup1.stopMotors();
        m_group1Running = false;
      }
    }

    // ===== SHOT COUNTER & LOGGING =====
    // Count a "shot" when feeder transitions from not running to running
    if (currentlyShooting && !m_wasShooting) {
      m_shotCount++;
      System.out.println("[AUTOSHOOT] Shot #" + m_shotCount + " fired!");

      // Record shot to telemetry for match analysis
      if (m_telemetry != null) {
        m_telemetry.recordShot(m_lastKnownRPM, area, yaw);
      }

      // Record to ShotLogger for detailed analysis (heat maps, strategy)
      double robotVelocity = 0.0;
      if (m_velocitySupplier != null) {
        ChassisSpeeds speeds = m_velocitySupplier.get();
        robotVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      }
      m_shotLogger.recordShot(m_lastKnownDistance, m_lastKnownRPM, m_lastKnownYaw, robotVelocity);
    }
    m_wasShooting = currentlyShooting;

    // Update shot count on Shuffleboard
    if (m_shuffleboardInitialized) {
      m_shotCountEntry.setInteger(m_shotCount);
    }

    // Update status
    updateShuffleboardStatus(
        m_group2Running ? (m_prespinComplete ? "SHOOTING" : "PRESPIN") : "WAITING");
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
      // Close to aligned
      if (yaw > 0) {
        arrow = "-> CLOSE";
        status = "TURN RIGHT";
      } else {
        arrow = "<- CLOSE";
        status = "TURN LEFT";
      }
    } else {
      // Far from aligned
      if (yaw > 0) {
        arrow = "-->>";
        status = "TURN RIGHT (" + String.format("%.1f", yaw) + "°)";
      } else {
        arrow = "<<--";
        status = "TURN LEFT (" + String.format("%.1f", Math.abs(yaw)) + "°)";
      }
    }

    m_alignmentArrowEntry.setString(arrow);
    m_alignmentStatusEntry.setString(status);
  }

  private void updateRumbleFeedback(boolean isAligned) {
    if (m_controller == null) return;

    if (isAligned) {
      // Pulse rumble when aligned
      if (!m_isRumbling) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble,
            VisionConstants.kRumbleIntensity);
        m_rumbleTimer.reset();
        m_rumbleTimer.start();
        m_isRumbling = true;
      } else if (m_rumbleTimer.hasElapsed(VisionConstants.kRumbleDuration)) {
        // Pulse off briefly then back on
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_rumbleTimer.reset();
        m_isRumbling = false;
      }
    } else {
      // Stop rumble when not aligned
      if (m_isRumbling) {
        m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        m_rumbleTimer.stop();
        m_isRumbling = false;
      }
    }
  }

  /**
   * Toggle between Lob Shot and Line Drive speed profiles.
   * Call this from a button binding.
   */
  public void toggleSpeedProfile() {
    m_lobShotMode = !m_lobShotMode;
    String profile = m_lobShotMode ? "LOB SHOT" : "LINE DRIVE";
    System.out.println("[AUTOSHOOT] Speed profile changed to: " + profile);

    if (m_shuffleboardInitialized) {
      m_speedProfileEntry.setString(profile);
    }
  }

  /**
   * Set speed profile directly.
   * @param lobShot true for Lob Shot, false for Line Drive
   */
  public void setSpeedProfile(boolean lobShot) {
    m_lobShotMode = lobShot;
    String profile = m_lobShotMode ? "LOB SHOT" : "LINE DRIVE";
    System.out.println("[AUTOSHOOT] Speed profile set to: " + profile);

    if (m_shuffleboardInitialized) {
      m_speedProfileEntry.setString(profile);
    }
  }

  /**
   * Get current shot count.
   */
  public int getShotCount() {
    return m_shotCount;
  }

  /**
   * Reset shot counter.
   */
  public void resetShotCount() {
    m_shotCount = 0;
    System.out.println("[AUTOSHOOT] Shot counter reset");

    if (m_shuffleboardInitialized) {
      m_shotCountEntry.setInteger(0);
    }
  }

  /**
   * Get context-aware hold time based on current shooting state.
   * When actively shooting, we hold longer to avoid interruption.
   * When idle, we release quickly to be responsive.
   */
  private double getContextAwareHoldTime() {
    switch (m_shootingState) {
      case SHOOTING:
        return TAG_HOLD_TIME_SHOOTING;  // 0.25s - maintain shot even if tag flickers
      case SPINUP:
        return TAG_HOLD_TIME_SPINUP;    // 0.15s - tighter tolerance during spinup
      case COOLDOWN:
        return TAG_HOLD_TIME_SHOOTING;  // 0.25s - still maintain during cooldown
      case IDLE:
      default:
        return TAG_HOLD_TIME_IDLE;      // 0.10s - quick release when not engaged
    }
  }

  /**
   * Update shooting state for context awareness.
   * Tracks IDLE -> SPINUP -> SHOOTING -> COOLDOWN -> IDLE transitions.
   */
  private void updateShootingState(boolean tagVisible, double currentTime) {
    ShootingState previousState = m_shootingState;

    // State machine for shooting context
    if (m_group1Running && m_prespinComplete) {
      // Feeder running + prespin done = actively shooting
      m_shootingState = ShootingState.SHOOTING;
    } else if (m_group2Running && !m_prespinComplete) {
      // Shooter spinning but not feeding = spinup
      m_shootingState = ShootingState.SPINUP;
    } else if (previousState == ShootingState.SHOOTING && !m_group1Running) {
      // Just stopped shooting = cooldown
      m_shootingState = ShootingState.COOLDOWN;
      m_stateChangeTime = currentTime;
    } else if (m_shootingState == ShootingState.COOLDOWN) {
      // Exit cooldown after a brief period
      if (currentTime - m_stateChangeTime > 0.3) {
        m_shootingState = ShootingState.IDLE;
      }
    } else if (!m_group2Running && !m_group1Running) {
      // Nothing running = idle
      m_shootingState = ShootingState.IDLE;
    }

    // Log state changes for debugging
    if (previousState != m_shootingState) {
      System.out.println("[AUTOSHOOT] State: " + previousState + " -> " + m_shootingState +
          " (hold time: " + String.format("%.0f", getContextAwareHoldTime() * 1000) + "ms)");
    }
  }

  /**
   * Calculate shooter RPM based on target area in frame.
   * Uses current speed profile (Lob Shot or Line Drive).
   */
  private double calculateShooterRPMFromArea(double area) {
    // Get RPM range based on current speed profile
    double minRPM, maxRPM;
    if (m_lobShotMode) {
      minRPM = VisionConstants.kLobShotMinRPM;
      maxRPM = VisionConstants.kLobShotMaxRPM;
    } else {
      minRPM = VisionConstants.kLineDriveMinRPM;
      maxRPM = VisionConstants.kLineDriveMaxRPM;
    }

    double clampedArea = Math.max(VisionConstants.kMinArea,
        Math.min(VisionConstants.kMaxArea, area));

    double areaRange = VisionConstants.kMaxArea - VisionConstants.kMinArea;
    double normalizedArea = (clampedArea - VisionConstants.kMinArea) / areaRange;

    double rpmRange = maxRPM - minRPM;
    double rpm = maxRPM - (normalizedArea * rpmRange);

    return rpm;
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

    // Disable vision control on hood
    if (m_hood != null) {
      m_hood.setVisionControlEnabled(false);
    }

    // Stop any rumble
    if (m_controller != null) {
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
    m_isRumbling = false;
    m_rumbleTimer.stop();

    System.out.println("[AUTOSHOOT] Command ended" + (interrupted ? " (interrupted)" : ""));
    System.out.println("[AUTOSHOOT] Total shots this session: " + m_shotCount);

    // Print shot logger summary for match analysis
    if (m_shotLogger.getTotalShots() > 0) {
      System.out.println(m_shotLogger.getSessionSummary());
    }

    // Reset filters for next run
    m_distanceFilter.reset();
    m_yawFilter.reset();
    m_predictiveTracker.reset();

    updateShuffleboardStatus("STOPPED");
  }

  // ===== ADVANCED FEATURE TOGGLES =====

  /**
   * Enable or disable Kalman filter smoothing.
   * Disable if you notice any tracking lag.
   */
  public void setKalmanFilterEnabled(boolean enabled) {
    m_useKalmanFilter = enabled;
    System.out.println("[AUTOSHOOT] Kalman filter: " + (enabled ? "ENABLED" : "DISABLED"));
  }

  /**
   * Enable or disable predictive tracking.
   * Only works if velocity supplier was provided.
   */
  public void setPredictiveTrackingEnabled(boolean enabled) {
    m_usePredictiveTracking = enabled && m_velocitySupplier != null;
    System.out.println("[AUTOSHOOT] Predictive tracking: " +
        (m_usePredictiveTracking ? "ENABLED" : "DISABLED"));
  }

  /**
   * Enable or disable auto-lead compensation.
   * Only works if velocity supplier was provided.
   */
  public void setLeadCompensationEnabled(boolean enabled) {
    m_useLeadCompensation = enabled && m_velocitySupplier != null;
    System.out.println("[AUTOSHOOT] Lead compensation: " +
        (m_useLeadCompensation ? "ENABLED" : "DISABLED"));
  }

  /**
   * Get shot logger for analysis access.
   */
  public ShotLogger getShotLogger() {
    return m_shotLogger;
  }

  /**
   * Get the most used shooting zone (for strategy).
   */
  public int getMostUsedShootingZone() {
    return m_shotLogger.getMostUsedZone();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
