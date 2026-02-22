// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem for AprilVision 3.2 AprilTag detection.
 * Supports up to 4 cameras for full field coverage.
 * Offsets tunable via SmartDashboard for accurate measurements.
 *
 * POWER MANAGEMENT: Vision is cut at EMERGENCY (< 10.5V) to save power/CPU.
 * This keeps swerve + shooter running when battery is critical.
 *
 * @author Baichen Yu
 */
public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera m_camera1;
  private final PhotonCamera m_camera2;
  private final PhotonCamera m_camera3;
  private final PhotonCamera m_camera4;
  private final boolean m_camera1Enabled;
  private final boolean m_camera2Enabled;
  private final boolean m_camera3Enabled;
  private final boolean m_camera4Enabled;

  // Power management (optional - vision cuts first during brownout)
  private PowerManagementSubsystem m_powerManagement = null;
  private boolean m_visionThrottled = false;  // Full throttle (all cameras off)
  private boolean m_partiallyThrottled = false;  // Partial throttle (only cam1 active)
  private int m_activeCameraInPartialMode = 1;  // Which camera is active during partial throttle

  // Camera offsets - tunable via SmartDashboard
  private double m_cam1X, m_cam1Y, m_cam1Z, m_cam1Roll, m_cam1Pitch, m_cam1Yaw;
  private double m_cam2X, m_cam2Y, m_cam2Z, m_cam2Roll, m_cam2Pitch, m_cam2Yaw;
  private double m_cam3X, m_cam3Y, m_cam3Z, m_cam3Roll, m_cam3Pitch, m_cam3Yaw;
  private double m_cam4X, m_cam4Y, m_cam4Z, m_cam4Roll, m_cam4Pitch, m_cam4Yaw;

  // Current detection state
  private List<Integer> m_detectedTagIds = new ArrayList<>();
  private boolean m_hasMotorGroup1Tags = false;
  private boolean m_hasMotorGroup2Tags = false;
  private boolean m_multiTagDetected = false;
  private int m_totalTagCount = 0;
  private boolean m_hasHubTags = false;
  private int m_cam1TagCount = 0;
  private int m_cam2TagCount = 0;
  private int m_cam3TagCount = 0;
  private int m_cam4TagCount = 0;

  // Distance tracking (meters)
  private double m_closestTagDistance = Double.MAX_VALUE;
  private double m_averageTagDistance = 0.0;

  // Best target data for logging
  private double m_bestYaw = 0.0;
  private double m_bestPitch = 0.0;
  private double m_bestArea = 0.0;
  private double m_bestSkew = 0.0;
  private double m_bestAmbiguity = 0.0;
  private int m_bestTagId = -1;

  // Logging control
  private int m_lastLoggedTagCount = -1;
  private String m_lastLoggedIds = "";
  private boolean m_cam1WasConnected = false;
  private boolean m_cam2WasConnected = false;
  private boolean m_cam3WasConnected = false;
  private boolean m_cam4WasConnected = false;
  private int m_logCooldown = 0;

  // Shuffleboard
  private boolean m_sbInit = false;

  public VisionSubsystem() {
    // Check which cameras are enabled (non-empty name)
    m_camera1Enabled = !VisionConstants.kCamera1Name.isEmpty();
    m_camera2Enabled = !VisionConstants.kCamera2Name.isEmpty();
    m_camera3Enabled = !VisionConstants.kCamera3Name.isEmpty();
    m_camera4Enabled = !VisionConstants.kCamera4Name.isEmpty();
    
    // Initialize cameras (only if enabled)
    m_camera1 = m_camera1Enabled ? new PhotonCamera(VisionConstants.kCamera1Name) : null;
    m_camera2 = m_camera2Enabled ? new PhotonCamera(VisionConstants.kCamera2Name) : null;
    m_camera3 = m_camera3Enabled ? new PhotonCamera(VisionConstants.kCamera3Name) : null;
    m_camera4 = m_camera4Enabled ? new PhotonCamera(VisionConstants.kCamera4Name) : null;

    // Initialize camera offsets from constants (tunable via SmartDashboard)
    m_cam1X = VisionConstants.kCam1OffsetX;
    m_cam1Y = VisionConstants.kCam1OffsetY;
    m_cam1Z = VisionConstants.kCam1OffsetZ;
    m_cam1Roll = VisionConstants.kCam1Roll;
    m_cam1Pitch = VisionConstants.kCam1Pitch;
    m_cam1Yaw = VisionConstants.kCam1Yaw;

    m_cam2X = VisionConstants.kCam2OffsetX;
    m_cam2Y = VisionConstants.kCam2OffsetY;
    m_cam2Z = VisionConstants.kCam2OffsetZ;
    m_cam2Roll = VisionConstants.kCam2Roll;
    m_cam2Pitch = VisionConstants.kCam2Pitch;
    m_cam2Yaw = VisionConstants.kCam2Yaw;

    m_cam3X = VisionConstants.kCam3OffsetX;
    m_cam3Y = VisionConstants.kCam3OffsetY;
    m_cam3Z = VisionConstants.kCam3OffsetZ;
    m_cam3Roll = VisionConstants.kCam3Roll;
    m_cam3Pitch = VisionConstants.kCam3Pitch;
    m_cam3Yaw = VisionConstants.kCam3Yaw;

    m_cam4X = VisionConstants.kCam4OffsetX;
    m_cam4Y = VisionConstants.kCam4OffsetY;
    m_cam4Z = VisionConstants.kCam4OffsetZ;
    m_cam4Roll = VisionConstants.kCam4Roll;
    m_cam4Pitch = VisionConstants.kCam4Pitch;
    m_cam4Yaw = VisionConstants.kCam4Yaw;

    // Push camera offsets to SmartDashboard for tuning
    SmartDashboard.putNumber("Vision/Cam1/OffsetX", m_cam1X);
    SmartDashboard.putNumber("Vision/Cam1/OffsetY", m_cam1Y);
    SmartDashboard.putNumber("Vision/Cam1/OffsetZ", m_cam1Z);
    SmartDashboard.putNumber("Vision/Cam1/Roll", m_cam1Roll);
    SmartDashboard.putNumber("Vision/Cam1/Pitch", m_cam1Pitch);
    SmartDashboard.putNumber("Vision/Cam1/Yaw", m_cam1Yaw);

    SmartDashboard.putNumber("Vision/Cam2/OffsetX", m_cam2X);
    SmartDashboard.putNumber("Vision/Cam2/OffsetY", m_cam2Y);
    SmartDashboard.putNumber("Vision/Cam2/OffsetZ", m_cam2Z);
    SmartDashboard.putNumber("Vision/Cam2/Roll", m_cam2Roll);
    SmartDashboard.putNumber("Vision/Cam2/Pitch", m_cam2Pitch);
    SmartDashboard.putNumber("Vision/Cam2/Yaw", m_cam2Yaw);

    SmartDashboard.putNumber("Vision/Cam3/OffsetX", m_cam3X);
    SmartDashboard.putNumber("Vision/Cam3/OffsetY", m_cam3Y);
    SmartDashboard.putNumber("Vision/Cam3/OffsetZ", m_cam3Z);
    SmartDashboard.putNumber("Vision/Cam3/Roll", m_cam3Roll);
    SmartDashboard.putNumber("Vision/Cam3/Pitch", m_cam3Pitch);
    SmartDashboard.putNumber("Vision/Cam3/Yaw", m_cam3Yaw);

    SmartDashboard.putNumber("Vision/Cam4/OffsetX", m_cam4X);
    SmartDashboard.putNumber("Vision/Cam4/OffsetY", m_cam4Y);
    SmartDashboard.putNumber("Vision/Cam4/OffsetZ", m_cam4Z);
    SmartDashboard.putNumber("Vision/Cam4/Roll", m_cam4Roll);
    SmartDashboard.putNumber("Vision/Cam4/Pitch", m_cam4Pitch);
    SmartDashboard.putNumber("Vision/Cam4/Yaw", m_cam4Yaw);

    System.out.println("===========================================");
    System.out.println("[VISION] VisionSubsystem initialized - up to 4 cameras");
    if (m_camera1Enabled) {
      System.out.println("[VISION] Camera 1: " + VisionConstants.kCamera1Name + " [ENABLED]");
    } else {
      System.out.println("[VISION] Camera 1: DISABLED");
    }
    if (m_camera2Enabled) {
      System.out.println("[VISION] Camera 2: " + VisionConstants.kCamera2Name + " [ENABLED]");
    } else {
      System.out.println("[VISION] Camera 2: DISABLED");
    }
    if (m_camera3Enabled) {
      System.out.println("[VISION] Camera 3: " + VisionConstants.kCamera3Name + " [ENABLED]");
    } else {
      System.out.println("[VISION] Camera 3: DISABLED");
    }
    if (m_camera4Enabled) {
      System.out.println("[VISION] Camera 4: " + VisionConstants.kCamera4Name + " [ENABLED]");
    } else {
      System.out.println("[VISION] Camera 4: DISABLED");
    }
    System.out.println("[VISION] Camera offsets tunable via SmartDashboard (Vision/CamX/)");
    System.out.println("[VISION] NOTE: Calibrate cameras in AprilVision 3.2 UI for best accuracy!");
    System.out.println("[VISION] POWER: Vision cuts at EMERGENCY to save CPU/bandwidth");
    System.out.println("===========================================");
  }

  /**
   * Set power management subsystem (optional).
   * If set, vision will be cut at EMERGENCY to save power/CPU.
   */
  public void setPowerManagement(PowerManagementSubsystem powerManagement) {
    m_powerManagement = powerManagement;
    System.out.println("[VISION] Power management enabled - vision cuts at EMERGENCY");
  }

  private void initShuffleboard() {
    if (m_sbInit) return;

    // ===== OVERVIEW TAB - Vision section (cols 0-4, rows 1-3) =====
    var overview = Shuffleboard.getTab("Overview");

    // Row 1: Cameras connected count (col 4)
    // Shows "CUT" when vision is disabled due to EMERGENCY
    // Shows "CAM1" when in partial throttle (only camera 1 active)
    overview.addString("Cams", () -> {
      if (m_visionThrottled) return "CUT";  // Vision cut due to EMERGENCY
      if (m_partiallyThrottled) return "CAM" + m_activeCameraInPartialMode;  // Partial throttle
      int count = 0;
      if (isCamera1Connected()) count++;
      if (isCamera2Connected()) count++;
      if (isCamera3Connected()) count++;
      if (isCamera4Connected()) count++;
      return String.valueOf(count);
    })
    .withWidget(BuiltInWidgets.kTextView)
    .withPosition(4, 1).withSize(1, 1);

    // Row 2: Hub locked (big indicator) + tag count
    overview.addBoolean("Hub", this::hasHubTags)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("color when true", "green", "color when false", "gray"))
        .withPosition(0, 2).withSize(2, 1);

    overview.addDouble("Tags", () -> (double) m_totalTagCount)
        .withPosition(2, 2).withSize(1, 1);

    // Row 3: Yaw to best target (0 = aligned)
    overview.addDouble("Yaw", this::getBestYaw)
        .withPosition(4, 3).withSize(1, 1);

    m_sbInit = true;
    System.out.println("[VISION] Shuffleboard added to Overview tab");
  }

  @Override
  public void periodic() {
    if (!m_sbInit) initShuffleboard();

    // Check if vision should be throttled to save power
    boolean shouldThrottle = m_powerManagement != null && m_powerManagement.shouldThrottleVision();
    boolean shouldPartialThrottle = m_powerManagement != null && m_powerManagement.shouldPartiallyThrottleVision();

    // Handle full throttle state changes
    if (shouldThrottle != m_visionThrottled) {
      m_visionThrottled = shouldThrottle;
      if (m_visionThrottled) {
        System.out.println("[VISION] CUT - EMERGENCY power state, disabling vision to save CPU/power");
      } else {
        System.out.println("[VISION] RESTORED - Battery recovered from EMERGENCY, re-enabling vision");
      }
    }

    // Handle partial throttle state changes
    if (shouldPartialThrottle != m_partiallyThrottled) {
      m_partiallyThrottled = shouldPartialThrottle;
      if (m_partiallyThrottled) {
        System.out.println("[VISION] PARTIAL - Lower CRITICAL state, keeping only camera " + m_activeCameraInPartialMode + " active");
      } else {
        System.out.println("[VISION] FULL POWER - Battery recovered, all cameras re-enabled");
      }
    }

    // If fully throttled, skip all processing (saves CPU + camera bandwidth)
    if (m_visionThrottled) {
      SmartDashboard.putBoolean("Vision/Throttled", true);
      SmartDashboard.putBoolean("Vision/PartialThrottle", false);
      SmartDashboard.putString("Vision/Status", "CUT (EMERGENCY)");
      return;  // Exit early, don't process cameras
    }

    // Update dashboard for partial throttle status
    SmartDashboard.putBoolean("Vision/Throttled", false);
    SmartDashboard.putBoolean("Vision/PartialThrottle", m_partiallyThrottled);
    SmartDashboard.putNumber("Vision/ActiveCamera", m_activeCameraInPartialMode);
    SmartDashboard.putString("Vision/Status", m_partiallyThrottled ? "PARTIAL (Cam " + m_activeCameraInPartialMode + ")" : "ACTIVE");

    // Read camera offsets from SmartDashboard (allows real-time tuning)
    m_cam1X = SmartDashboard.getNumber("Vision/Cam1/OffsetX", m_cam1X);
    m_cam1Y = SmartDashboard.getNumber("Vision/Cam1/OffsetY", m_cam1Y);
    m_cam1Z = SmartDashboard.getNumber("Vision/Cam1/OffsetZ", m_cam1Z);
    m_cam1Roll = SmartDashboard.getNumber("Vision/Cam1/Roll", m_cam1Roll);
    m_cam1Pitch = SmartDashboard.getNumber("Vision/Cam1/Pitch", m_cam1Pitch);
    m_cam1Yaw = SmartDashboard.getNumber("Vision/Cam1/Yaw", m_cam1Yaw);

    m_cam2X = SmartDashboard.getNumber("Vision/Cam2/OffsetX", m_cam2X);
    m_cam2Y = SmartDashboard.getNumber("Vision/Cam2/OffsetY", m_cam2Y);
    m_cam2Z = SmartDashboard.getNumber("Vision/Cam2/OffsetZ", m_cam2Z);
    m_cam2Roll = SmartDashboard.getNumber("Vision/Cam2/Roll", m_cam2Roll);
    m_cam2Pitch = SmartDashboard.getNumber("Vision/Cam2/Pitch", m_cam2Pitch);
    m_cam2Yaw = SmartDashboard.getNumber("Vision/Cam2/Yaw", m_cam2Yaw);

    m_cam3X = SmartDashboard.getNumber("Vision/Cam3/OffsetX", m_cam3X);
    m_cam3Y = SmartDashboard.getNumber("Vision/Cam3/OffsetY", m_cam3Y);
    m_cam3Z = SmartDashboard.getNumber("Vision/Cam3/OffsetZ", m_cam3Z);
    m_cam3Roll = SmartDashboard.getNumber("Vision/Cam3/Roll", m_cam3Roll);
    m_cam3Pitch = SmartDashboard.getNumber("Vision/Cam3/Pitch", m_cam3Pitch);
    m_cam3Yaw = SmartDashboard.getNumber("Vision/Cam3/Yaw", m_cam3Yaw);

    m_cam4X = SmartDashboard.getNumber("Vision/Cam4/OffsetX", m_cam4X);
    m_cam4Y = SmartDashboard.getNumber("Vision/Cam4/OffsetY", m_cam4Y);
    m_cam4Z = SmartDashboard.getNumber("Vision/Cam4/OffsetZ", m_cam4Z);
    m_cam4Roll = SmartDashboard.getNumber("Vision/Cam4/Roll", m_cam4Roll);
    m_cam4Pitch = SmartDashboard.getNumber("Vision/Cam4/Pitch", m_cam4Pitch);
    m_cam4Yaw = SmartDashboard.getNumber("Vision/Cam4/Yaw", m_cam4Yaw);

    // Clear previous detection state
    m_detectedTagIds.clear();
    m_hasMotorGroup1Tags = false;
    m_hasMotorGroup2Tags = false;
    m_hasHubTags = false;
    m_totalTagCount = 0;
    m_cam1TagCount = 0;
    m_cam2TagCount = 0;
    m_cam3TagCount = 0;
    m_cam4TagCount = 0;
    m_closestTagDistance = Double.MAX_VALUE;
    m_averageTagDistance = 0.0;
    m_bestTagId = -1;
    m_bestArea = 0.0;

    // Process enabled cameras only (fail-safe: if a camera can't connect, we just skip it)
    // In partial throttle mode, only process camera 1 to save power/CPU
    if (m_camera1Enabled && m_camera1 != null) {
      m_cam1TagCount = processCamera(m_camera1, "Cam1");
    }

    // Skip cameras 2-4 during partial throttle
    if (!m_partiallyThrottled) {
      if (m_camera2Enabled && m_camera2 != null) {
        m_cam2TagCount = processCamera(m_camera2, "Cam2");
      }
      if (m_camera3Enabled && m_camera3 != null) {
        m_cam3TagCount = processCamera(m_camera3, "Cam3");
      }
      if (m_camera4Enabled && m_camera4 != null) {
        m_cam4TagCount = processCamera(m_camera4, "Cam4");
      }
    }

    // Check for HUB tags
    for (int tagId : m_detectedTagIds) {
      if (isInArray(tagId, VisionConstants.kAllHubTagIDs)) {
        m_hasHubTags = true;
        break;
      }
    }

    // Check for multi-tag detection
    m_multiTagDetected = m_totalTagCount >= VisionConstants.kMultiTagThreshold;

    // Calculate average distance
    if (m_totalTagCount > 0 && m_averageTagDistance == 0.0) {
      m_averageTagDistance = m_closestTagDistance;
    }

    // Update SmartDashboard
    SmartDashboard.putNumber("Vision/TotalTagCount", m_totalTagCount);
    SmartDashboard.putNumber("Vision/Cam1TagCount", m_cam1TagCount);
    SmartDashboard.putNumber("Vision/Cam2TagCount", m_cam2TagCount);
    SmartDashboard.putNumber("Vision/Cam3TagCount", m_cam3TagCount);
    SmartDashboard.putNumber("Vision/Cam4TagCount", m_cam4TagCount);
    SmartDashboard.putBoolean("Vision/MultiTagDetected", m_multiTagDetected);
    SmartDashboard.putBoolean("Vision/HubDetected", m_hasHubTags);
    SmartDashboard.putBoolean("Vision/ShooterTrigger", m_hasMotorGroup2Tags);
    SmartDashboard.putBoolean("Vision/FeederTrigger", m_hasMotorGroup1Tags);
    SmartDashboard.putString("Vision/DetectedIDs", m_detectedTagIds.toString());
    SmartDashboard.putNumber("Vision/ClosestDistance", getClosestTagDistance());
    SmartDashboard.putNumber("Vision/BestYaw", m_bestYaw);
    SmartDashboard.putNumber("Vision/BestPitch", m_bestPitch);
    SmartDashboard.putNumber("Vision/BestArea", m_bestArea);
    SmartDashboard.putNumber("Vision/BestAmbiguity", m_bestAmbiguity);

    // Camera connection status
    SmartDashboard.putBoolean("Vision/Cam1Connected", isCamera1Connected());
    SmartDashboard.putBoolean("Vision/Cam2Connected", isCamera2Connected());
    SmartDashboard.putBoolean("Vision/Cam3Connected", isCamera3Connected());
    SmartDashboard.putBoolean("Vision/Cam4Connected", isCamera4Connected());

    // Log camera connection status changes (only for enabled cameras)
    logCameraStatus();

    // Decrement cooldown
    if (m_logCooldown > 0) {
      m_logCooldown--;
    }

    // Console logging - only when meaningful changes occur
    String currentIds = m_detectedTagIds.toString();
    boolean idsChanged = !currentIds.equals(m_lastLoggedIds);
    boolean countChanged = m_totalTagCount != m_lastLoggedTagCount;

    if (countChanged || (idsChanged && m_totalTagCount > 0)) {
      if (m_totalTagCount > 0) {
        System.out.println("-------------------------------------------");
        System.out.println("[VISION] TAGS DETECTED: " + m_totalTagCount);
        System.out.println("[VISION] IDs: " + m_detectedTagIds + (m_hasHubTags ? " [HUB]" : ""));
        System.out.println("[VISION] Best Tag ID: " + m_bestTagId);
        System.out.println("[VISION] Yaw: " + String.format("%.1f", m_bestYaw) + "° | Pitch: " + String.format("%.1f", m_bestPitch) + "°");
        System.out.println("[VISION] Area: " + String.format("%.2f", m_bestArea) + "% | Skew: " + String.format("%.1f", m_bestSkew) + "°");
        System.out.println("[VISION] Distance: " + String.format("%.2f", getClosestTagDistance()) + "m");
        System.out.println("[VISION] Ambiguity: " + String.format("%.3f", m_bestAmbiguity) +
            (m_bestAmbiguity > 0.2 ? " (HIGH)" : " (good)"));
        if (m_hasHubTags) {
          System.out.println("[VISION] >>> AUTOSHOOT READY <<<");
        }
        System.out.println("-------------------------------------------");
        m_logCooldown = 50; // ~1 second cooldown
      } else if (m_lastLoggedTagCount > 0 && m_logCooldown == 0) {
        System.out.println("[VISION] Tags lost");
      }
      m_lastLoggedTagCount = m_totalTagCount;
      m_lastLoggedIds = currentIds;
    }
  }

  private void logCameraStatus() {
    if (m_camera1Enabled && m_camera1 != null) {
      boolean cam1Connected = m_camera1.isConnected();
      if (cam1Connected != m_cam1WasConnected) {
        System.out.println("[VISION] " + VisionConstants.kCamera1Name + ": " +
            (cam1Connected ? "CONNECTED" : "DISCONNECTED"));
        m_cam1WasConnected = cam1Connected;
      }
    }
    if (m_camera2Enabled && m_camera2 != null) {
      boolean cam2Connected = m_camera2.isConnected();
      if (cam2Connected != m_cam2WasConnected) {
        System.out.println("[VISION] " + VisionConstants.kCamera2Name + ": " +
            (cam2Connected ? "CONNECTED" : "DISCONNECTED"));
        m_cam2WasConnected = cam2Connected;
      }
    }
    if (m_camera3Enabled && m_camera3 != null) {
      boolean cam3Connected = m_camera3.isConnected();
      if (cam3Connected != m_cam3WasConnected) {
        System.out.println("[VISION] " + VisionConstants.kCamera3Name + ": " +
            (cam3Connected ? "CONNECTED" : "DISCONNECTED"));
        m_cam3WasConnected = cam3Connected;
      }
    }
    if (m_camera4Enabled && m_camera4 != null) {
      boolean cam4Connected = m_camera4.isConnected();
      if (cam4Connected != m_cam4WasConnected) {
        System.out.println("[VISION] " + VisionConstants.kCamera4Name + ": " +
            (cam4Connected ? "CONNECTED" : "DISCONNECTED"));
        m_cam4WasConnected = cam4Connected;
      }
    }
  }

  private int processCamera(PhotonCamera camera, String cameraName) {
    if (camera == null || !camera.isConnected()) {
      return 0;
    }

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    int cameraTargetCount = 0;
    double totalDistance = 0.0;
    int distanceCount = 0;

    for (PhotonPipelineResult result : results) {
      if (result.hasTargets()) {
        List<PhotonTrackedTarget> targets = result.getTargets();

        for (PhotonTrackedTarget target : targets) {
          int tagId = target.getFiducialId();

          if (tagId > 0) {
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            double ambiguity = target.getPoseAmbiguity();

            // Calculate distance from 3D transform
            Transform3d transform = target.getBestCameraToTarget();
            double distance = 0.0;
            if (transform != null) {
              double x = transform.getX();
              double y = transform.getY();
              double z = transform.getZ();
              distance = Math.sqrt(x * x + y * y + z * z);
            }

            // Track best target (largest area = closest/clearest view)
            if (area > m_bestArea) {
              m_bestTagId = tagId;
              m_bestYaw = yaw;
              m_bestPitch = pitch;
              m_bestArea = area;
              m_bestSkew = skew;
              m_bestAmbiguity = ambiguity;
            }

            // Track distance
            if (distance > 0 && distance < m_closestTagDistance) {
              m_closestTagDistance = distance;
            }
            if (distance > 0) {
              totalDistance += distance;
              distanceCount++;
            }

            // Avoid counting duplicate tags
            if (!m_detectedTagIds.contains(tagId)) {
              m_detectedTagIds.add(tagId);
              m_totalTagCount++;
              cameraTargetCount++;

              // Check motor group triggers - ANY HUB tag triggers both
              if (isInArray(tagId, VisionConstants.kMotorGroup1TagIDs)) {
                m_hasMotorGroup1Tags = true;
              }
              if (isInArray(tagId, VisionConstants.kMotorGroup2TagIDs)) {
                m_hasMotorGroup2Tags = true;
              }

              SmartDashboard.putNumber("Vision/" + cameraName + "/TagID", tagId);
              SmartDashboard.putNumber("Vision/" + cameraName + "/Yaw", yaw);
              SmartDashboard.putNumber("Vision/" + cameraName + "/Pitch", pitch);
              SmartDashboard.putNumber("Vision/" + cameraName + "/Area", area);
              SmartDashboard.putNumber("Vision/" + cameraName + "/Distance", distance);
            }
          }
        }
      }
    }

    if (distanceCount > 0) {
      m_averageTagDistance = totalDistance / distanceCount;
    }

    SmartDashboard.putNumber("Vision/" + cameraName + "/TargetCount", cameraTargetCount);
    SmartDashboard.putBoolean("Vision/" + cameraName + "/Connected", camera.isConnected());

    return cameraTargetCount;
  }

  private boolean isInArray(int value, int[] array) {
    for (int element : array) {
      if (element == value) return true;
    }
    return false;
  }

  // Getters
  public boolean hasMotorGroup1Tags() { return m_hasMotorGroup1Tags; }
  public boolean hasMotorGroup2Tags() { return m_hasMotorGroup2Tags; }
  public boolean hasHubTags() { return m_hasHubTags; }
  public boolean isMultiTagDetected() { return m_multiTagDetected; }
  public int getTotalTagCount() { return m_totalTagCount; }
  public int getCam1TagCount() { return m_cam1TagCount; }
  public int getCam2TagCount() { return m_cam2TagCount; }
  public List<Integer> getDetectedTagIds() { return new ArrayList<>(m_detectedTagIds); }
  public boolean isTagDetected(int tagId) { return m_detectedTagIds.contains(tagId); }
  public boolean hasAnyTags() { return m_totalTagCount > 0; }

  public boolean isCamera1Connected() {
    return m_camera1Enabled && m_camera1 != null && m_camera1.isConnected();
  }

  public boolean isCamera2Connected() {
    return m_camera2Enabled && m_camera2 != null && m_camera2.isConnected();
  }

  public boolean isCamera3Connected() {
    return m_camera3Enabled && m_camera3 != null && m_camera3.isConnected();
  }

  public boolean isCamera4Connected() {
    return m_camera4Enabled && m_camera4 != null && m_camera4.isConnected();
  }

  public int getCam3TagCount() { return m_cam3TagCount; }
  public int getCam4TagCount() { return m_cam4TagCount; }

  /** Returns true if vision is currently throttled due to low battery. */
  public boolean isThrottled() { return m_visionThrottled; }

  public double getClosestTagDistance() {
    return m_closestTagDistance == Double.MAX_VALUE ? 0.0 : m_closestTagDistance;
  }

  public double getAverageTagDistance() { return m_averageTagDistance; }
  public double getBestYaw() { return m_bestYaw; }
  public double getBestPitch() { return m_bestPitch; }
  public double getBestArea() { return m_bestArea; }
  public double getBestAmbiguity() { return m_bestAmbiguity; }
  public int getBestTagId() { return m_bestTagId; }

  /**
   * How well aligned we are to the best visible target.
   * Based on yaw: under 3° = ALIGNED, under 10° = CLOSE, otherwise OFF.
   */
  public String getAlignmentStatus() {
    if (m_totalTagCount == 0) return "NO TAG";
    double yaw = Math.abs(m_bestYaw);
    if (yaw < 3.0) return "ALIGNED";
    if (yaw < 10.0) return "CLOSE";
    return "OFF";
  }

  // Dashboard convenience methods
  public boolean isCameraConnected(int camNumber) {
    switch (camNumber) {
      case 1: return isCamera1Connected();
      case 2: return isCamera2Connected();
      case 3: return isCamera3Connected();
      case 4: return isCamera4Connected();
      default: return false;
    }
  }

  public int getCameraTagCount(int camNumber) {
    switch (camNumber) {
      case 1: return m_cam1TagCount;
      case 2: return m_cam2TagCount;
      case 3: return m_cam3TagCount;
      case 4: return m_cam4TagCount;
      default: return 0;
    }
  }

  public String getStatus() {
    if (m_visionThrottled) return "CUT";
    if (m_partiallyThrottled) return "PARTIAL";
    return "ACTIVE";
  }

  public String getDetectedIDsString() { return m_detectedTagIds.toString(); }
  public boolean isHubDetected() { return m_hasHubTags; }
  public boolean isShooterTrigger() { return m_hasMotorGroup2Tags; }
  public boolean isFeederTrigger() { return m_hasMotorGroup1Tags; }
  public double getClosestDistance() { return getClosestTagDistance(); }
}
