// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.TelemetryConstants;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Telemetry subsystem for collecting and publishing robot metrics.
 *
 * AUTO-DISCOVERS motors on the CAN bus - just wire them in and they'll be logged!
 *
 * Data Categories:
 * - Odometry: Robot position on field (X, Y, heading) - requires Pigeon 2
 * - Motors: Auto-discovered TalonFX motors (RPM, current, temp, voltage)
 * - Vision: Tag detections, areas, alignment
 * - System: Battery voltage, CAN utilization, loop times
 * - Events: Shots fired, mode changes, errors
 * - Match: Game phase, time remaining, alliance
 */
public class TelemetrySubsystem extends SubsystemBase {

  // ===== PIGEON 2 IMU =====
  private Pigeon2 m_pigeon;
  private boolean m_pigeonAvailable = false;
  private boolean m_pigeonChecked = false;

  // ===== ODOMETRY STATE =====
  private double m_robotX = TelemetryConstants.kDefaultStartX;
  private double m_robotY = TelemetryConstants.kDefaultStartY;
  private double m_robotHeadingDeg = TelemetryConstants.kDefaultStartHeadingDeg;

  // ===== AUTO-DISCOVERED MOTORS =====
  private final Map<Integer, TalonFX> m_discoveredMotors = new HashMap<>();
  private final Map<Integer, MotorPublishers> m_motorPublishers = new HashMap<>();
  private boolean m_motorScanComplete = false;
  private boolean m_motorScanStarted = false;
  private int m_currentScanId = 0;
  private int m_currentBusIndex = 0;
  private double m_startupTime = 0.0;
  private static final double SCAN_DELAY_SECONDS = 3.0;  // Wait 3s after startup before scanning

  // ===== SUBSYSTEM REFERENCES =====
  private VisionSubsystem m_vision;
  private PowerManagementSubsystem m_power;
  private CommandPS5Controller m_controller;

  // ===== NETWORKTABLES =====
  private final NetworkTableInstance m_ntInstance;
  private final NetworkTable m_motorTable;

  // Odometry publishers
  private final DoublePublisher m_xPub;
  private final DoublePublisher m_yPub;
  private final DoublePublisher m_headingPub;
  private final DoublePublisher m_headingRatePub;
  private final DoubleArrayPublisher m_posePub;
  private final StringPublisher m_odomStatusPub;

  // Motor discovery status
  private final IntegerPublisher m_motorCountPub;
  private final StringPublisher m_motorListPub;

  // Vision publishers
  private final IntegerPublisher m_tagCountPub;
  private final DoublePublisher m_bestTagAreaPub;
  private final DoublePublisher m_bestTagYawPub;
  private final DoublePublisher m_bestTagPitchPub;
  private final DoublePublisher m_bestTagAmbiguityPub;
  private final StringPublisher m_detectedTagsPub;
  private final StringPublisher m_visionStatusPub;

  // System health publishers
  private final DoublePublisher m_batteryVoltagePub;
  private final DoublePublisher m_loopTimePub;
  private final DoublePublisher m_canUtilPub;
  private final IntegerPublisher m_brownoutCountPub;

  // Power management publishers (from PowerManagementSubsystem)
  private final StringPublisher m_powerStatePub;
  private final DoublePublisher m_dischargeRatePub;
  private final DoublePublisher m_batteryLifePub;
  private final DoublePublisher m_batteryHealthPub;
  private final DoublePublisher m_minVoltagePub;
  private final DoublePublisher m_voltageDropPub;
  private final StringPublisher m_safeToShootPub;

  // Match info publishers
  private final StringPublisher m_matchPhasePub;
  private final DoublePublisher m_matchTimePub;
  private final StringPublisher m_alliancePub;
  private final IntegerPublisher m_matchNumberPub;

  // Event log publishers
  private final StringArrayPublisher m_eventLogPub;
  private final IntegerPublisher m_shotCountPub;

  // ===== REPLAY DATA PUBLISHERS =====
  // Frame timing (critical for replay synchronization)
  private final IntegerPublisher m_frameNumberPub;
  private final DoublePublisher m_frameTimestampPub;
  private final DoublePublisher m_matchTimestampPub;

  // Controller input publishers (for match replay)
  private final DoublePublisher m_leftXPub;
  private final DoublePublisher m_leftYPub;
  private final DoublePublisher m_rightXPub;
  private final DoublePublisher m_rightYPub;
  private final DoublePublisher m_l2Pub;
  private final DoublePublisher m_r2Pub;
  private final IntegerPublisher m_buttonStatePub;  // Bitmask of all buttons

  // Command state publisher
  private final StringPublisher m_activeCommandsPub;

  // ===== TIMING =====
  private double m_lastSlowUpdate = 0.0;
  private double m_lastLoopTime = 0.0;
  private int m_brownoutCount = 0;

  // ===== EVENT LOG =====
  private final List<String> m_eventLog = new ArrayList<>();
  private static final int MAX_EVENT_LOG_SIZE = 100;

  // ===== SESSION DATA =====
  private int m_totalShotCount = 0;
  private double m_sessionStartTime = 0.0;
  private long m_frameNumber = 0;
  private String m_activeCommands = "";

  /**
   * Publishers for a single motor's telemetry data.
   */
  private static class MotorPublishers {
    final DoublePublisher rpmPub;
    final DoublePublisher ampsPub;
    final DoublePublisher tempPub;
    final DoublePublisher voltagePub;
    final StringPublisher statusPub;

    MotorPublishers(NetworkTable table, int canId, String busName) {
      String prefix = "motor_" + canId;
      if (!busName.isEmpty()) {
        prefix += "_" + busName.replace(" ", "_");
      }

      rpmPub = table.getDoubleTopic(prefix + "/rpm").publish();
      ampsPub = table.getDoubleTopic(prefix + "/amps").publish();
      tempPub = table.getDoubleTopic(prefix + "/temp_c").publish();
      voltagePub = table.getDoubleTopic(prefix + "/voltage").publish();
      statusPub = table.getStringTopic(prefix + "/status").publish();
    }
  }

  public TelemetrySubsystem() {
    m_ntInstance = NetworkTableInstance.getDefault();
    m_motorTable = m_ntInstance.getTable(TelemetryConstants.kMotorsTableName);

    // Initialize Odometry publishers
    NetworkTable odomTable = m_ntInstance.getTable(TelemetryConstants.kOdometryTableName);
    m_xPub = odomTable.getDoubleTopic("x_meters").publish();
    m_yPub = odomTable.getDoubleTopic("y_meters").publish();
    m_headingPub = odomTable.getDoubleTopic("heading_deg").publish();
    m_headingRatePub = odomTable.getDoubleTopic("heading_rate_dps").publish();
    m_posePub = odomTable.getDoubleArrayTopic("pose").publish();
    m_odomStatusPub = odomTable.getStringTopic("status").publish();

    // Motor discovery status publishers
    m_motorCountPub = m_motorTable.getIntegerTopic("discovered_count").publish();
    m_motorListPub = m_motorTable.getStringTopic("discovered_ids").publish();

    // Initialize Vision publishers
    NetworkTable visionTable = m_ntInstance.getTable(TelemetryConstants.kVisionTelemetryTableName);
    m_tagCountPub = visionTable.getIntegerTopic("tag_count").publish();
    m_bestTagAreaPub = visionTable.getDoubleTopic("best_tag_area").publish();
    m_bestTagYawPub = visionTable.getDoubleTopic("best_tag_yaw").publish();
    m_bestTagPitchPub = visionTable.getDoubleTopic("best_tag_pitch").publish();
    m_bestTagAmbiguityPub = visionTable.getDoubleTopic("best_tag_ambiguity").publish();
    m_detectedTagsPub = visionTable.getStringTopic("detected_tags").publish();
    m_visionStatusPub = visionTable.getStringTopic("status").publish();

    // Initialize System health publishers
    NetworkTable sysTable = m_ntInstance.getTable(TelemetryConstants.kSystemTableName);
    m_batteryVoltagePub = sysTable.getDoubleTopic("battery_voltage").publish();
    m_loopTimePub = sysTable.getDoubleTopic("loop_time_ms").publish();
    m_canUtilPub = sysTable.getDoubleTopic("can_utilization").publish();
    m_brownoutCountPub = sysTable.getIntegerTopic("brownout_count").publish();

    // Initialize Power management publishers (detailed battery/power data)
    m_powerStatePub = sysTable.getStringTopic("power_state").publish();
    m_dischargeRatePub = sysTable.getDoubleTopic("discharge_rate_vs").publish();
    m_batteryLifePub = sysTable.getDoubleTopic("battery_life_est_s").publish();
    m_batteryHealthPub = sysTable.getDoubleTopic("battery_health_pct").publish();
    m_minVoltagePub = sysTable.getDoubleTopic("min_voltage_session").publish();
    m_voltageDropPub = sysTable.getDoubleTopic("voltage_drop_v").publish();
    m_safeToShootPub = sysTable.getStringTopic("safe_to_shoot").publish();

    // Initialize Match info publishers
    NetworkTable matchTable = m_ntInstance.getTable(TelemetryConstants.kMatchTableName);
    m_matchPhasePub = matchTable.getStringTopic("phase").publish();
    m_matchTimePub = matchTable.getDoubleTopic("time_remaining").publish();
    m_alliancePub = matchTable.getStringTopic("alliance").publish();
    m_matchNumberPub = matchTable.getIntegerTopic("match_number").publish();

    // Initialize Event publishers
    NetworkTable eventTable = m_ntInstance.getTable(TelemetryConstants.kEventsTableName);
    m_eventLogPub = eventTable.getStringArrayTopic("log").publish();
    m_shotCountPub = eventTable.getIntegerTopic("shot_count").publish();

    // Initialize Replay data publishers (for match simulation)
    NetworkTable replayTable = m_ntInstance.getTable(TelemetryConstants.kTelemetryTableName + "/Replay");
    m_frameNumberPub = replayTable.getIntegerTopic("frame_number").publish();
    m_frameTimestampPub = replayTable.getDoubleTopic("frame_timestamp").publish();
    m_matchTimestampPub = replayTable.getDoubleTopic("match_timestamp").publish();
    m_activeCommandsPub = replayTable.getStringTopic("active_commands").publish();

    // Initialize Controller input publishers (for match replay)
    NetworkTable controllerTable = m_ntInstance.getTable(TelemetryConstants.kTelemetryTableName + "/Controller");
    m_leftXPub = controllerTable.getDoubleTopic("left_x").publish();
    m_leftYPub = controllerTable.getDoubleTopic("left_y").publish();
    m_rightXPub = controllerTable.getDoubleTopic("right_x").publish();
    m_rightYPub = controllerTable.getDoubleTopic("right_y").publish();
    m_l2Pub = controllerTable.getDoubleTopic("l2_axis").publish();
    m_r2Pub = controllerTable.getDoubleTopic("r2_axis").publish();
    m_buttonStatePub = controllerTable.getIntegerTopic("button_state").publish();

    m_sessionStartTime = Timer.getFPGATimestamp();
    m_startupTime = m_sessionStartTime;

    System.out.println("[TELEMETRY] TelemetrySubsystem initialized");
    System.out.println("[TELEMETRY] Motor scan deferred until " + SCAN_DELAY_SECONDS + "s after startup");
    logEvent("SYSTEM", "Telemetry started - motor scan deferred");
  }

  /**
   * Set references to other subsystems for data collection.
   */
  public void setSubsystems(
      MotorGroup1Subsystem motorGroup1,
      MotorGroup2Subsystem motorGroup2,
      VisionSubsystem vision,
      CommandPS5Controller controller) {
    // Motor groups no longer needed - we auto-discover!
    m_vision = vision;
    m_controller = controller;
    logEvent("SYSTEM", "Subsystem references set (vision + controller)");
  }

  /**
   * Set power management subsystem for battery/power telemetry.
   */
  public void setPowerManagement(PowerManagementSubsystem power) {
    m_power = power;
    logEvent("SYSTEM", "Power management telemetry enabled");
  }

  /**
   * Motor discovery - deferred until after startup to not impact boot time.
   * Waits SCAN_DELAY_SECONDS after startup, then scans all expected IDs at once.
   */
  private void scanForMotors() {
    if (m_motorScanComplete) return;

    // Skip if discovery disabled
    if (!TelemetryConstants.kEnableMotorDiscovery) {
      m_motorScanComplete = true;
      m_motorCountPub.set(0);
      m_motorListPub.set("");
      return;
    }

    // Wait until after startup delay (zero impact on boot time)
    double timeSinceStartup = Timer.getFPGATimestamp() - m_startupTime;
    if (timeSinceStartup < SCAN_DELAY_SECONDS) {
      return;  // Not yet - keep waiting
    }

    // Now do the full scan (startup is done, safe to take a bit of time)
    if (!m_motorScanStarted) {
      m_motorScanStarted = true;
      System.out.println("[TELEMETRY] Starting motor discovery...");

      int[] expectedIds = TelemetryConstants.kExpectedMotorIDs;
      String[] canBuses = TelemetryConstants.kCANBusNames;

      // Scan all expected IDs on all buses
      for (String busName : canBuses) {
        for (int canId : expectedIds) {
          if (m_discoveredMotors.containsKey(canId)) continue;

          try {
            TalonFX motor;
            if (busName.isEmpty()) {
              motor = new TalonFX(canId);
            } else {
              motor = new TalonFX(canId, busName);
            }

            if (motor.isAlive()) {
              m_discoveredMotors.put(canId, motor);
              m_motorPublishers.put(canId, new MotorPublishers(m_motorTable, canId, busName));

              String busLabel = busName.isEmpty() ? "rio" : busName;
              System.out.println("[TELEMETRY] Found TalonFX: CAN ID " + canId + " on " + busLabel);
            }
          } catch (Exception e) {
            // Motor doesn't exist - continue
          }
        }
      }

      // Build ID list
      StringBuilder allIds = new StringBuilder();
      for (int id : m_discoveredMotors.keySet()) {
        if (allIds.length() > 0) allIds.append(", ");
        allIds.append(id);
      }

      m_motorCountPub.set(m_discoveredMotors.size());
      m_motorListPub.set(allIds.toString());
      m_motorScanComplete = true;

      if (m_discoveredMotors.isEmpty()) {
        System.out.println("[TELEMETRY] No motors found");
      } else {
        System.out.println("[TELEMETRY] Found " + m_discoveredMotors.size() +
            " motors: [" + allIds + "]");
      }
      logEvent("MOTOR", "Discovered " + m_discoveredMotors.size() + " motors");
    }
  }

  /**
   * Initialize Pigeon 2 IMU. Call this lazily to avoid blocking robot startup.
   */
  private void initializePigeon() {
    if (m_pigeonChecked) return;
    m_pigeonChecked = true;

    if (TelemetryConstants.kPigeon2ID < 0) {
      System.out.println("[TELEMETRY] Pigeon 2 disabled in config (ID = -1)");
      m_odomStatusPub.set("DISABLED");
      logEvent("ODOM", "Pigeon 2 disabled by config");
      return;
    }

    try {
      if (TelemetryConstants.kPigeon2CANBus.isEmpty()) {
        m_pigeon = new Pigeon2(TelemetryConstants.kPigeon2ID);
      } else {
        m_pigeon = new Pigeon2(TelemetryConstants.kPigeon2ID, TelemetryConstants.kPigeon2CANBus);
      }

      double testYaw = m_pigeon.getYaw().getValueAsDouble();
      if (Double.isNaN(testYaw)) {
        throw new RuntimeException("Pigeon returned NaN");
      }

      m_pigeonAvailable = true;
      m_odomStatusPub.set("ACTIVE");
      System.out.println("[TELEMETRY] Pigeon 2 detected on CAN ID " + TelemetryConstants.kPigeon2ID);
      logEvent("ODOM", "Pigeon 2 initialized - odometry enabled");
      m_robotHeadingDeg = testYaw;

    } catch (Exception e) {
      m_pigeonAvailable = false;
      m_pigeon = null;
      m_odomStatusPub.set("NOT_FOUND");
      System.out.println("[TELEMETRY] Pigeon 2 not detected - odometry disabled");
      logEvent("ODOM", "Pigeon 2 not found - odometry disabled");
    }
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Lazy initialization (doesn't block startup)
    if (!m_pigeonChecked) {
      initializePigeon();
    }
    if (!m_motorScanComplete) {
      scanForMotors();
    }

    // Calculate loop time
    double loopTime = (currentTime - m_lastLoopTime) * 1000.0;
    m_lastLoopTime = currentTime;
    m_loopTimePub.set(loopTime);

    // ===== HIGH-FREQUENCY UPDATES (every loop) =====
    updateReplayData(currentTime);
    updateControllerInputs();
    updateOdometry();
    updateVisionTelemetry();
    updateDiscoveredMotors();

    // ===== LOW-FREQUENCY UPDATES (10Hz) =====
    if (currentTime - m_lastSlowUpdate >= (1.0 / TelemetryConstants.kSlowTelemetryHz)) {
      m_lastSlowUpdate = currentTime;
      updateSystemHealth();
      updateMatchInfo();
      publishEventLog();
    }
  }

  private void updateOdometry() {
    if (!m_pigeonAvailable || m_pigeon == null) {
      m_posePub.set(new double[]{0, 0, 0});
      return;
    }

    try {
      double yaw = m_pigeon.getYaw().getValueAsDouble();
      double yawRate = m_pigeon.getAngularVelocityZWorld().getValueAsDouble();

      m_robotHeadingDeg = yaw;

      m_xPub.set(m_robotX);
      m_yPub.set(m_robotY);
      m_headingPub.set(m_robotHeadingDeg);
      m_headingRatePub.set(yawRate);
      m_posePub.set(new double[]{m_robotX, m_robotY, m_robotHeadingDeg});

    } catch (Exception e) {
      m_pigeonAvailable = false;
      m_odomStatusPub.set("ERROR");
      logEvent("ODOM", "Pigeon read error: " + e.getMessage());
    }
  }

  private void updateVisionTelemetry() {
    if (m_vision == null || !TelemetryConstants.kLogVisionData) return;

    m_tagCountPub.set(m_vision.getTotalTagCount());
    m_bestTagAreaPub.set(m_vision.getBestArea());
    m_bestTagYawPub.set(m_vision.getBestYaw());
    m_bestTagPitchPub.set(m_vision.getBestPitch());
    m_bestTagAmbiguityPub.set(m_vision.getBestAmbiguity());
    m_detectedTagsPub.set(m_vision.getDetectedTagIds().toString());

    String status = m_vision.hasHubTags() ? "HUB_LOCKED" :
        (m_vision.hasAnyTags() ? "TAGS_VISIBLE" : "NO_TAGS");
    m_visionStatusPub.set(status);
  }

  /**
   * Update telemetry for all auto-discovered motors.
   */
  private void updateDiscoveredMotors() {
    if (!TelemetryConstants.kLogMotorData) return;

    for (Map.Entry<Integer, TalonFX> entry : m_discoveredMotors.entrySet()) {
      int canId = entry.getKey();
      TalonFX motor = entry.getValue();
      MotorPublishers pubs = m_motorPublishers.get(canId);

      if (pubs == null) continue;

      try {
        // Read motor data
        double velocityRPS = motor.getVelocity().getValueAsDouble();
        double rpm = velocityRPS * 60.0;
        double amps = motor.getSupplyCurrent().getValueAsDouble();
        double tempC = motor.getDeviceTemp().getValueAsDouble();
        double voltage = motor.getMotorVoltage().getValueAsDouble();

        // Publish to NetworkTables
        pubs.rpmPub.set(rpm);
        pubs.ampsPub.set(amps);
        pubs.tempPub.set(tempC);
        pubs.voltagePub.set(voltage);
        pubs.statusPub.set(motor.isAlive() ? "OK" : "FAULT");

      } catch (Exception e) {
        pubs.statusPub.set("ERROR");
      }
    }
  }

  /**
   * Update replay synchronization data (frame number, timestamps).
   * Critical for match simulation - provides timing reference for all data.
   */
  private void updateReplayData(double currentTime) {
    m_frameNumber++;
    m_frameNumberPub.set(m_frameNumber);
    m_frameTimestampPub.set(currentTime);
    m_matchTimestampPub.set(currentTime - m_sessionStartTime);
    m_activeCommandsPub.set(m_activeCommands);
  }

  /**
   * Update controller input state for match replay.
   * Logs all axes and buttons to allow full simulation replay.
   */
  private void updateControllerInputs() {
    if (m_controller == null || !TelemetryConstants.kLogControllerInputs) return;

    try {
      // Get underlying HID for raw axis/button access
      var hid = m_controller.getHID();

      // Joystick axes
      m_leftXPub.set(hid.getLeftX());
      m_leftYPub.set(hid.getLeftY());
      m_rightXPub.set(hid.getRightX());
      m_rightYPub.set(hid.getRightY());
      m_l2Pub.set(hid.getL2Axis());
      m_r2Pub.set(hid.getR2Axis());

      // Button state as bitmask (efficient single value for all buttons)
      // Bit layout: 0=Square, 1=Cross, 2=Circle, 3=Triangle, 4=L1, 5=R1,
      //             6=L2, 7=R2, 8=Create, 9=Options, 10=L3, 11=R3, 12=PS, 13=Touchpad
      int buttonState = 0;
      if (hid.getSquareButton()) buttonState |= (1 << 0);
      if (hid.getCrossButton()) buttonState |= (1 << 1);
      if (hid.getCircleButton()) buttonState |= (1 << 2);
      if (hid.getTriangleButton()) buttonState |= (1 << 3);
      if (hid.getL1Button()) buttonState |= (1 << 4);
      if (hid.getR1Button()) buttonState |= (1 << 5);
      if (hid.getL2Button()) buttonState |= (1 << 6);
      if (hid.getR2Button()) buttonState |= (1 << 7);
      if (hid.getCreateButton()) buttonState |= (1 << 8);
      if (hid.getOptionsButton()) buttonState |= (1 << 9);
      if (hid.getL3Button()) buttonState |= (1 << 10);
      if (hid.getR3Button()) buttonState |= (1 << 11);
      if (hid.getPSButton()) buttonState |= (1 << 12);
      if (hid.getTouchpadButton()) buttonState |= (1 << 13);

      m_buttonStatePub.set(buttonState);

    } catch (Exception e) {
      // Controller may not be connected - just skip
    }
  }

  /**
   * Set current active commands (for replay tracking).
   * Call this from commands when they start/end.
   */
  public void setActiveCommands(String commands) {
    m_activeCommands = commands;
  }

  private void updateSystemHealth() {
    if (!TelemetryConstants.kLogSystemHealth) return;

    double voltage = RobotController.getBatteryVoltage();
    m_batteryVoltagePub.set(voltage);

    if (RobotController.isBrownedOut()) {
      m_brownoutCount++;
      logEvent("SYSTEM", "Brownout detected! Count: " + m_brownoutCount);
    }
    m_brownoutCountPub.set(m_brownoutCount);

    double canUtil = RobotController.getCANStatus().percentBusUtilization * 100.0;
    m_canUtilPub.set(canUtil);

    // Publish power management data if available
    if (m_power != null) {
      m_powerStatePub.set(m_power.getPowerState().toString());
      m_dischargeRatePub.set(m_power.getSmoothedSlope());
      m_batteryLifePub.set(m_power.getEstimatedBatteryLife());
      m_batteryHealthPub.set(m_power.getBatteryHealthPercent());
      m_minVoltagePub.set(m_power.getVoltage());  // Current voltage
      m_voltageDropPub.set(m_power.getVoltageDropSinceStart());
      m_safeToShootPub.set(m_power.isSafeToShoot() ? "YES" : "NO");
    }
  }

  private void updateMatchInfo() {
    String phase;
    if (DriverStation.isDisabled()) {
      phase = "DISABLED";
    } else if (DriverStation.isAutonomous()) {
      phase = "AUTO";
    } else if (DriverStation.isTeleop()) {
      phase = "TELEOP";
    } else if (DriverStation.isTest()) {
      phase = "TEST";
    } else {
      phase = "UNKNOWN";
    }
    m_matchPhasePub.set(phase);
    m_matchTimePub.set(DriverStation.getMatchTime());

    var alliance = DriverStation.getAlliance();
    String allianceStr = alliance.isPresent() ?
        (alliance.get() == DriverStation.Alliance.Red ? "RED" : "BLUE") : "UNKNOWN";
    m_alliancePub.set(allianceStr);
    m_matchNumberPub.set(DriverStation.getMatchNumber());
  }

  private void publishEventLog() {
    if (m_eventLog.isEmpty()) return;
    String[] logArray = m_eventLog.toArray(new String[0]);
    m_eventLogPub.set(logArray);
  }

  /**
   * Log an event with timestamp and category.
   */
  public void logEvent(String category, String message) {
    double timestamp = Timer.getFPGATimestamp() - m_sessionStartTime;
    String entry = String.format("[%.2f][%s] %s", timestamp, category, message);

    m_eventLog.add(entry);
    System.out.println("[TELEMETRY] " + entry);

    while (m_eventLog.size() > MAX_EVENT_LOG_SIZE) {
      m_eventLog.remove(0);
    }
  }

  /**
   * Record a shot event.
   */
  public void recordShot(double rpm, double tagArea, double yaw) {
    m_totalShotCount++;
    m_shotCountPub.set(m_totalShotCount);
    logEvent("SHOT", String.format("Shot #%d @ %.0f RPM, area=%.2f%%, yaw=%.1f°",
        m_totalShotCount, rpm, tagArea, yaw));
  }

  /**
   * Set robot position (for initial pose or vision correction).
   */
  public void setRobotPose(double x, double y, double headingDeg) {
    m_robotX = x;
    m_robotY = y;
    m_robotHeadingDeg = headingDeg;

    if (m_pigeonAvailable && m_pigeon != null) {
      m_pigeon.setYaw(headingDeg);
    }

    logEvent("ODOM", String.format("Pose set: (%.2f, %.2f) @ %.1f°", x, y, headingDeg));
  }

  /**
   * Get current robot pose.
   */
  public Pose2d getRobotPose() {
    return new Pose2d(m_robotX, m_robotY, Rotation2d.fromDegrees(m_robotHeadingDeg));
  }

  /**
   * Check if odometry (Pigeon 2) is available.
   */
  public boolean isOdometryAvailable() {
    return m_pigeonAvailable;
  }

  /**
   * Get number of discovered motors.
   */
  public int getDiscoveredMotorCount() {
    return m_discoveredMotors.size();
  }

  /**
   * Get list of discovered motor CAN IDs.
   */
  public List<Integer> getDiscoveredMotorIds() {
    return new ArrayList<>(m_discoveredMotors.keySet());
  }

  /**
   * Get session shot count.
   */
  public int getTotalShotCount() {
    return m_totalShotCount;
  }

  /**
   * Get session duration in seconds.
   */
  public double getSessionDuration() {
    return Timer.getFPGATimestamp() - m_sessionStartTime;
  }
}
