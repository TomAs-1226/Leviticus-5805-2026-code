// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * Shot Success Logging system for match analysis.
 *
 * Records every shot attempt with:
 * - Distance to HUB
 * - Shooter RPM
 * - Alignment (yaw)
 * - Robot velocity (if moving)
 * - Timestamp
 *
 * Creates data for heat map analysis and strategy optimization.
 *
 * @author Baichen Yu
 */
public class ShotLogger {

  /**
   * Represents a single shot attempt.
   */
  public static class ShotRecord {
    public final double timestamp;
    public final double distanceMeters;
    public final double rpm;
    public final double yawDegrees;
    public final double robotVelocity;  // m/s
    public final boolean wasMoving;

    public ShotRecord(double timestamp, double distance, double rpm,
                      double yaw, double velocity) {
      this.timestamp = timestamp;
      this.distanceMeters = distance;
      this.rpm = rpm;
      this.yawDegrees = yaw;
      this.robotVelocity = velocity;
      this.wasMoving = Math.abs(velocity) > 0.1;
    }

    @Override
    public String toString() {
      return String.format("t=%.1f d=%.2fm rpm=%.0f yaw=%.1f° v=%.2fm/s %s",
          timestamp, distanceMeters, rpm, yawDegrees, robotVelocity,
          wasMoving ? "[MOVING]" : "[STATIC]");
    }
  }

  // Shot history
  private final List<ShotRecord> m_shotHistory = new ArrayList<>();
  private static final int MAX_HISTORY = 100;

  // Statistics
  private int m_totalShots = 0;
  private int m_staticShots = 0;
  private int m_movingShots = 0;
  private double m_avgDistance = 0.0;
  private double m_avgRPM = 0.0;
  private double m_minDistance = Double.MAX_VALUE;
  private double m_maxDistance = 0.0;

  // Distance zone tracking (for heat map)
  private int[] m_zoneHits = new int[6];  // 0-1m, 1-2m, 2-3m, 3-4m, 4-5m, 5+m

  // NetworkTables publishing
  private final NetworkTable m_table;
  private final StringArrayPublisher m_historyPub;
  private final DoubleArrayPublisher m_zonesPub;

  private double m_sessionStart;

  public ShotLogger() {
    m_sessionStart = Timer.getFPGATimestamp();

    // Setup NetworkTables
    m_table = NetworkTableInstance.getDefault().getTable("Telemetry/Shots");
    m_historyPub = m_table.getStringArrayTopic("history").publish();
    m_zonesPub = m_table.getDoubleArrayTopic("zone_hits").publish();

    System.out.println("[SHOT LOGGER] Ready - tracking shots for strategy analysis");
  }

  /**
   * Record a shot attempt.
   *
   * @param distance Distance to target in meters
   * @param rpm Shooter RPM at time of shot
   * @param yaw Alignment angle in degrees
   * @param robotVelocity Robot velocity in m/s (0 if stationary)
   */
  public void recordShot(double distance, double rpm, double yaw, double robotVelocity) {
    double timestamp = Timer.getFPGATimestamp() - m_sessionStart;

    ShotRecord shot = new ShotRecord(timestamp, distance, rpm, yaw, robotVelocity);
    m_shotHistory.add(shot);

    // Limit history size
    while (m_shotHistory.size() > MAX_HISTORY) {
      m_shotHistory.remove(0);
    }

    // Update statistics
    m_totalShots++;
    if (shot.wasMoving) {
      m_movingShots++;
    } else {
      m_staticShots++;
    }

    // Update distance stats
    if (distance < m_minDistance) m_minDistance = distance;
    if (distance > m_maxDistance) m_maxDistance = distance;

    // Running average
    m_avgDistance = ((m_avgDistance * (m_totalShots - 1)) + distance) / m_totalShots;
    m_avgRPM = ((m_avgRPM * (m_totalShots - 1)) + rpm) / m_totalShots;

    // Update zone tracking
    int zone = (int) Math.min(5, Math.floor(distance));
    m_zoneHits[zone]++;

    // Publish to NetworkTables
    publishData();

    // Console log
    System.out.println("[SHOT #" + m_totalShots + "] " + shot);
  }

  private void publishData() {
    // Publish recent history
    String[] history = new String[Math.min(10, m_shotHistory.size())];
    for (int i = 0; i < history.length; i++) {
      history[i] = m_shotHistory.get(m_shotHistory.size() - 1 - i).toString();
    }
    m_historyPub.set(history);

    // Publish zone hits (for heat map visualization)
    double[] zones = new double[6];
    for (int i = 0; i < 6; i++) {
      zones[i] = m_zoneHits[i];
    }
    m_zonesPub.set(zones);

    // Publish stats to NetworkTables
    m_table.getEntry("total_shots").setInteger(m_totalShots);
    m_table.getEntry("static_shots").setInteger(m_staticShots);
    m_table.getEntry("moving_shots").setInteger(m_movingShots);
    m_table.getEntry("avg_distance").setDouble(m_avgDistance);
    m_table.getEntry("avg_rpm").setDouble(m_avgRPM);
    m_table.getEntry("min_distance").setDouble(m_minDistance == Double.MAX_VALUE ? 0 : m_minDistance);
    m_table.getEntry("max_distance").setDouble(m_maxDistance);
  }

  /**
   * Get session summary for logging.
   */
  public String getSessionSummary() {
    if (m_totalShots == 0) {
      return "No shots recorded this session";
    }

    StringBuilder sb = new StringBuilder();
    sb.append("=== SHOT SESSION SUMMARY ===\n");
    sb.append(String.format("Total shots: %d (static: %d, moving: %d)\n",
        m_totalShots, m_staticShots, m_movingShots));
    sb.append(String.format("Distance range: %.2f - %.2fm (avg: %.2fm)\n",
        m_minDistance, m_maxDistance, m_avgDistance));
    sb.append(String.format("Average RPM: %.0f\n", m_avgRPM));
    sb.append("Zone distribution:\n");
    for (int i = 0; i < 6; i++) {
      sb.append(String.format("  %d-%dm: %d shots (%.0f%%)\n",
          i, i + 1, m_zoneHits[i],
          m_totalShots > 0 ? (m_zoneHits[i] * 100.0 / m_totalShots) : 0));
    }
    return sb.toString();
  }

  /**
   * Get the most successful distance zone.
   * Returns the zone with most shots (where team is most comfortable shooting).
   */
  public int getMostUsedZone() {
    int maxZone = 0;
    int maxHits = 0;
    for (int i = 0; i < 6; i++) {
      if (m_zoneHits[i] > maxHits) {
        maxHits = m_zoneHits[i];
        maxZone = i;
      }
    }
    return maxZone;
  }

  /**
   * Get shot history for analysis.
   */
  public List<ShotRecord> getHistory() {
    return new ArrayList<>(m_shotHistory);
  }

  public int getTotalShots() { return m_totalShots; }
  public int getStaticShots() { return m_staticShots; }
  public int getMovingShots() { return m_movingShots; }
  public double getAverageDistance() { return m_avgDistance; }
  public double getAverageRPM() { return m_avgRPM; }
}
