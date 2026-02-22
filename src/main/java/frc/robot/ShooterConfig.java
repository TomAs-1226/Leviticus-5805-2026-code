// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Shooter configuration manager - loads distance-to-RPM lookup table from JSON.
 *
 * DESIGN:
 * - Loads once at robot startup (zero file I/O during matches)
 * - All data cached in memory for instant lookups
 * - Uses linear interpolation between data points
 * - Handles out-of-range distances gracefully
 *
 * JSON FORMAT:
 * {
 *   "version": "1.0",
 *   "table": [
 *     {"distance_meters": 1.0, "rpm": 1500, "notes": "..."},
 *     {"distance_meters": 2.0, "rpm": 2500, "notes": "..."}
 *   ]
 * }
 *
 * @author Baichen Yu
 */
public class ShooterConfig {

  /** Single entry in the shooter lookup table. */
  public static class ShooterEntry implements Comparable<ShooterEntry> {
    public final double distanceMeters;
    public final double rpm;
    public final double hoodAngleDegrees;  // Hood angle for this distance
    public final String notes;

    public ShooterEntry(double distance, double rpm, double hoodAngle, String notes) {
      this.distanceMeters = distance;
      this.rpm = rpm;
      this.hoodAngleDegrees = hoodAngle;
      this.notes = notes != null ? notes : "";
    }

    // Legacy constructor without hood angle
    public ShooterEntry(double distance, double rpm, String notes) {
      this(distance, rpm, 25.0, notes);  // Default to 25 degrees
    }

    @Override
    public int compareTo(ShooterEntry other) {
      return Double.compare(this.distanceMeters, other.distanceMeters);
    }

    @Override
    public String toString() {
      return String.format("%.1fm -> %.0f RPM @ %.1f° (%s)", distanceMeters, rpm, hoodAngleDegrees, notes);
    }
  }

  // Cached data (loaded once at startup, never changes)
  private final List<ShooterEntry> m_table = new ArrayList<>();
  private String m_version = "unknown";
  private boolean m_loaded = false;

  // Default fallback values if file can't be loaded
  private static final double DEFAULT_RPM = 3000.0;

  /**
   * Load shooter configuration from deploy/shooter_config.json.
   * Call this once at robot init - all data is cached in memory.
   */
  public boolean loadConfig() {
    if (m_loaded) {
      System.out.println("[SHOOTER CONFIG] Already loaded, skipping");
      return true;
    }

    try {
      // Get path to deployed JSON file
      Path deployDir = Filesystem.getDeployDirectory().toPath();
      Path configFile = deployDir.resolve("shooter_config.json");

      if (!Files.exists(configFile)) {
        System.err.println("[SHOOTER CONFIG] ERROR: shooter_config.json not found at " + configFile);
        System.err.println("[SHOOTER CONFIG] Using default fallback RPM: " + DEFAULT_RPM);
        loadDefaultConfig();
        return false;
      }

      // Read entire file into memory (happens once at startup, no runtime overhead)
      File configFileObj = configFile.toFile();

      // Parse JSON using Jackson (included in WPILib)
      ObjectMapper mapper = new ObjectMapper();
      JsonNode root = mapper.readTree(configFileObj);

      // Read version
      if (root.has("version")) {
        m_version = root.get("version").asText();
      }

      // Read table entries
      JsonNode table = root.get("table");
      if (table != null && table.isArray()) {
        for (JsonNode entry : table) {
          double distance = entry.get("distance_meters").asDouble();
          double rpm = entry.get("rpm").asDouble();
          // Hood angle is optional - default to 25 degrees if not present
          double hoodAngle = entry.has("hood_angle_degrees") ?
              entry.get("hood_angle_degrees").asDouble() : 25.0;
          String notes = entry.has("notes") ? entry.get("notes").asText() : "";

          m_table.add(new ShooterEntry(distance, rpm, hoodAngle, notes));
        }
      }

      // Sort table by distance for binary search and interpolation
      Collections.sort(m_table);

      m_loaded = true;

      // Log success
      System.out.println("===========================================");
      System.out.println("[SHOOTER CONFIG] Loaded successfully");
      System.out.println("[SHOOTER CONFIG] Version: " + m_version);
      System.out.println("[SHOOTER CONFIG] Entries: " + m_table.size());
      System.out.println("[SHOOTER CONFIG] Range: " + getMinDistance() + "m - " + getMaxDistance() + "m");
      System.out.println("[SHOOTER CONFIG] Lookup table:");
      for (ShooterEntry e : m_table) {
        System.out.println("[SHOOTER CONFIG]   " + e);
      }
      System.out.println("===========================================");

      return true;

    } catch (IOException e) {
      System.err.println("[SHOOTER CONFIG] ERROR: Failed to read shooter_config.json");
      e.printStackTrace();
      loadDefaultConfig();
      return false;
    } catch (Exception e) {
      System.err.println("[SHOOTER CONFIG] ERROR: Failed to parse shooter_config.json");
      e.printStackTrace();
      loadDefaultConfig();
      return false;
    }
  }

  /**
   * Load hardcoded default config if file can't be loaded.
   * This ensures the robot can still shoot even if the JSON is missing.
   */
  private void loadDefaultConfig() {
    m_table.clear();
    // Default entries with distance, RPM, and hood angle
    m_table.add(new ShooterEntry(1.0, 1500, 18.0, "Default - close"));
    m_table.add(new ShooterEntry(2.0, 2500, 25.0, "Default - mid"));
    m_table.add(new ShooterEntry(3.0, 3500, 33.0, "Default - far"));
    m_table.add(new ShooterEntry(4.0, 4500, 42.0, "Default - max"));
    Collections.sort(m_table);
    m_loaded = true;
    m_version = "default-fallback";

    System.out.println("[SHOOTER CONFIG] Loaded fallback defaults (" + m_table.size() + " entries)");
  }

  /**
   * Get shooter RPM for a given distance.
   * Uses linear interpolation between data points.
   *
   * @param distanceMeters Distance to target in meters
   * @return Recommended shooter RPM
   */
  public double getRPMForDistance(double distanceMeters) {
    if (!m_loaded || m_table.isEmpty()) {
      System.err.println("[SHOOTER CONFIG] Config not loaded! Using default RPM: " + DEFAULT_RPM);
      return DEFAULT_RPM;
    }

    // Clamp to valid range
    if (distanceMeters <= m_table.get(0).distanceMeters) {
      // Closer than closest entry - use minimum RPM
      return m_table.get(0).rpm;
    }

    if (distanceMeters >= m_table.get(m_table.size() - 1).distanceMeters) {
      // Farther than farthest entry - use maximum RPM
      return m_table.get(m_table.size() - 1).rpm;
    }

    // Find the two entries to interpolate between
    for (int i = 0; i < m_table.size() - 1; i++) {
      ShooterEntry lower = m_table.get(i);
      ShooterEntry upper = m_table.get(i + 1);

      if (distanceMeters >= lower.distanceMeters && distanceMeters <= upper.distanceMeters) {
        // Linear interpolation: rpm = rpm1 + (rpm2 - rpm1) * (d - d1) / (d2 - d1)
        double fraction = (distanceMeters - lower.distanceMeters) / (upper.distanceMeters - lower.distanceMeters);
        double interpolatedRPM = lower.rpm + (upper.rpm - lower.rpm) * fraction;
        return interpolatedRPM;
      }
    }

    // Should never reach here, but fallback to default just in case
    System.err.println("[SHOOTER CONFIG] Interpolation failed for distance: " + distanceMeters);
    return DEFAULT_RPM;
  }

  // Default hood angle if no config loaded
  private static final double DEFAULT_HOOD_ANGLE = 25.0;

  /**
   * Get hood angle for a given distance.
   * Uses linear interpolation between data points.
   *
   * @param distanceMeters Distance to target in meters
   * @return Recommended hood angle in degrees
   */
  public double getHoodAngleForDistance(double distanceMeters) {
    if (!m_loaded || m_table.isEmpty()) {
      return DEFAULT_HOOD_ANGLE;
    }

    // Clamp to valid range
    if (distanceMeters <= m_table.get(0).distanceMeters) {
      return m_table.get(0).hoodAngleDegrees;
    }

    if (distanceMeters >= m_table.get(m_table.size() - 1).distanceMeters) {
      return m_table.get(m_table.size() - 1).hoodAngleDegrees;
    }

    // Find the two entries to interpolate between
    for (int i = 0; i < m_table.size() - 1; i++) {
      ShooterEntry lower = m_table.get(i);
      ShooterEntry upper = m_table.get(i + 1);

      if (distanceMeters >= lower.distanceMeters && distanceMeters <= upper.distanceMeters) {
        double fraction = (distanceMeters - lower.distanceMeters) /
                          (upper.distanceMeters - lower.distanceMeters);
        return lower.hoodAngleDegrees + (upper.hoodAngleDegrees - lower.hoodAngleDegrees) * fraction;
      }
    }

    return DEFAULT_HOOD_ANGLE;
  }

  /**
   * Get both RPM and hood angle for a distance.
   * Returns array: [rpm, hoodAngleDegrees]
   */
  public double[] getSettingsForDistance(double distanceMeters) {
    return new double[] {
      getRPMForDistance(distanceMeters),
      getHoodAngleForDistance(distanceMeters)
    };
  }

  /**
   * Get the exact RPM entry for a distance (no interpolation).
   * Returns null if not an exact match.
   */
  public ShooterEntry getExactEntry(double distanceMeters) {
    for (ShooterEntry entry : m_table) {
      if (Math.abs(entry.distanceMeters - distanceMeters) < 0.001) {
        return entry;
      }
    }
    return null;
  }

  /**
   * Get all entries (for display/logging).
   */
  public List<ShooterEntry> getAllEntries() {
    return new ArrayList<>(m_table);
  }

  /** Get minimum distance in table. */
  public double getMinDistance() {
    return m_table.isEmpty() ? 0.0 : m_table.get(0).distanceMeters;
  }

  /** Get maximum distance in table. */
  public double getMaxDistance() {
    return m_table.isEmpty() ? 0.0 : m_table.get(m_table.size() - 1).distanceMeters;
  }

  /** Check if config was loaded successfully. */
  public boolean isLoaded() {
    return m_loaded;
  }

  /** Get config version. */
  public String getVersion() {
    return m_version;
  }

  /** Get number of entries in table. */
  public int getEntryCount() {
    return m_table.size();
  }
}
