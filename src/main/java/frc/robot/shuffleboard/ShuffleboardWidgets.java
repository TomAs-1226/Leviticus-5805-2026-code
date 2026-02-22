package frc.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

import java.util.Map;

/**
 * Custom Shuffleboard widget factory for FRC 5805 Alphabot.
 * Provides reusable widget components with consistent styling.
 */
public class ShuffleboardWidgets {

    // ==================== GAUGE WIDGETS ====================

    /**
     * Creates an RPM gauge widget with standard FRC motor ranges.
     * @param tab The Shuffleboard tab to add to
     * @param title Widget title
     * @param col Column position
     * @param row Row position
     * @param width Widget width
     * @param height Widget height
     * @param maxRPM Maximum RPM for gauge scale
     * @return GenericEntry for updating the value
     */
    public static GenericEntry createRPMGauge(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, double maxRPM) {
        return tab.add(title, 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", maxRPM,
                "showValue", true
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a voltage gauge widget (0-14V range for battery).
     */
    public static GenericEntry createVoltageGauge(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, 12.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", 14.0,
                "showValue", true
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a current/amps gauge widget.
     */
    public static GenericEntry createAmpsGauge(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, double maxAmps) {
        return tab.add(title, 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", maxAmps,
                "showValue", true
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a temperature gauge widget (0-100C range).
     */
    public static GenericEntry createTempGauge(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, 25.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", 100.0,
                "showValue", true
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a percentage gauge widget (0-100%).
     */
    public static GenericEntry createPercentGauge(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", 100.0,
                "showValue", true
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    // ==================== BAR/METER WIDGETS ====================

    /**
     * Creates a horizontal number bar for value ranges.
     */
    public static GenericEntry createNumberBar(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, double min, double max) {
        return tab.add(title, min)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of(
                "min", min,
                "max", max,
                "center", (min + max) / 2
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a battery voltage bar with color coding thresholds.
     */
    public static GenericEntry createBatteryBar(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, 12.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of(
                "min", 9.0,
                "max", 13.5,
                "center", 11.5
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    // ==================== BOOLEAN INDICATORS ====================

    /**
     * Creates a boolean indicator light.
     */
    public static GenericEntry createBooleanIndicator(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, String colorWhenTrue, String colorWhenFalse) {
        return tab.add(title, false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of(
                "colorWhenTrue", colorWhenTrue,
                "colorWhenFalse", colorWhenFalse
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a green/red status indicator.
     */
    public static GenericEntry createStatusIndicator(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return createBooleanIndicator(tab, title, col, row, width, height, "#00FF00", "#FF0000");
    }

    /**
     * Creates a green/gray active indicator (for running states).
     */
    public static GenericEntry createActiveIndicator(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return createBooleanIndicator(tab, title, col, row, width, height, "#00FF00", "#404040");
    }

    /**
     * Creates a warning indicator (yellow when true).
     */
    public static GenericEntry createWarningIndicator(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return createBooleanIndicator(tab, title, col, row, width, height, "#FFFF00", "#404040");
    }

    /**
     * Creates an alert indicator (red when true).
     */
    public static GenericEntry createAlertIndicator(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return createBooleanIndicator(tab, title, col, row, width, height, "#FF0000", "#404040");
    }

    // ==================== TEXT/VALUE DISPLAYS ====================

    /**
     * Creates a simple text display.
     */
    public static GenericEntry createTextDisplay(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, "---")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    /**
     * Creates a numeric value display.
     */
    public static GenericEntry createValueDisplay(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.add(title, 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    // ==================== GRAPH WIDGETS ====================

    /**
     * Creates a real-time graph widget.
     */
    public static GenericEntry createGraph(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, double visibleTime) {
        return tab.add(title, 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withProperties(Map.of(
                "Visible time", visibleTime
            ))
            .withPosition(col, row)
            .withSize(width, height)
            .getEntry();
    }

    // ==================== LAYOUT HELPERS ====================

    /**
     * Creates a vertical list layout for grouping related widgets.
     */
    public static ShuffleboardLayout createVerticalGroup(ShuffleboardTab tab, String title,
            int col, int row, int width, int height) {
        return tab.getLayout(title, BuiltInLayouts.kList)
            .withPosition(col, row)
            .withSize(width, height)
            .withProperties(Map.of(
                "Label position", "TOP"
            ));
    }

    /**
     * Creates a grid layout for motor groups.
     */
    public static ShuffleboardLayout createGridGroup(ShuffleboardTab tab, String title,
            int col, int row, int width, int height, int columns) {
        return tab.getLayout(title, BuiltInLayouts.kGrid)
            .withPosition(col, row)
            .withSize(width, height)
            .withProperties(Map.of(
                "Number of columns", columns,
                "Number of rows", (int) Math.ceil(4.0 / columns),
                "Label position", "TOP"
            ));
    }

    // ==================== LAYOUT WIDGET HELPERS ====================

    /**
     * Adds an RPM gauge to a layout.
     */
    public static GenericEntry addRPMToLayout(ShuffleboardLayout layout, String title, double maxRPM) {
        return layout.add(title, 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", maxRPM,
                "showValue", true
            ))
            .getEntry();
    }

    /**
     * Adds an amps gauge to a layout.
     */
    public static GenericEntry addAmpsToLayout(ShuffleboardLayout layout, String title, double maxAmps) {
        return layout.add(title, 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", maxAmps,
                "showValue", true
            ))
            .getEntry();
    }

    /**
     * Adds a boolean indicator to a layout.
     */
    public static GenericEntry addStatusToLayout(ShuffleboardLayout layout, String title) {
        return layout.add(title, false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of(
                "colorWhenTrue", "#00FF00",
                "colorWhenFalse", "#FF0000"
            ))
            .getEntry();
    }

    /**
     * Adds a text display to a layout.
     */
    public static GenericEntry addTextToLayout(ShuffleboardLayout layout, String title) {
        return layout.add(title, "---")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }

    /**
     * Adds a temperature gauge to a layout.
     */
    public static GenericEntry addTempToLayout(ShuffleboardLayout layout, String title) {
        return layout.add(title, 25.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of(
                "min", 0.0,
                "max", 100.0,
                "showValue", true
            ))
            .getEntry();
    }
}
