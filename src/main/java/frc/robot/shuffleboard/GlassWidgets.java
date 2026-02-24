package frc.robot.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Glass-compatible widget factory for FRC 5805 Alphabot.
 * Provides Mechanism2d and Field2d visualizations that work in both
 * Shuffleboard and Glass dashboards.
 *
 * Glass Widgets:
 * - Field2d: 2D field visualization with robot position
 * - Mechanism2d: Animated mechanism visualization (hood, intake arm, etc.)
 *
 * All widgets are published to NetworkTables and viewable in Glass/Shuffleboard.
 */
public class GlassWidgets {

    // ==================== COLOR CONSTANTS ====================

    public static final Color8Bit COLOR_GREEN = new Color8Bit(Color.kGreen);
    public static final Color8Bit COLOR_RED = new Color8Bit(Color.kRed);
    public static final Color8Bit COLOR_YELLOW = new Color8Bit(Color.kYellow);
    public static final Color8Bit COLOR_ORANGE = new Color8Bit(Color.kOrange);
    public static final Color8Bit COLOR_BLUE = new Color8Bit(Color.kBlue);
    public static final Color8Bit COLOR_CYAN = new Color8Bit(Color.kCyan);
    public static final Color8Bit COLOR_WHITE = new Color8Bit(Color.kWhite);
    public static final Color8Bit COLOR_GRAY = new Color8Bit(Color.kGray);
    public static final Color8Bit COLOR_DARK_GRAY = new Color8Bit(64, 64, 64);
    public static final Color8Bit COLOR_PURPLE = new Color8Bit(Color.kPurple);

    // ==================== FIELD2D WIDGET ====================

    /**
     * Creates a Field2d widget for robot position visualization.
     * Displays robot on the 2026 REBUILT field.
     *
     * @param name Widget name for NetworkTables
     * @return Field2d object for position updates
     */
    public static Field2d createFieldWidget(String name) {
        Field2d field = new Field2d();
        SmartDashboard.putData(name, field);
        return field;
    }

    /**
     * Updates robot position on field.
     */
    public static void updateFieldPosition(Field2d field, double x, double y, double headingDegrees) {
        field.setRobotPose(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
    }

    /**
     * Adds a target position marker to the field.
     */
    public static void setFieldTarget(Field2d field, String name, double x, double y, double headingDegrees) {
        field.getObject(name).setPose(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
    }

    // ==================== HOOD MECHANISM WIDGET ====================

    /**
     * Hood mechanism visualization showing the adjustable hood angle.
     * Canvas: 100x100 units, hood pivots from bottom-center.
     */
    public static class HoodMechanism {
        private final Mechanism2d mechanism;
        private final MechanismRoot2d root;
        private final MechanismLigament2d hoodBase;
        private final MechanismLigament2d hoodArm;
        private final MechanismLigament2d targetIndicator;

        // Hood angle limits
        private static final double MIN_ANGLE = 15.0;
        private static final double MAX_ANGLE = 75.0;

        public HoodMechanism(String name) {
            // Create 100x80 canvas
            mechanism = new Mechanism2d(100, 80);

            // Base structure (shooter housing)
            root = mechanism.getRoot("hood_pivot", 50, 10);

            // Shooter base (horizontal bar representing shooter body)
            hoodBase = root.append(
                new MechanismLigament2d("shooter_base", 40, 0, 8, COLOR_DARK_GRAY)
            );

            // Hood arm (the adjustable flap)
            hoodArm = root.append(
                new MechanismLigament2d("hood_arm", 30, 45, 6, COLOR_BLUE)
            );

            // Target angle indicator (thin line showing target)
            targetIndicator = root.append(
                new MechanismLigament2d("target", 35, 45, 2, COLOR_GREEN)
            );

            SmartDashboard.putData(name, mechanism);
        }

        /**
         * Updates hood position visualization.
         * @param currentAngle Current hood angle in degrees
         * @param targetAngle Target hood angle in degrees
         * @param isCalibrated Whether hood is calibrated
         * @param atTarget Whether hood is at target position
         */
        public void update(double currentAngle, double targetAngle, boolean isCalibrated, boolean atTarget) {
            // Clamp angles to valid range
            currentAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, currentAngle));
            targetAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, targetAngle));

            // Update arm position
            hoodArm.setAngle(currentAngle);

            // Update target indicator
            targetIndicator.setAngle(targetAngle);

            // Color coding based on state
            if (!isCalibrated) {
                hoodArm.setColor(COLOR_RED);
            } else if (atTarget) {
                hoodArm.setColor(COLOR_GREEN);
            } else {
                hoodArm.setColor(COLOR_BLUE);
            }
        }
    }

    // ==================== INTAKE ARM MECHANISM WIDGET ====================

    /**
     * Intake deploy arm visualization showing arm position.
     * Shows arm rotating between stowed (up) and deployed (down) positions.
     */
    public static class IntakeArmMechanism {
        private final Mechanism2d mechanism;
        private final MechanismRoot2d root;
        private final MechanismLigament2d chassis;
        private final MechanismLigament2d arm;
        private final MechanismLigament2d rollers;

        // Arm angle positions
        private static final double STOWED_ANGLE = 90.0;   // Pointing up
        private static final double DEPLOYED_ANGLE = -45.0; // Pointing down-forward

        public IntakeArmMechanism(String name) {
            // Create 120x100 canvas
            mechanism = new Mechanism2d(120, 100);

            // Chassis/frame (robot body reference)
            root = mechanism.getRoot("arm_pivot", 60, 50);
            chassis = root.append(
                new MechanismLigament2d("chassis", 30, 180, 10, COLOR_DARK_GRAY)
            );

            // Intake arm (rotates from pivot point)
            arm = root.append(
                new MechanismLigament2d("arm", 35, STOWED_ANGLE, 6, COLOR_ORANGE)
            );

            // Rollers at end of arm
            rollers = arm.append(
                new MechanismLigament2d("rollers", 15, 90, 8, COLOR_GRAY)
            );

            SmartDashboard.putData(name, mechanism);
        }

        /**
         * Updates intake arm visualization.
         * @param state Current arm state (STOWED, DEPLOYED, MOVING)
         * @param isRunning Whether intake rollers are running
         * @param isStalled Whether arm motor is stalled
         */
        public void update(String state, boolean isRunning, boolean isStalled) {
            // Set arm angle based on state
            double targetAngle;
            switch (state.toUpperCase()) {
                case "DEPLOYED":
                    targetAngle = DEPLOYED_ANGLE;
                    break;
                case "STOWED":
                    targetAngle = STOWED_ANGLE;
                    break;
                case "MOVING_DOWN":
                    targetAngle = (STOWED_ANGLE + DEPLOYED_ANGLE) / 2;
                    break;
                case "MOVING_UP":
                    targetAngle = (STOWED_ANGLE + DEPLOYED_ANGLE) / 2;
                    break;
                default:
                    targetAngle = STOWED_ANGLE;
            }

            arm.setAngle(targetAngle);

            // Color coding
            if (isStalled) {
                arm.setColor(COLOR_RED);
                rollers.setColor(COLOR_RED);
            } else if (isRunning) {
                arm.setColor(COLOR_GREEN);
                rollers.setColor(COLOR_GREEN);
            } else {
                arm.setColor(COLOR_ORANGE);
                rollers.setColor(COLOR_GRAY);
            }
        }

        /**
         * Updates with specific angle for smooth animation.
         */
        public void updateWithAngle(double armAngleDegrees, boolean isRunning, boolean isStalled) {
            arm.setAngle(armAngleDegrees);

            if (isStalled) {
                arm.setColor(COLOR_RED);
                rollers.setColor(COLOR_RED);
            } else if (isRunning) {
                arm.setColor(COLOR_GREEN);
                rollers.setColor(COLOR_GREEN);
            } else {
                arm.setColor(COLOR_ORANGE);
                rollers.setColor(COLOR_GRAY);
            }
        }
    }

    // ==================== SHOOTER MECHANISM WIDGET ====================

    /**
     * Shooter wheel visualization showing motor status and RPM.
     * Four wheels arranged in a 2x2 grid.
     */
    public static class ShooterMechanism {
        private final Mechanism2d mechanism;
        private final MechanismLigament2d[] wheels;
        private final MechanismLigament2d[] spokes;
        private double[] rotations = new double[4];

        public ShooterMechanism(String name) {
            // Create 100x100 canvas
            mechanism = new Mechanism2d(100, 100);

            wheels = new MechanismLigament2d[4];
            spokes = new MechanismLigament2d[4];

            // Wheel positions in 2x2 grid
            double[][] positions = {
                {25, 70},  // M1 - top left
                {75, 70},  // M2 - top right
                {25, 30},  // M3 - bottom left
                {75, 30}   // M4 - bottom right
            };

            for (int i = 0; i < 4; i++) {
                MechanismRoot2d wheelRoot = mechanism.getRoot("wheel" + (i + 1), positions[i][0], positions[i][1]);

                // Wheel rim
                wheels[i] = wheelRoot.append(
                    new MechanismLigament2d("rim" + (i + 1), 15, 0, 4, COLOR_GRAY)
                );

                // Spoke (to show rotation)
                spokes[i] = wheelRoot.append(
                    new MechanismLigament2d("spoke" + (i + 1), 12, 0, 2, COLOR_WHITE)
                );
            }

            SmartDashboard.putData(name, mechanism);
        }

        /**
         * Updates shooter wheel visualization.
         * @param rpms Array of 4 motor RPMs
         * @param alive Array of 4 motor alive states
         * @param targetRPM Target RPM for color coding
         */
        public void update(double[] rpms, boolean[] alive, double targetRPM) {
            for (int i = 0; i < 4; i++) {
                // Rotate spokes based on RPM (scaled for visibility)
                rotations[i] += rpms[i] * 0.002; // Scale factor for visible rotation
                spokes[i].setAngle(rotations[i] % 360);

                // Color based on status
                if (!alive[i]) {
                    wheels[i].setColor(COLOR_RED);
                    spokes[i].setColor(COLOR_RED);
                } else if (rpms[i] < targetRPM * 0.8) {
                    wheels[i].setColor(COLOR_YELLOW);
                    spokes[i].setColor(COLOR_YELLOW);
                } else if (rpms[i] >= targetRPM * 0.95) {
                    wheels[i].setColor(COLOR_GREEN);
                    spokes[i].setColor(COLOR_GREEN);
                } else {
                    wheels[i].setColor(COLOR_BLUE);
                    spokes[i].setColor(COLOR_BLUE);
                }
            }
        }
    }

    // ==================== BATTERY STATUS MECHANISM ====================

    /**
     * Battery level visualization as a horizontal bar.
     */
    public static class BatteryMechanism {
        private final Mechanism2d mechanism;
        private final MechanismLigament2d batteryOutline;
        private final MechanismLigament2d batteryLevel;
        private final MechanismLigament2d batteryTip;

        public BatteryMechanism(String name) {
            // Create 100x40 canvas
            mechanism = new Mechanism2d(100, 40);

            // Battery outline
            MechanismRoot2d outlineRoot = mechanism.getRoot("outline", 10, 20);
            batteryOutline = outlineRoot.append(
                new MechanismLigament2d("outline", 70, 0, 15, COLOR_DARK_GRAY)
            );

            // Battery tip (positive terminal)
            MechanismRoot2d tipRoot = mechanism.getRoot("tip", 80, 20);
            batteryTip = tipRoot.append(
                new MechanismLigament2d("tip", 5, 0, 8, COLOR_DARK_GRAY)
            );

            // Battery level (fills based on percentage)
            MechanismRoot2d levelRoot = mechanism.getRoot("level", 12, 20);
            batteryLevel = levelRoot.append(
                new MechanismLigament2d("level", 66, 0, 11, COLOR_GREEN)
            );

            SmartDashboard.putData(name, mechanism);
        }

        /**
         * Updates battery visualization.
         * @param voltage Current battery voltage
         * @param healthPercent Battery health percentage (0-100)
         */
        public void update(double voltage, double healthPercent) {
            // Scale battery level length (max 66 units at 100%)
            double levelLength = (healthPercent / 100.0) * 66.0;
            batteryLevel.setLength(Math.max(1, levelLength));

            // Color based on voltage thresholds
            if (voltage < 10.5) {
                batteryLevel.setColor(COLOR_RED);
            } else if (voltage < 11.5) {
                batteryLevel.setColor(COLOR_ORANGE);
            } else if (voltage < 12.0) {
                batteryLevel.setColor(COLOR_YELLOW);
            } else {
                batteryLevel.setColor(COLOR_GREEN);
            }
        }
    }

    // ==================== COMBINED ROBOT STATUS MECHANISM ====================

    /**
     * Combined robot status visualization showing all major systems.
     * Displays a top-down robot view with status indicators.
     */
    public static class RobotStatusMechanism {
        private final Mechanism2d mechanism;
        private final MechanismLigament2d chassisOutline;
        private final MechanismLigament2d intakeIndicator;
        private final MechanismLigament2d shooterIndicator;
        private final MechanismLigament2d feederIndicator;
        private final MechanismLigament2d hoodIndicator;
        private final MechanismLigament2d visionIndicator;

        public RobotStatusMechanism(String name) {
            // Create 100x100 canvas
            mechanism = new Mechanism2d(100, 100);

            // Robot chassis outline (rectangle represented by lines)
            MechanismRoot2d chassisRoot = mechanism.getRoot("chassis", 50, 50);
            chassisOutline = chassisRoot.append(
                new MechanismLigament2d("chassis", 40, 90, 3, COLOR_WHITE)
            );

            // Intake indicator (front of robot)
            MechanismRoot2d intakeRoot = mechanism.getRoot("intake", 50, 85);
            intakeIndicator = intakeRoot.append(
                new MechanismLigament2d("intake", 20, 0, 6, COLOR_GRAY)
            );

            // Shooter indicator (back of robot)
            MechanismRoot2d shooterRoot = mechanism.getRoot("shooter", 50, 15);
            shooterIndicator = shooterRoot.append(
                new MechanismLigament2d("shooter", 25, 0, 8, COLOR_GRAY)
            );

            // Feeder indicator (middle of robot)
            MechanismRoot2d feederRoot = mechanism.getRoot("feeder", 50, 45);
            feederIndicator = feederRoot.append(
                new MechanismLigament2d("feeder", 15, 0, 5, COLOR_GRAY)
            );

            // Hood indicator (above shooter)
            MechanismRoot2d hoodRoot = mechanism.getRoot("hood", 50, 25);
            hoodIndicator = hoodRoot.append(
                new MechanismLigament2d("hood", 10, 45, 4, COLOR_GRAY)
            );

            // Vision indicator (top corners)
            MechanismRoot2d visionRoot = mechanism.getRoot("vision", 80, 80);
            visionIndicator = visionRoot.append(
                new MechanismLigament2d("vision", 8, 135, 4, COLOR_GRAY)
            );

            SmartDashboard.putData(name, mechanism);
        }

        /**
         * Updates all system indicators.
         */
        public void update(
                boolean intakeRunning,
                boolean shooterReady,
                boolean feederRunning,
                boolean hoodCalibrated,
                boolean visionConnected) {

            intakeIndicator.setColor(intakeRunning ? COLOR_GREEN : COLOR_GRAY);
            shooterIndicator.setColor(shooterReady ? COLOR_GREEN : COLOR_GRAY);
            feederIndicator.setColor(feederRunning ? COLOR_GREEN : COLOR_GRAY);
            hoodIndicator.setColor(hoodCalibrated ? COLOR_GREEN : COLOR_RED);
            visionIndicator.setColor(visionConnected ? COLOR_CYAN : COLOR_RED);
        }
    }

    // ==================== MOTOR HEALTH BAR ====================

    /**
     * Creates a horizontal bar representing motor current/temperature.
     */
    public static class MotorHealthBar {
        private final Mechanism2d mechanism;
        private final MechanismLigament2d bar;
        private final MechanismLigament2d outline;

        private final double maxValue;
        private final double warningThreshold;
        private final double criticalThreshold;

        public MotorHealthBar(String name, double maxValue, double warningThreshold, double criticalThreshold) {
            this.maxValue = maxValue;
            this.warningThreshold = warningThreshold;
            this.criticalThreshold = criticalThreshold;

            mechanism = new Mechanism2d(100, 20);

            MechanismRoot2d outlineRoot = mechanism.getRoot("outline", 5, 10);
            outline = outlineRoot.append(
                new MechanismLigament2d("outline", 90, 0, 12, COLOR_DARK_GRAY)
            );

            MechanismRoot2d barRoot = mechanism.getRoot("bar", 7, 10);
            bar = barRoot.append(
                new MechanismLigament2d("bar", 86, 0, 8, COLOR_GREEN)
            );

            SmartDashboard.putData(name, mechanism);
        }

        public void update(double value) {
            // Scale bar length
            double ratio = Math.min(1.0, value / maxValue);
            bar.setLength(Math.max(1, ratio * 86));

            // Color based on thresholds
            if (value >= criticalThreshold) {
                bar.setColor(COLOR_RED);
            } else if (value >= warningThreshold) {
                bar.setColor(COLOR_YELLOW);
            } else {
                bar.setColor(COLOR_GREEN);
            }
        }
    }
}
