package frc.robot.shuffleboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.TelemetryConstants;
import frc.robot.subsystems.*;
import frc.robot.shuffleboard.GlassWidgets.*;

import java.util.Map;

/**
 * Unified single-page Glass-compatible dashboard for FRC 5805 Leviticus.
 * 2026 REBUILT Game Field - All critical data on ONE page.
 *
 * Layout (16 columns x 7 rows):
 * +------------------+--------+--------+--------+--------+--------+
 * | FIELD 2D         | SHOOTER RPM DIALS (4x)   | HOOD   | POWER  |
 * | (2026 REBUILT)   | M1  M2 | M3  M4 | AvgRPM | Angle  | Volt   |
 * | 5x4              | 2x2    | 2x2    | 2x2    | 2x2    | 2x2    |
 * +------------------+--------+--------+--------+--------+--------+
 * | INTAKE   | FEEDER   | SHOOTER STATS    | VISION              |
 * | RPM,Amps | M1,M2    | Target,Ready,Err | Cams,Tags,Dist,Trig |
 * | 3x3      | 2x3      | 4x3              | 5x3                 |
 * +------------------+--------+--------+--------+-----------------+
 * | STATUS: NOMINAL - All systems GO              | 2:15 | [ENABLED] |
 * +-----------------------------------------------+------+-----------+
 */
public class UnifiedGlassDashboard {

    // Main tab - everything on ONE page
    private final ShuffleboardTab mainTab;

    // Glass mechanism widgets
    private Field2d fieldWidget;
    private RobotStatusMechanism robotStatusMech;
    private HoodMechanism hoodMech;
    private ShooterMechanism shooterMech;
    private IntakeArmMechanism intakeArmMech;
    private BatteryMechanism batteryMech;

    // NetworkTable for Glass-direct publishing
    private final NetworkTable glassTable;

    // ==================== TOP ROW - RPM DIALS ====================
    private GenericEntry shM1RPM, shM2RPM, shM3RPM, shM4RPM;
    private GenericEntry shAvgRPMDial;
    private GenericEntry hdAngleDial;
    private GenericEntry pwVoltageDial;

    // ==================== INTAKE SECTION ====================
    private GenericEntry inRPMDial;
    private GenericEntry inAmpsDial;
    private GenericEntry inRunning;
    private GenericEntry inMotorOK;
    private GenericEntry inArmState;
    private GenericEntry inStalled;

    // ==================== FEEDER SECTION ====================
    private GenericEntry fdM1RPM;
    private GenericEntry fdM2RPM;
    private GenericEntry fdRunning;
    private GenericEntry fdM1OK;
    private GenericEntry fdM2OK;
    private GenericEntry fdPower;

    // ==================== SHOOTER STATS SECTION ====================
    private GenericEntry shTarget;
    private GenericEntry shError;
    private GenericEntry shReady;
    private GenericEntry shAllOK;
    private GenericEntry shTotalAmps;
    private GenericEntry shPeakRPM;
    private GenericEntry shIdleActive;
    private GenericEntry shReversing;

    // ==================== HOOD SECTION ====================
    private GenericEntry hdTarget;
    private GenericEntry hdCalibrated;
    private GenericEntry hdAtTarget;
    private GenericEntry hdVisionCtrl;

    // ==================== POWER SECTION ====================
    private GenericEntry pwState;
    private GenericEntry pwSafe;
    private GenericEntry pwPercent;
    private GenericEntry pwDischarge;

    // ==================== VISION SECTION ====================
    private GenericEntry viTags;
    private GenericEntry viHub;
    private GenericEntry viDist;
    private GenericEntry viYaw;
    private GenericEntry viCam1, viCam2, viCam3, viCam4;
    private GenericEntry viShooterTrig;
    private GenericEntry viFeederTrig;
    private GenericEntry viMultiTag;

    // ==================== STATUS BAR ====================
    private GenericEntry statusMessage;
    private GenericEntry matchTime;
    private GenericEntry robotEnabled;

    /**
     * Creates the unified Glass-compatible dashboard for 2026 REBUILT.
     */
    public UnifiedGlassDashboard() {
        // Create single main tab
        mainTab = Shuffleboard.getTab("Leviticus");

        // Get Glass-compatible NetworkTable
        glassTable = NetworkTableInstance.getDefault().getTable("Glass");

        // Build all sections with proper sizing
        buildFieldWidget();
        buildTopRowDials();
        buildIntakeSection();
        buildFeederSection();
        buildShooterStatsSection();
        buildVisionSection();
        buildStatusBar();
        buildGlassMechanisms();

        // Select this tab as default
        Shuffleboard.selectTab("Leviticus");

        System.out.println("[DASHBOARD] 2026 REBUILT Glass Dashboard - ALL ON ONE PAGE");
    }

    // ==================== FIELD WIDGET (2026 REBUILT) ====================

    private void buildFieldWidget() {
        // Field2d - Robot position on 2026 REBUILT field
        fieldWidget = new Field2d();

        // Add field game pieces for 2026 REBUILT
        // HUB locations (center of field)
        fieldWidget.getObject("RedHUB").setPose(
            new Pose2d(TelemetryConstants.kFieldLengthMeters - 3.0,
                       TelemetryConstants.kFieldWidthMeters / 2,
                       new edu.wpi.first.math.geometry.Rotation2d()));
        fieldWidget.getObject("BlueHUB").setPose(
            new Pose2d(3.0,
                       TelemetryConstants.kFieldWidthMeters / 2,
                       new edu.wpi.first.math.geometry.Rotation2d()));

        SmartDashboard.putData("Field2026", fieldWidget);

        // Add to Shuffleboard with large size for visibility
        mainTab.add("2026 Field", fieldWidget)
            .withWidget(BuiltInWidgets.kField)
            .withPosition(0, 0)
            .withSize(5, 4)
            .withProperties(Map.of(
                "field_game", "REBUILT 2026",
                "robot_width", 0.75,
                "robot_length", 0.75
            ));
    }

    // ==================== TOP ROW - CRITICAL RPM DIALS ====================

    private void buildTopRowDials() {
        // Shooter Motor 1 RPM - Large dial
        shM1RPM = mainTab.add("M1 RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 5000.0, "showValue", true))
            .withPosition(5, 0)
            .withSize(2, 2)
            .getEntry();

        // Shooter Motor 2 RPM
        shM2RPM = mainTab.add("M2 RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 5000.0, "showValue", true))
            .withPosition(7, 0)
            .withSize(2, 2)
            .getEntry();

        // Shooter Motor 3 RPM
        shM3RPM = mainTab.add("M3 RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 5000.0, "showValue", true))
            .withPosition(5, 2)
            .withSize(2, 2)
            .getEntry();

        // Shooter Motor 4 RPM
        shM4RPM = mainTab.add("M4 RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 5000.0, "showValue", true))
            .withPosition(7, 2)
            .withSize(2, 2)
            .getEntry();

        // Average Shooter RPM - Prominent dial
        shAvgRPMDial = mainTab.add("Avg RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 5000.0, "showValue", true))
            .withPosition(9, 0)
            .withSize(2, 2)
            .getEntry();

        // Hood Angle Dial
        hdAngleDial = mainTab.add("Hood Angle", 30.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 15.0, "max", 75.0, "showValue", true))
            .withPosition(11, 0)
            .withSize(2, 2)
            .getEntry();

        // Battery Voltage Dial - Critical for power monitoring
        pwVoltageDial = mainTab.add("Battery V", 12.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 9.0, "max", 14.0, "showValue", true))
            .withPosition(13, 0)
            .withSize(2, 2)
            .getEntry();

        // Hood status indicators below hood dial
        ShuffleboardLayout hoodStatus = mainTab.getLayout("Hood", BuiltInLayouts.kGrid)
            .withPosition(11, 2)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));

        hdTarget = hoodStatus.add("Target", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        hdCalibrated = hoodStatus.add("Cal", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        hdAtTarget = hoodStatus.add("AtTgt", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#404040"))
            .getEntry();
        hdVisionCtrl = hoodStatus.add("Auto", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FFFF", "colorWhenFalse", "#404040"))
            .getEntry();

        // Power status indicators below voltage dial
        ShuffleboardLayout powerStatus = mainTab.getLayout("Power", BuiltInLayouts.kGrid)
            .withPosition(13, 2)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));

        pwState = powerStatus.add("State", "---")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        pwSafe = powerStatus.add("Safe", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        pwPercent = powerStatus.add("Health", 100.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0.0, "max", 100.0))
            .getEntry();
        pwDischarge = powerStatus.add("V/s", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
    }

    // ==================== INTAKE SECTION ====================

    private void buildIntakeSection() {
        // Intake RPM Dial - Large
        inRPMDial = mainTab.add("Intake RPM", 0.0)
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0.0, "max", 6000.0, "showValue", true))
            .withPosition(0, 4)
            .withSize(2, 2)
            .getEntry();

        // Intake status grid
        ShuffleboardLayout intakeStatus = mainTab.getLayout("Intake", BuiltInLayouts.kGrid)
            .withPosition(2, 4)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 3));

        inRunning = intakeStatus.add("Run", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#404040"))
            .getEntry();
        inMotorOK = intakeStatus.add("Motor", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        inAmpsDial = intakeStatus.add("Amps", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0.0, "max", 50.0))
            .getEntry();
        inArmState = intakeStatus.add("Arm", "---")
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        inStalled = intakeStatus.add("STALL", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#FF0000", "colorWhenFalse", "#404040"))
            .getEntry();
    }

    // ==================== FEEDER SECTION ====================

    private void buildFeederSection() {
        ShuffleboardLayout feederLayout = mainTab.getLayout("Feeder", BuiltInLayouts.kGrid)
            .withPosition(4, 4)
            .withSize(2, 2)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 3));

        fdM1RPM = feederLayout.add("M1 RPM", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        fdM2RPM = feederLayout.add("M2 RPM", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        fdRunning = feederLayout.add("Run", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#404040"))
            .getEntry();
        fdPower = feederLayout.add("Pwr%", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        fdM1OK = feederLayout.add("M1", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        fdM2OK = feederLayout.add("M2", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
    }

    // ==================== SHOOTER STATS SECTION ====================

    private void buildShooterStatsSection() {
        ShuffleboardLayout shooterStats = mainTab.getLayout("Shooter Stats", BuiltInLayouts.kGrid)
            .withPosition(6, 4)
            .withSize(3, 2)
            .withProperties(Map.of("Number of columns", 3, "Number of rows", 3));

        shTarget = shooterStats.add("Target", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        shError = shooterStats.add("Error", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        shTotalAmps = shooterStats.add("Amps", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();

        shReady = shooterStats.add("READY", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#404040"))
            .getEntry();
        shAllOK = shooterStats.add("AllOK", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        shIdleActive = shooterStats.add("Idle", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#FFFF00", "colorWhenFalse", "#404040"))
            .getEntry();

        shPeakRPM = shooterStats.add("Peak", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        shReversing = shooterStats.add("REV!", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#FF0000", "colorWhenFalse", "#404040"))
            .getEntry();
    }

    // ==================== VISION SECTION ====================

    private void buildVisionSection() {
        ShuffleboardLayout visionLayout = mainTab.getLayout("Vision", BuiltInLayouts.kGrid)
            .withPosition(9, 4)
            .withSize(6, 2)
            .withProperties(Map.of("Number of columns", 6, "Number of rows", 2));

        // Row 1: Camera status
        viCam1 = visionLayout.add("CAM1", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        viCam2 = visionLayout.add("CAM2", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        viCam3 = visionLayout.add("CAM3", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        viCam4 = visionLayout.add("CAM4", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .getEntry();
        viTags = visionLayout.add("Tags", 0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        viMultiTag = visionLayout.add("Multi", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FFFF", "colorWhenFalse", "#404040"))
            .getEntry();

        // Row 2: Target data and triggers
        viHub = visionLayout.add("HUB", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#404040"))
            .getEntry();
        viDist = visionLayout.add("Dist m", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        viYaw = visionLayout.add("Yaw", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .getEntry();
        viShooterTrig = visionLayout.add("ShTRIG", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#FFFF00", "colorWhenFalse", "#404040"))
            .getEntry();
        viFeederTrig = visionLayout.add("FdTRIG", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#FFFF00", "colorWhenFalse", "#404040"))
            .getEntry();
    }

    // ==================== STATUS BAR ====================

    private void buildStatusBar() {
        statusMessage = mainTab.add("Status", "READY - 2026 REBUILT")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 6)
            .withSize(11, 1)
            .getEntry();

        matchTime = mainTab.add("Time", "--:--")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(11, 6)
            .withSize(2, 1)
            .getEntry();

        robotEnabled = mainTab.add("EN", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("colorWhenTrue", "#00FF00", "colorWhenFalse", "#FF0000"))
            .withPosition(13, 6)
            .withSize(2, 1)
            .getEntry();
    }

    // ==================== GLASS MECHANISMS ====================

    private void buildGlassMechanisms() {
        // These publish to NetworkTables for Glass visualization
        robotStatusMech = new RobotStatusMechanism("RobotStatus");
        hoodMech = new HoodMechanism("HoodMech");
        shooterMech = new ShooterMechanism("ShooterMech");
        intakeArmMech = new IntakeArmMechanism("IntakeArmMech");
        batteryMech = new BatteryMechanism("BatteryMech");
    }

    // ==================== UPDATE METHODS ====================

    public void update(
            IntakeSubsystem intake,
            IntakeDeploySubsystem deploy,
            MotorGroup1Subsystem feeder,
            MotorGroup2Subsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision,
            PowerManagementSubsystem power,
            TelemetrySubsystem telemetry) {

        updateTopRowDials(shooter, hood, power);
        updateIntakeSection(intake, deploy);
        updateFeederSection(feeder);
        updateShooterStats(shooter);
        updateVisionSection(vision);
        updateStatusBar(power);
        updateMechanisms(intake, deploy, shooter, hood, vision, power, telemetry);
    }

    public void update(
            IntakeSubsystem intake,
            IntakeDeploySubsystem deploy,
            MotorGroup1Subsystem feeder,
            MotorGroup2Subsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision,
            PowerManagementSubsystem power) {
        update(intake, deploy, feeder, shooter, hood, vision, power, null);
    }

    private void updateTopRowDials(MotorGroup2Subsystem shooter, HoodSubsystem hood,
                                    PowerManagementSubsystem power) {
        double[] rpms = shooter.getMotorRPMs();
        shM1RPM.setDouble(Math.round(rpms[0]));
        shM2RPM.setDouble(Math.round(rpms[1]));
        shM3RPM.setDouble(Math.round(rpms[2]));
        shM4RPM.setDouble(Math.round(rpms[3]));
        shAvgRPMDial.setDouble(Math.round(shooter.getAverageRPM()));

        hdAngleDial.setDouble(Math.round(hood.getAngleDegrees() * 10.0) / 10.0);
        hdTarget.setDouble(Math.round(hood.getTargetAngleDegrees() * 10.0) / 10.0);
        hdCalibrated.setBoolean(hood.isCalibrated());
        hdAtTarget.setBoolean(hood.isAtTarget());
        hdVisionCtrl.setBoolean(hood.isVisionControlEnabled());

        pwVoltageDial.setDouble(power.getBatteryVoltage());
        pwState.setString(power.getPowerState().name());
        pwSafe.setBoolean(power.isSafeToShoot());
        pwPercent.setDouble(power.getBatteryHealthPercent());
        pwDischarge.setDouble(Math.round(power.getDischargeRate() * 1000.0) / 1000.0);
    }

    private void updateIntakeSection(IntakeSubsystem intake, IntakeDeploySubsystem deploy) {
        inRPMDial.setDouble(Math.round(intake.getRPM()));
        inRunning.setBoolean(intake.isRunning());
        inMotorOK.setBoolean(intake.isMotorAlive());
        inAmpsDial.setDouble(intake.getCurrentAmps());
        inArmState.setString(deploy.getState().name());
        inStalled.setBoolean(deploy.isStalled());
    }

    private void updateFeederSection(MotorGroup1Subsystem feeder) {
        fdM1RPM.setDouble(Math.round(feeder.getMotor1RPM()));
        fdM2RPM.setDouble(Math.round(feeder.getMotor2RPM()));
        fdRunning.setBoolean(feeder.isRunning());
        fdPower.setDouble(Math.round(feeder.getPowerPercent() * 100.0));
        fdM1OK.setBoolean(feeder.isMotor1Alive());
        fdM2OK.setBoolean(feeder.isMotor2Alive());
    }

    private void updateShooterStats(MotorGroup2Subsystem shooter) {
        shTarget.setDouble(shooter.getTargetRPM());
        shError.setDouble(Math.round(shooter.getRPMError()));
        shTotalAmps.setDouble(Math.round(shooter.getTotalAmps() * 10.0) / 10.0);
        shReady.setBoolean(shooter.isAtTargetRPM());
        shAllOK.setBoolean(shooter.areAllMotorsAlive());
        shIdleActive.setBoolean(shooter.isIdleActive());
        shPeakRPM.setDouble(Math.round(shooter.getPeakRPM()));
        shReversing.setBoolean(shooter.isReversing());
    }

    private void updateVisionSection(VisionSubsystem vision) {
        viCam1.setBoolean(vision.isCameraConnected(1));
        viCam2.setBoolean(vision.isCameraConnected(2));
        viCam3.setBoolean(vision.isCameraConnected(3));
        viCam4.setBoolean(vision.isCameraConnected(4));
        viTags.setDouble(vision.getTotalTagCount());
        viMultiTag.setBoolean(vision.isMultiTagDetected());

        viHub.setBoolean(vision.isHubDetected());
        viDist.setDouble(Math.round(vision.getClosestDistance() * 100.0) / 100.0);
        viYaw.setDouble(Math.round(vision.getBestYaw() * 10.0) / 10.0);
        viShooterTrig.setBoolean(vision.isShooterTrigger());
        viFeederTrig.setBoolean(vision.isFeederTrigger());
    }

    private void updateStatusBar(PowerManagementSubsystem power) {
        String status;
        switch (power.getPowerState()) {
            case EMERGENCY:
                status = "EMERGENCY - LOW BATTERY! REDUCE LOAD";
                break;
            case CRITICAL:
                status = "CRITICAL - Battery below 11V - Conserve power";
                break;
            case WARNING:
                status = "WARNING - Battery getting low - Monitor closely";
                break;
            case NOMINAL:
            default:
                status = "NOMINAL - 2026 REBUILT - All systems GO";
        }
        statusMessage.setString(status);

        double time = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
        if (time < 0) {
            matchTime.setString("--:--");
        } else {
            int minutes = (int) (time / 60);
            int seconds = (int) (time % 60);
            matchTime.setString(String.format("%d:%02d", minutes, seconds));
        }

        robotEnabled.setBoolean(edu.wpi.first.wpilibj.DriverStation.isEnabled());
    }

    private void updateMechanisms(
            IntakeSubsystem intake,
            IntakeDeploySubsystem deploy,
            MotorGroup2Subsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision,
            PowerManagementSubsystem power,
            TelemetrySubsystem telemetry) {

        // Update Field2d with robot position
        if (telemetry != null) {
            Pose2d pose = telemetry.getRobotPose();
            if (pose != null) {
                fieldWidget.setRobotPose(pose);
            }
        }

        // Update Glass mechanisms
        robotStatusMech.update(
            intake.isRunning(),
            shooter.isAtTargetRPM(),
            shooter.getTotalAmps() > 0,
            hood.isCalibrated(),
            vision.getTotalTagCount() > 0
        );

        hoodMech.update(
            hood.getAngleDegrees(),
            hood.getTargetAngleDegrees(),
            hood.isCalibrated(),
            hood.isAtTarget()
        );

        double[] rpms = shooter.getMotorRPMs();
        boolean[] alive = {
            shooter.isMotor1Alive(),
            shooter.isMotor2Alive(),
            shooter.isMotor3Alive(),
            shooter.isMotor4Alive()
        };
        shooterMech.update(rpms, alive, shooter.getTargetRPM());

        intakeArmMech.update(
            deploy.getState().name(),
            intake.isRunning(),
            deploy.isStalled()
        );

        batteryMech.update(power.getBatteryVoltage(), power.getBatteryHealthPercent());

        // Update vision target on field
        if (vision.isHubDetected()) {
            double dist = vision.getClosestDistance();
            double yaw = Math.toRadians(vision.getBestYaw());
            Pose2d robotPose = telemetry != null ? telemetry.getRobotPose() : new Pose2d();
            double targetX = robotPose.getX() + dist * Math.cos(robotPose.getRotation().getRadians() + yaw);
            double targetY = robotPose.getY() + dist * Math.sin(robotPose.getRotation().getRadians() + yaw);
            fieldWidget.getObject("Target").setPose(new Pose2d(targetX, targetY,
                new edu.wpi.first.math.geometry.Rotation2d()));
        }
    }

    public void publishGlassExtras(
            MotorGroup2Subsystem shooter,
            VisionSubsystem vision,
            TelemetrySubsystem telemetry) {
        if (telemetry != null) {
            glassTable.getEntry("shot_count").setInteger(telemetry.getTotalShotCount());
        }
    }
}
