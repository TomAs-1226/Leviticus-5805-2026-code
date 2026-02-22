package frc.robot.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.subsystems.*;

/**
 * Main dashboard manager for FRC 5805 Alphabot.
 * Organizes all robot stats into custom Shuffleboard tabs with easy-to-read widgets.
 *
 * Tab Layout:
 * - Overview: All critical stats on ONE page (most important)
 * - Shooter: Detailed shooter motor stats
 * - Intake: Detailed intake stats
 * - Power: Battery and power management details
 * - Vision: Camera and AprilTag details
 */
public class RobotDashboard {

    // Tabs
    private final ShuffleboardTab overviewTab;
    private final ShuffleboardTab shooterTab;
    private final ShuffleboardTab intakeTab;
    private final ShuffleboardTab powerTab;
    private final ShuffleboardTab visionTab;

    // ==================== OVERVIEW TAB ENTRIES ====================

    // Power Section
    private GenericEntry ovBatteryVoltage;
    private GenericEntry ovPowerState;
    private GenericEntry ovSafeToShoot;
    private GenericEntry ovBatteryPercent;

    // Intake Section
    private GenericEntry ovIntakeRunning;
    private GenericEntry ovIntakeRPM;
    private GenericEntry ovIntakeAmps;
    private GenericEntry ovArmState;
    private GenericEntry ovArmStalled;

    // Shooter Section
    private GenericEntry ovShooterAvgRPM;
    private GenericEntry ovShooterTargetRPM;
    private GenericEntry ovShooterTotalAmps;
    private GenericEntry ovShooterAllAlive;
    private GenericEntry ovShooterRPMError;

    // Feeder Section
    private GenericEntry ovFeederRunning;
    private GenericEntry ovFeederM1Alive;
    private GenericEntry ovFeederM2Alive;

    // Hood Section
    private GenericEntry ovHoodAngle;
    private GenericEntry ovHoodCalibrated;
    private GenericEntry ovHoodAtTarget;
    private GenericEntry ovHoodVisionControl;

    // Vision Section
    private GenericEntry ovVisionTagCount;
    private GenericEntry ovHubDetected;
    private GenericEntry ovCam1Connected;
    private GenericEntry ovCam2Connected;
    private GenericEntry ovClosestDistance;
    private GenericEntry ovVisionStatus;

    // ==================== SHOOTER TAB ENTRIES ====================

    private GenericEntry shM1RPM;
    private GenericEntry shM2RPM;
    private GenericEntry shM3RPM;
    private GenericEntry shM4RPM;
    private GenericEntry shM1Amps;
    private GenericEntry shM2Amps;
    private GenericEntry shM3Amps;
    private GenericEntry shM4Amps;
    private GenericEntry shM1Temp;
    private GenericEntry shM2Temp;
    private GenericEntry shM3Temp;
    private GenericEntry shM4Temp;
    private GenericEntry shTargetRPM;
    private GenericEntry shPeakRPM;
    private GenericEntry shPeakAmps;
    private GenericEntry shSpinupCount;
    private GenericEntry shIdleActive;
    private GenericEntry shReversing;
    private GenericEntry shRPMGraph;

    // ==================== INTAKE TAB ENTRIES ====================

    private GenericEntry inRollerRPM;
    private GenericEntry inCurrentAmps;
    private GenericEntry inPowerPercent;
    private GenericEntry inMotorTemp;
    private GenericEntry inPeakAmps;
    private GenericEntry inTotalRotations;
    private GenericEntry inRunning;
    private GenericEntry inMotorAlive;
    private GenericEntry inUptime;
    private GenericEntry inDeployState;
    private GenericEntry inDeployAmps;
    private GenericEntry inDeployStalled;
    private GenericEntry inDeployMoving;
    private GenericEntry inRPMGraph;

    // ==================== POWER TAB ENTRIES ====================

    private GenericEntry pwBatteryVoltage;
    private GenericEntry pwPowerState;
    private GenericEntry pwDischargeRate;
    private GenericEntry pwDischargeSmoothed;
    private GenericEntry pwBrownoutRisk;
    private GenericEntry pwMinVoltage;
    private GenericEntry pwVoltageDrop;
    private GenericEntry pwBatteryHealth;
    private GenericEntry pwTotalCurrent;
    private GenericEntry pwPeakCurrent;
    private GenericEntry pwMaxMotorTemp;
    private GenericEntry pwWarningCount;
    private GenericEntry pwCriticalCount;
    private GenericEntry pwSessionDuration;
    private GenericEntry pwSafeToShoot;
    private GenericEntry pwEstBatteryLife;
    private GenericEntry pwVoltageGraph;

    // ==================== VISION TAB ENTRIES ====================

    private GenericEntry viTotalTagCount;
    private GenericEntry viCam1Tags;
    private GenericEntry viCam2Tags;
    private GenericEntry viCam3Tags;
    private GenericEntry viCam4Tags;
    private GenericEntry viCam1Connected;
    private GenericEntry viCam2Connected;
    private GenericEntry viCam3Connected;
    private GenericEntry viCam4Connected;
    private GenericEntry viMultiTagDetected;
    private GenericEntry viHubDetected;
    private GenericEntry viShooterTrigger;
    private GenericEntry viFeederTrigger;
    private GenericEntry viDetectedIDs;
    private GenericEntry viClosestDistance;
    private GenericEntry viBestYaw;
    private GenericEntry viBestPitch;
    private GenericEntry viBestArea;
    private GenericEntry viBestAmbiguity;
    private GenericEntry viThrottled;
    private GenericEntry viStatus;

    /**
     * Creates the robot dashboard with all tabs and widgets.
     */
    public RobotDashboard() {
        // Create tabs - Overview first so it's the default
        overviewTab = Shuffleboard.getTab("Overview");
        shooterTab = Shuffleboard.getTab("Shooter");
        intakeTab = Shuffleboard.getTab("Intake");
        powerTab = Shuffleboard.getTab("Power");
        visionTab = Shuffleboard.getTab("Vision");

        // Build all tab layouts
        buildOverviewTab();
        buildShooterTab();
        buildIntakeTab();
        buildPowerTab();
        buildVisionTab();

        // Select Overview tab as default
        Shuffleboard.selectTab("Overview");

        System.out.println("[DASHBOARD] Custom Shuffleboard UI initialized");
    }

    // ==================== TAB BUILDERS ====================

    /**
     * Builds the Overview tab with ALL critical stats on one page.
     * Layout: 10 columns x 5 rows
     *
     * | POWER (col 0-1)      | INTAKE (col 2-3)    | SHOOTER (col 4-6)      | VISION (col 7-9)     |
     * | Battery V [dial]     | Running [ind]       | Avg RPM [dial]         | Tags [value]         |
     * | Power State [text]   | RPM [dial]          | Target RPM [value]     | Hub [ind]            |
     * | Safe to Shoot [ind]  | Amps [bar]          | Total Amps [bar]       | Cam1/Cam2 [ind]      |
     * | Battery % [bar]      | Arm State [text]    | All Alive [ind]        | Distance [value]     |
     * |                      | Arm Stalled [ind]   | RPM Error [value]      | Status [text]        |
     * |                      |                     | FEEDER: Running, M1, M2|                      |
     */
    private void buildOverviewTab() {
        // ===== POWER SECTION (columns 0-1) =====
        ovBatteryVoltage = ShuffleboardWidgets.createVoltageGauge(overviewTab, "Battery V", 0, 0, 2, 2);
        ovPowerState = ShuffleboardWidgets.createTextDisplay(overviewTab, "Power State", 0, 2, 2, 1);
        ovSafeToShoot = ShuffleboardWidgets.createStatusIndicator(overviewTab, "Safe to Shoot", 0, 3, 1, 1);
        ovBatteryPercent = ShuffleboardWidgets.createNumberBar(overviewTab, "Battery %", 1, 3, 1, 1, 0, 100);

        // ===== INTAKE SECTION (columns 2-3) =====
        ovIntakeRunning = ShuffleboardWidgets.createActiveIndicator(overviewTab, "Intake ON", 2, 0, 1, 1);
        ovArmStalled = ShuffleboardWidgets.createAlertIndicator(overviewTab, "STALLED", 3, 0, 1, 1);
        ovIntakeRPM = ShuffleboardWidgets.createRPMGauge(overviewTab, "Intake RPM", 2, 1, 2, 2, 6000);
        ovIntakeAmps = ShuffleboardWidgets.createNumberBar(overviewTab, "Intake Amps", 2, 3, 2, 1, 0, 50);
        ovArmState = ShuffleboardWidgets.createTextDisplay(overviewTab, "Arm", 2, 4, 2, 1);

        // ===== SHOOTER SECTION (columns 4-6) =====
        ovShooterAvgRPM = ShuffleboardWidgets.createRPMGauge(overviewTab, "Shooter RPM", 4, 0, 2, 2, 5000);
        ovShooterAllAlive = ShuffleboardWidgets.createStatusIndicator(overviewTab, "Motors OK", 6, 0, 1, 1);
        ovShooterTargetRPM = ShuffleboardWidgets.createValueDisplay(overviewTab, "Target", 6, 1, 1, 1);
        ovShooterTotalAmps = ShuffleboardWidgets.createNumberBar(overviewTab, "Shooter Amps", 4, 2, 3, 1, 0, 320);
        ovShooterRPMError = ShuffleboardWidgets.createValueDisplay(overviewTab, "RPM Error", 4, 3, 1, 1);

        // Feeder sub-section
        ovFeederRunning = ShuffleboardWidgets.createActiveIndicator(overviewTab, "Feeder", 5, 3, 1, 1);
        ovFeederM1Alive = ShuffleboardWidgets.createStatusIndicator(overviewTab, "F1", 6, 3, 1, 1);
        ovFeederM2Alive = ShuffleboardWidgets.createStatusIndicator(overviewTab, "F2", 6, 4, 1, 1);

        // Hood sub-section (row 4)
        ovHoodAngle = ShuffleboardWidgets.createValueDisplay(overviewTab, "Hood°", 4, 4, 1, 1);
        ovHoodCalibrated = ShuffleboardWidgets.createStatusIndicator(overviewTab, "Cal", 5, 4, 1, 1);
        ovHoodAtTarget = ShuffleboardWidgets.createActiveIndicator(overviewTab, "At Tgt", 4, 5, 1, 1);
        ovHoodVisionControl = ShuffleboardWidgets.createActiveIndicator(overviewTab, "Auto", 5, 5, 1, 1);

        // ===== VISION SECTION (columns 7-9) =====
        ovVisionTagCount = ShuffleboardWidgets.createValueDisplay(overviewTab, "Tags", 7, 0, 1, 1);
        ovHubDetected = ShuffleboardWidgets.createActiveIndicator(overviewTab, "HUB", 8, 0, 1, 1);
        ovVisionStatus = ShuffleboardWidgets.createTextDisplay(overviewTab, "Status", 9, 0, 1, 1);
        ovCam1Connected = ShuffleboardWidgets.createStatusIndicator(overviewTab, "CAM1", 7, 1, 1, 1);
        ovCam2Connected = ShuffleboardWidgets.createStatusIndicator(overviewTab, "CAM2", 8, 1, 1, 1);
        ovClosestDistance = ShuffleboardWidgets.createValueDisplay(overviewTab, "Dist (m)", 9, 1, 1, 1);
    }

    /**
     * Builds the detailed Shooter tab.
     */
    private void buildShooterTab() {
        // RPM Gauges Row
        shM1RPM = ShuffleboardWidgets.createRPMGauge(shooterTab, "M1 RPM", 0, 0, 2, 2, 5000);
        shM2RPM = ShuffleboardWidgets.createRPMGauge(shooterTab, "M2 RPM", 2, 0, 2, 2, 5000);
        shM3RPM = ShuffleboardWidgets.createRPMGauge(shooterTab, "M3 RPM", 4, 0, 2, 2, 5000);
        shM4RPM = ShuffleboardWidgets.createRPMGauge(shooterTab, "M4 RPM", 6, 0, 2, 2, 5000);

        // Amps Row
        shM1Amps = ShuffleboardWidgets.createAmpsGauge(shooterTab, "M1 Amps", 0, 2, 2, 1, 80);
        shM2Amps = ShuffleboardWidgets.createAmpsGauge(shooterTab, "M2 Amps", 2, 2, 2, 1, 80);
        shM3Amps = ShuffleboardWidgets.createAmpsGauge(shooterTab, "M3 Amps", 4, 2, 2, 1, 80);
        shM4Amps = ShuffleboardWidgets.createAmpsGauge(shooterTab, "M4 Amps", 6, 2, 2, 1, 80);

        // Temps Row
        shM1Temp = ShuffleboardWidgets.createTempGauge(shooterTab, "M1 Temp", 0, 3, 2, 1);
        shM2Temp = ShuffleboardWidgets.createTempGauge(shooterTab, "M2 Temp", 2, 3, 2, 1);
        shM3Temp = ShuffleboardWidgets.createTempGauge(shooterTab, "M3 Temp", 4, 3, 2, 1);
        shM4Temp = ShuffleboardWidgets.createTempGauge(shooterTab, "M4 Temp", 6, 3, 2, 1);

        // Stats Column (right side)
        shTargetRPM = ShuffleboardWidgets.createValueDisplay(shooterTab, "Target RPM", 8, 0, 2, 1);
        shPeakRPM = ShuffleboardWidgets.createValueDisplay(shooterTab, "Peak RPM", 8, 1, 2, 1);
        shPeakAmps = ShuffleboardWidgets.createValueDisplay(shooterTab, "Peak Amps", 8, 2, 2, 1);
        shSpinupCount = ShuffleboardWidgets.createValueDisplay(shooterTab, "Spinups", 8, 3, 1, 1);
        shIdleActive = ShuffleboardWidgets.createActiveIndicator(shooterTab, "Idle", 9, 3, 1, 1);
        shReversing = ShuffleboardWidgets.createWarningIndicator(shooterTab, "Reverse", 9, 4, 1, 1);

        // RPM Graph (bottom)
        shRPMGraph = ShuffleboardWidgets.createGraph(shooterTab, "RPM History", 0, 4, 8, 2, 10.0);
    }

    /**
     * Builds the detailed Intake tab.
     */
    private void buildIntakeTab() {
        // Main Intake Section
        inRunning = ShuffleboardWidgets.createActiveIndicator(intakeTab, "Running", 0, 0, 1, 1);
        inMotorAlive = ShuffleboardWidgets.createStatusIndicator(intakeTab, "Motor OK", 1, 0, 1, 1);
        inRollerRPM = ShuffleboardWidgets.createRPMGauge(intakeTab, "Roller RPM", 0, 1, 2, 2, 6000);
        inCurrentAmps = ShuffleboardWidgets.createAmpsGauge(intakeTab, "Current", 2, 0, 2, 2, 50);
        inPowerPercent = ShuffleboardWidgets.createPercentGauge(intakeTab, "Power %", 4, 0, 2, 2);
        inMotorTemp = ShuffleboardWidgets.createTempGauge(intakeTab, "Motor Temp", 6, 0, 2, 2);

        // Stats Row
        inPeakAmps = ShuffleboardWidgets.createValueDisplay(intakeTab, "Peak Amps", 2, 2, 2, 1);
        inTotalRotations = ShuffleboardWidgets.createValueDisplay(intakeTab, "Rotations", 4, 2, 2, 1);
        inUptime = ShuffleboardWidgets.createValueDisplay(intakeTab, "Uptime (s)", 6, 2, 2, 1);

        // Deploy Section
        inDeployState = ShuffleboardWidgets.createTextDisplay(intakeTab, "Arm State", 0, 3, 2, 1);
        inDeployMoving = ShuffleboardWidgets.createActiveIndicator(intakeTab, "Moving", 2, 3, 1, 1);
        inDeployStalled = ShuffleboardWidgets.createAlertIndicator(intakeTab, "STALLED", 3, 3, 1, 1);
        inDeployAmps = ShuffleboardWidgets.createAmpsGauge(intakeTab, "Deploy Amps", 4, 3, 2, 1, 80);

        // RPM Graph
        inRPMGraph = ShuffleboardWidgets.createGraph(intakeTab, "RPM History", 0, 4, 8, 2, 10.0);
    }

    /**
     * Builds the detailed Power tab.
     */
    private void buildPowerTab() {
        // Main Voltage Display
        pwBatteryVoltage = ShuffleboardWidgets.createVoltageGauge(powerTab, "Battery Voltage", 0, 0, 3, 3);
        pwPowerState = ShuffleboardWidgets.createTextDisplay(powerTab, "Power State", 3, 0, 2, 1);
        pwSafeToShoot = ShuffleboardWidgets.createStatusIndicator(powerTab, "Safe to Shoot", 5, 0, 1, 1);

        // Discharge Metrics
        pwDischargeRate = ShuffleboardWidgets.createValueDisplay(powerTab, "Discharge V/s", 3, 1, 2, 1);
        pwDischargeSmoothed = ShuffleboardWidgets.createValueDisplay(powerTab, "Smoothed V/s", 5, 1, 2, 1);
        pwBrownoutRisk = ShuffleboardWidgets.createAlertIndicator(powerTab, "Brownout Risk", 7, 1, 1, 1);

        // Battery Health Row
        pwBatteryHealth = ShuffleboardWidgets.createPercentGauge(powerTab, "Battery Health", 3, 2, 2, 2);
        pwMinVoltage = ShuffleboardWidgets.createValueDisplay(powerTab, "Min Voltage", 5, 2, 2, 1);
        pwVoltageDrop = ShuffleboardWidgets.createValueDisplay(powerTab, "Voltage Drop", 5, 3, 2, 1);

        // Current/Temp Row
        pwTotalCurrent = ShuffleboardWidgets.createNumberBar(powerTab, "Total Current", 0, 3, 3, 1, 0, 200);
        pwPeakCurrent = ShuffleboardWidgets.createValueDisplay(powerTab, "Peak Current", 7, 2, 2, 1);
        pwMaxMotorTemp = ShuffleboardWidgets.createTempGauge(powerTab, "Max Motor Temp", 7, 3, 2, 1);

        // Stats Row
        pwWarningCount = ShuffleboardWidgets.createValueDisplay(powerTab, "Warnings", 0, 4, 1, 1);
        pwCriticalCount = ShuffleboardWidgets.createValueDisplay(powerTab, "Criticals", 1, 4, 1, 1);
        pwSessionDuration = ShuffleboardWidgets.createValueDisplay(powerTab, "Session (s)", 2, 4, 2, 1);
        pwEstBatteryLife = ShuffleboardWidgets.createValueDisplay(powerTab, "Est. Life (s)", 4, 4, 2, 1);

        // Voltage Graph
        pwVoltageGraph = ShuffleboardWidgets.createGraph(powerTab, "Voltage History", 0, 5, 9, 2, 30.0);
    }

    /**
     * Builds the detailed Vision tab.
     */
    private void buildVisionTab() {
        // Camera Status Row
        viCam1Connected = ShuffleboardWidgets.createStatusIndicator(visionTab, "CAM1", 0, 0, 1, 1);
        viCam2Connected = ShuffleboardWidgets.createStatusIndicator(visionTab, "CAM2", 1, 0, 1, 1);
        viCam3Connected = ShuffleboardWidgets.createStatusIndicator(visionTab, "CAM3", 2, 0, 1, 1);
        viCam4Connected = ShuffleboardWidgets.createStatusIndicator(visionTab, "CAM4", 3, 0, 1, 1);
        viStatus = ShuffleboardWidgets.createTextDisplay(visionTab, "Vision Status", 4, 0, 2, 1);
        viThrottled = ShuffleboardWidgets.createWarningIndicator(visionTab, "Throttled", 6, 0, 1, 1);

        // Tag Counts Row
        viTotalTagCount = ShuffleboardWidgets.createValueDisplay(visionTab, "Total Tags", 0, 1, 2, 1);
        viCam1Tags = ShuffleboardWidgets.createValueDisplay(visionTab, "Cam1 Tags", 2, 1, 1, 1);
        viCam2Tags = ShuffleboardWidgets.createValueDisplay(visionTab, "Cam2 Tags", 3, 1, 1, 1);
        viCam3Tags = ShuffleboardWidgets.createValueDisplay(visionTab, "Cam3 Tags", 4, 1, 1, 1);
        viCam4Tags = ShuffleboardWidgets.createValueDisplay(visionTab, "Cam4 Tags", 5, 1, 1, 1);

        // Detection Status Row
        viMultiTagDetected = ShuffleboardWidgets.createActiveIndicator(visionTab, "Multi-Tag", 0, 2, 1, 1);
        viHubDetected = ShuffleboardWidgets.createActiveIndicator(visionTab, "HUB", 1, 2, 1, 1);
        viShooterTrigger = ShuffleboardWidgets.createActiveIndicator(visionTab, "Shooter Trig", 2, 2, 1, 1);
        viFeederTrigger = ShuffleboardWidgets.createActiveIndicator(visionTab, "Feeder Trig", 3, 2, 1, 1);
        viDetectedIDs = ShuffleboardWidgets.createTextDisplay(visionTab, "Detected IDs", 4, 2, 3, 1);

        // Best Tag Info
        viClosestDistance = ShuffleboardWidgets.createValueDisplay(visionTab, "Distance (m)", 0, 3, 2, 1);
        viBestYaw = ShuffleboardWidgets.createValueDisplay(visionTab, "Yaw (deg)", 2, 3, 2, 1);
        viBestPitch = ShuffleboardWidgets.createValueDisplay(visionTab, "Pitch (deg)", 4, 3, 2, 1);
        viBestArea = ShuffleboardWidgets.createValueDisplay(visionTab, "Area (%)", 6, 3, 1, 1);
        viBestAmbiguity = ShuffleboardWidgets.createValueDisplay(visionTab, "Ambiguity", 7, 3, 1, 1);
    }

    // ==================== UPDATE METHODS ====================

    /**
     * Updates all dashboard widgets from subsystem data.
     * Call this in robotPeriodic() at ~50Hz.
     */
    public void update(
            IntakeSubsystem intake,
            IntakeDeploySubsystem deploy,
            MotorGroup1Subsystem feeder,
            MotorGroup2Subsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision,
            PowerManagementSubsystem power) {

        updateOverviewTab(intake, deploy, feeder, shooter, hood, vision, power);
        updateShooterTab(shooter);
        updateIntakeTab(intake, deploy);
        updatePowerTab(power);
        updateVisionTab(vision);
    }

    private void updateOverviewTab(
            IntakeSubsystem intake,
            IntakeDeploySubsystem deploy,
            MotorGroup1Subsystem feeder,
            MotorGroup2Subsystem shooter,
            HoodSubsystem hood,
            VisionSubsystem vision,
            PowerManagementSubsystem power) {

        // Power
        ovBatteryVoltage.setDouble(power.getBatteryVoltage());
        ovPowerState.setString(power.getPowerState().name());
        ovSafeToShoot.setBoolean(power.isSafeToShoot());
        ovBatteryPercent.setDouble(power.getBatteryHealthPercent());

        // Intake
        ovIntakeRunning.setBoolean(intake.isRunning());
        ovIntakeRPM.setDouble(intake.getRPM());
        ovIntakeAmps.setDouble(intake.getCurrentAmps());
        ovArmState.setString(deploy.getState().name());
        ovArmStalled.setBoolean(deploy.isStalled());

        // Shooter
        ovShooterAvgRPM.setDouble(shooter.getAverageRPM());
        ovShooterTargetRPM.setDouble(shooter.getTargetRPM());
        ovShooterTotalAmps.setDouble(shooter.getTotalAmps());
        ovShooterAllAlive.setBoolean(shooter.areAllMotorsAlive());
        ovShooterRPMError.setDouble(shooter.getRPMError());

        // Feeder
        ovFeederRunning.setBoolean(feeder.isRunning());
        ovFeederM1Alive.setBoolean(feeder.isMotor1Alive());
        ovFeederM2Alive.setBoolean(feeder.isMotor2Alive());

        // Hood
        ovHoodAngle.setDouble(hood.getAngleDegrees());
        ovHoodCalibrated.setBoolean(hood.isCalibrated());
        ovHoodAtTarget.setBoolean(hood.isAtTarget());
        ovHoodVisionControl.setBoolean(hood.isVisionControlEnabled());

        // Vision
        ovVisionTagCount.setDouble(vision.getTotalTagCount());
        ovHubDetected.setBoolean(vision.isHubDetected());
        ovCam1Connected.setBoolean(vision.isCameraConnected(1));
        ovCam2Connected.setBoolean(vision.isCameraConnected(2));
        ovClosestDistance.setDouble(vision.getClosestDistance());
        ovVisionStatus.setString(vision.getStatus());
    }

    private void updateShooterTab(MotorGroup2Subsystem shooter) {
        // RPM
        double[] rpms = shooter.getMotorRPMs();
        shM1RPM.setDouble(rpms[0]);
        shM2RPM.setDouble(rpms[1]);
        shM3RPM.setDouble(rpms[2]);
        shM4RPM.setDouble(rpms[3]);

        // Amps
        double[] amps = shooter.getMotorAmps();
        shM1Amps.setDouble(amps[0]);
        shM2Amps.setDouble(amps[1]);
        shM3Amps.setDouble(amps[2]);
        shM4Amps.setDouble(amps[3]);

        // Temps
        double[] temps = shooter.getMotorTemps();
        shM1Temp.setDouble(temps[0]);
        shM2Temp.setDouble(temps[1]);
        shM3Temp.setDouble(temps[2]);
        shM4Temp.setDouble(temps[3]);

        // Stats
        shTargetRPM.setDouble(shooter.getTargetRPM());
        shPeakRPM.setDouble(shooter.getPeakRPM());
        shPeakAmps.setDouble(shooter.getPeakAmps());
        shSpinupCount.setDouble(shooter.getSpinupCount());
        shIdleActive.setBoolean(shooter.isIdleActive());
        shReversing.setBoolean(shooter.isReversing());

        // Graph
        shRPMGraph.setDouble(shooter.getAverageRPM());
    }

    private void updateIntakeTab(IntakeSubsystem intake, IntakeDeploySubsystem deploy) {
        // Intake
        inRunning.setBoolean(intake.isRunning());
        inMotorAlive.setBoolean(intake.isMotorAlive());
        inRollerRPM.setDouble(intake.getRPM());
        inCurrentAmps.setDouble(intake.getCurrentAmps());
        inPowerPercent.setDouble(intake.getPowerPercent());
        inMotorTemp.setDouble(intake.getMotorTemp());
        inPeakAmps.setDouble(intake.getPeakCurrent());
        inTotalRotations.setDouble(intake.getTotalRotations());
        inUptime.setDouble(intake.getUptime());

        // Deploy
        inDeployState.setString(deploy.getState().name());
        inDeployMoving.setBoolean(deploy.isMoving());
        inDeployStalled.setBoolean(deploy.isStalled());
        inDeployAmps.setDouble(deploy.getCurrentAmps());

        // Graph
        inRPMGraph.setDouble(intake.getRPM());
    }

    private void updatePowerTab(PowerManagementSubsystem power) {
        pwBatteryVoltage.setDouble(power.getBatteryVoltage());
        pwPowerState.setString(power.getPowerState().name());
        pwSafeToShoot.setBoolean(power.isSafeToShoot());
        pwDischargeRate.setDouble(power.getDischargeRate());
        pwDischargeSmoothed.setDouble(power.getDischargeRateSmoothed());
        pwBrownoutRisk.setBoolean(power.isBrownoutRisk());
        pwBatteryHealth.setDouble(power.getBatteryHealthPercent());
        pwMinVoltage.setDouble(power.getMinVoltage());
        pwVoltageDrop.setDouble(power.getVoltageDrop());
        pwTotalCurrent.setDouble(power.getTotalCurrent());
        pwPeakCurrent.setDouble(power.getPeakCurrent());
        pwMaxMotorTemp.setDouble(power.getMaxMotorTemp());
        pwWarningCount.setDouble(power.getWarningCount());
        pwCriticalCount.setDouble(power.getCriticalCount());
        pwSessionDuration.setDouble(power.getSessionDuration());
        pwEstBatteryLife.setDouble(power.getEstimatedBatteryLife());

        // Graph
        pwVoltageGraph.setDouble(power.getBatteryVoltage());
    }

    private void updateVisionTab(VisionSubsystem vision) {
        // Camera Status
        viCam1Connected.setBoolean(vision.isCameraConnected(1));
        viCam2Connected.setBoolean(vision.isCameraConnected(2));
        viCam3Connected.setBoolean(vision.isCameraConnected(3));
        viCam4Connected.setBoolean(vision.isCameraConnected(4));
        viStatus.setString(vision.getStatus());
        viThrottled.setBoolean(vision.isThrottled());

        // Tag Counts
        viTotalTagCount.setDouble(vision.getTotalTagCount());
        viCam1Tags.setDouble(vision.getCameraTagCount(1));
        viCam2Tags.setDouble(vision.getCameraTagCount(2));
        viCam3Tags.setDouble(vision.getCameraTagCount(3));
        viCam4Tags.setDouble(vision.getCameraTagCount(4));

        // Detection Status
        viMultiTagDetected.setBoolean(vision.isMultiTagDetected());
        viHubDetected.setBoolean(vision.isHubDetected());
        viShooterTrigger.setBoolean(vision.isShooterTrigger());
        viFeederTrigger.setBoolean(vision.isFeederTrigger());
        viDetectedIDs.setString(vision.getDetectedIDsString());

        // Best Tag Info
        viClosestDistance.setDouble(vision.getClosestDistance());
        viBestYaw.setDouble(vision.getBestYaw());
        viBestPitch.setDouble(vision.getBestPitch());
        viBestArea.setDouble(vision.getBestArea());
        viBestAmbiguity.setDouble(vision.getBestAmbiguity());
    }
}
