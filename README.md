# FRC Team 5805 - Alphabot 2026 Robot Code

![Java](https://img.shields.io/badge/Java-17-ED8B00?style=flat-square&logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026-00629B?style=flat-square&logo=first&logoColor=white)
![Phoenix 6](https://img.shields.io/badge/Phoenix_6-25.0-FF6B00?style=flat-square&logo=ctre&logoColor=white)
![AprilVision](https://img.shields.io/badge/AprilVision-3.2-4CAF50?style=flat-square)
![Gradle](https://img.shields.io/badge/Gradle-8.5-02303A?style=flat-square&logo=gradle&logoColor=white)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen?style=flat-square)
![Tests](https://img.shields.io/badge/Tests-Passing-brightgreen?style=flat-square)
![License](https://img.shields.io/badge/License-WPILib_BSD-blue?style=flat-square)

Competition-ready code for the 2026 FIRST Robotics Competition season.

---

## Quick Stats

| Metric | Value |
|--------|-------|
| ![Subsystems](https://img.shields.io/badge/Subsystems-8-purple?style=flat-square) | PowerManagement, Vision, Shooter, Feeder, Hood, Intake, IntakeDeploy, Telemetry |
| ![Motors](https://img.shields.io/badge/Motors-9-orange?style=flat-square) | 4x Shooter + 2x Feeder + 1x Intake + 1x Deploy + 1x Hood |
| ![Cameras](https://img.shields.io/badge/Cameras-4-green?style=flat-square) | Multi-camera AprilTag vision system |
| ![Tabs](https://img.shields.io/badge/Dashboard_Tabs-5-blue?style=flat-square) | Overview, Shooter, Intake, Power, Vision |

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Configuration](#hardware-configuration)
- [Software Architecture](#software-architecture)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [Controls](#controls)
- [Dashboard](#dashboard)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This repository contains the robot code for FRC Team 5805's 2026 competition robot. The codebase is built on WPILib's command-based framework with Phoenix 6 motor control and AprilVision 3.2 for vision processing.

### Technologies Used

| Technology | Badge | Purpose |
|------------|-------|---------|
| WPILib | ![WPILib](https://img.shields.io/badge/WPILib-2026-00629B?style=flat-square) | FRC robot framework |
| Java | ![Java](https://img.shields.io/badge/Java-17-ED8B00?style=flat-square) | Programming language |
| Phoenix 6 | ![Phoenix](https://img.shields.io/badge/Phoenix_6-25.0-FF6B00?style=flat-square) | CTRE motor controllers |
| AprilVision | ![Vision](https://img.shields.io/badge/AprilVision-3.2-4CAF50?style=flat-square) | AprilTag detection |
| Shuffleboard | ![Shuffle](https://img.shields.io/badge/Shuffleboard-2026-9C27B0?style=flat-square) | Custom dashboard |
| Jackson | ![Jackson](https://img.shields.io/badge/Jackson-2.15-blue?style=flat-square) | JSON configuration |
| Gradle | ![Gradle](https://img.shields.io/badge/Gradle-8.5-02303A?style=flat-square) | Build system |

---

## Features

### Vision-Guided Shooting
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)
![Cameras](https://img.shields.io/badge/Cameras-4-blue?style=flat-square)
![Filter](https://img.shields.io/badge/Filter-Kalman-purple?style=flat-square)

- AprilTag detection with multi-camera support (up to 4 cameras)
- Distance-based RPM calculation from JSON lookup tables
- Linear interpolation for smooth RPM curves between data points
- Automatic alignment with controller rumble feedback
- Failsafe operation with any number of cameras
- Kalman filtering for stable distance readings

### Adjustable Hood System
![Motor](https://img.shields.io/badge/Motor-Kraken_X44-FF6B00?style=flat-square)
![Encoder](https://img.shields.io/badge/Encoder-CANCoder-FF6B00?style=flat-square)
![Control](https://img.shields.io/badge/Control-Motion_Magic-9C27B0?style=flat-square)

- Kraken X44 motor with CANCoder for precise position control
- Motion Magic profiling for smooth, jitter-free movement
- Anti-jitter features: deadband, low-pass filtering, settling counter
- Hardstop calibration for automatic range detection
- Vision-based automatic angle adjustment based on target distance
- Angle range: 15 to 45 degrees

### Power Management
![Protection](https://img.shields.io/badge/Protection-Multi_Tier-yellow?style=flat-square)
![States](https://img.shields.io/badge/States-4-orange?style=flat-square)
![Monitoring](https://img.shields.io/badge/Monitoring-Real_Time-brightgreen?style=flat-square)

- Real-time battery monitoring with discharge rate tracking
- Multi-tier throttling system (NOMINAL, WARNING, CRITICAL, EMERGENCY)
- Partial vision throttle mode to conserve power
- Priority-based subsystem protection (drive and shooter are never throttled)
- Optimistic battery life prediction with exponential smoothing

### 4-Motor Shooter
![Motors](https://img.shields.io/badge/Motors-4x_Kraken_X60-FF6B00?style=flat-square)
![Control](https://img.shields.io/badge/Control-FOC-9C27B0?style=flat-square)
![Idle](https://img.shields.io/badge/Idle_RPM-500-blue?style=flat-square)

- Four Kraken X60 motors in follower configuration
- Phoenix 6 velocity control with FOC
- JSON-configurable distance-to-RPM lookup table
- Idle mode at 500 RPM for faster spin-up
- Per-motor telemetry (RPM, current, temperature)

### Custom Shuffleboard Dashboard
![Tabs](https://img.shields.io/badge/Tabs-5-blue?style=flat-square)
![Update](https://img.shields.io/badge/Update_Rate-50Hz-brightgreen?style=flat-square)
![Widgets](https://img.shields.io/badge/Widgets-Custom-purple?style=flat-square)

- Overview tab with all critical stats on one page
- Real-time graphs for RPM, voltage, and current
- Dedicated tabs for Shooter, Intake, Power, and Vision details
- Real-time updates at 50Hz
- Custom widget factory for consistent styling

---

## Hardware Configuration

### Motor Controllers

| Subsystem | Motor Type | Qty | CAN IDs | Status |
|-----------|------------|-----|---------|--------|
| Shooter | ![Kraken](https://img.shields.io/badge/Kraken_X60-FF6B00?style=flat-square) | 4 | 1-4 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Feeder | ![Kraken](https://img.shields.io/badge/Kraken_X60-FF6B00?style=flat-square) | 2 | 5-6 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Intake Roller | ![TalonFX](https://img.shields.io/badge/TalonFX-FF6B00?style=flat-square) | 1 | 10 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Intake Deploy | ![TalonFX](https://img.shields.io/badge/TalonFX-FF6B00?style=flat-square) | 1 | 11 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Hood | ![Kraken](https://img.shields.io/badge/Kraken_X44-FF6B00?style=flat-square) | 1 | 17 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |

### Sensors

| Sensor | Type | CAN ID | Status |
|--------|------|--------|--------|
| Hood Encoder | ![CANCoder](https://img.shields.io/badge/CANCoder-FF6B00?style=flat-square) | 18 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| IMU | ![Pigeon2](https://img.shields.io/badge/Pigeon_2-FF6B00?style=flat-square) | 20 | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |

### Vision System

| Camera | Name | Priority | Status |
|--------|------|----------|--------|
| Primary | cam1 | ![Required](https://img.shields.io/badge/-Required-red?style=flat-square) | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Secondary | cam2 | ![Optional](https://img.shields.io/badge/-Optional-yellow?style=flat-square) | ![Active](https://img.shields.io/badge/-Active-brightgreen?style=flat-square) |
| Tertiary | cam3 | ![Optional](https://img.shields.io/badge/-Optional-yellow?style=flat-square) | ![Standby](https://img.shields.io/badge/-Standby-blue?style=flat-square) |
| Quaternary | cam4 | ![Optional](https://img.shields.io/badge/-Optional-yellow?style=flat-square) | ![Standby](https://img.shields.io/badge/-Standby-blue?style=flat-square) |

---

## Software Architecture

```
                              +------------------+
                              |  RobotContainer  |
                              +--------+---------+
                                       |
       +---------------+---------------+---------------+---------------+
       |               |               |               |               |
+------v------+ +------v------+ +------v------+ +------v------+ +------v------+
|   Vision    | |   Shooter   | |   Feeder    | |    Hood     | |   Intake    |
| (4 cameras) | | (4 motors)  | | (2 motors)  | | (1 motor)   | | (2 motors)  |
+------+------+ +------+------+ +------+------+ +------+------+ +------+------+
       |               |               |               |               |
       +---------------+-------+-------+---------------+---------------+
                               |
                    +----------v-----------+
                    |  PowerManagement     |
                    |  (Battery Monitor)   |
                    +----------+-----------+
                               |
                    +----------v-----------+
                    |   RobotDashboard     |
                    |   (Shuffleboard)     |
                    +----------------------+
```

### Subsystem Status

| Subsystem | Badge | Description |
|-----------|-------|-------------|
| PowerManagement | ![Power](https://img.shields.io/badge/Power-Online-brightgreen?style=flat-square) | Battery monitoring and throttle control |
| VisionSubsystem | ![Vision](https://img.shields.io/badge/Vision-Online-brightgreen?style=flat-square) | AprilTag detection (4 cameras) |
| MotorGroup2 | ![Shooter](https://img.shields.io/badge/Shooter-Online-brightgreen?style=flat-square) | Shooter (4x Kraken X60) |
| MotorGroup1 | ![Feeder](https://img.shields.io/badge/Feeder-Online-brightgreen?style=flat-square) | Feeder (2x Kraken X60) |
| HoodSubsystem | ![Hood](https://img.shields.io/badge/Hood-Online-brightgreen?style=flat-square) | Adjustable hood angle |
| IntakeSubsystem | ![Intake](https://img.shields.io/badge/Intake-Online-brightgreen?style=flat-square) | Intake rollers |
| IntakeDeploy | ![Deploy](https://img.shields.io/badge/Deploy-Online-brightgreen?style=flat-square) | Intake arm deployment |
| Telemetry | ![Telemetry](https://img.shields.io/badge/Telemetry-Online-brightgreen?style=flat-square) | Data logging and NetworkTables |

### Key Classes

| Class | Description |
|-------|-------------|
| ShooterConfig | Loads distance-to-RPM lookup table from JSON |
| VisionControlCommand | Auto-aim and shoot command with vision tracking |
| HoodSubsystem | Position-controlled hood with calibration and anti-jitter |
| RobotDashboard | Custom Shuffleboard UI with 5 tabs |
| ShuffleboardWidgets | Reusable widget factory |

---

## Getting Started

### Prerequisites

![Java](https://img.shields.io/badge/Java-17+-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026-00629B?style=for-the-badge)
![Git](https://img.shields.io/badge/Git-Required-F05032?style=for-the-badge&logo=git&logoColor=white)
![VSCode](https://img.shields.io/badge/VS_Code-Recommended-007ACC?style=for-the-badge&logo=visual-studio-code&logoColor=white)

### Build and Deploy

```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Clean build
./gradlew clean build
```

### Simulation

```bash
./gradlew simulateJava
```

---

## Configuration

### Shooter RPM Tuning

Edit `src/main/deploy/shooter_config.json` to adjust the distance-to-RPM lookup table:

```json
{
  "version": "1.0",
  "table": [
    {"distance_meters": 1.0, "rpm": 1500, "hood_angle_degrees": 18.0, "notes": "Close shot"},
    {"distance_meters": 2.0, "rpm": 2500, "hood_angle_degrees": 25.0, "notes": "Mid range"},
    {"distance_meters": 3.0, "rpm": 3500, "hood_angle_degrees": 33.0, "notes": "Far shot"},
    {"distance_meters": 4.0, "rpm": 4500, "hood_angle_degrees": 42.0, "notes": "Maximum range"}
  ]
}
```

Changes take effect on the next deploy without recompiling code.

### Constants

All hardware IDs, PID gains, and thresholds are centralized in `Constants.java`:

| Category | Examples |
|----------|----------|
| Motor IDs | CAN IDs for all motors |
| Vision | Camera names, offsets, thresholds |
| Power | Voltage thresholds for throttling |
| Hood | Motion profile, PID gains, limits |

---

## Controls

### PS5 Controller Mapping

| Button | Action | Mode | Badge |
|--------|--------|------|-------|
| Triangle | Auto-aim and shoot | Vision | ![Vision](https://img.shields.io/badge/-Vision-4CAF50?style=flat-square) |
| Square | Auto-deploy intake and run rollers | Auto | ![Auto](https://img.shields.io/badge/-Auto-2196F3?style=flat-square) |
| Cross | Run feeder manually | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| Circle | Eject (reverse intake) | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| L1 | Shooter forward | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| R1 | Shooter reverse | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| D-Pad Up | Deploy intake arm up | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| D-Pad Down | Deploy intake arm down | Manual | ![Manual](https://img.shields.io/badge/-Manual-9E9E9E?style=flat-square) |
| L3 | Toggle speed profile | Config | ![Config](https://img.shields.io/badge/-Config-FF9800?style=flat-square) |
| Options | Reset shot counter | Config | ![Config](https://img.shields.io/badge/-Config-FF9800?style=flat-square) |

---

## Dashboard

### Tab Overview

| Tab | Badge | Purpose | Key Widgets |
|-----|-------|---------|-------------|
| Overview | ![Overview](https://img.shields.io/badge/Overview-Primary-blue?style=flat-square) | All critical stats | Battery, RPM gauges, status indicators |
| Shooter | ![Shooter](https://img.shields.io/badge/Shooter-Detail-orange?style=flat-square) | Motor details | 4x RPM dials, amps bars, temp gauges, graphs |
| Intake | ![Intake](https://img.shields.io/badge/Intake-Detail-green?style=flat-square) | Roller/arm details | RPM graph, current bar, state display |
| Power | ![Power](https://img.shields.io/badge/Power-Detail-yellow?style=flat-square) | Battery health | Voltage graph, discharge rate, life estimate |
| Vision | ![Vision](https://img.shields.io/badge/Vision-Detail-purple?style=flat-square) | Camera status | Tag count, distance, alignment |

### Overview Tab Layout

```
+----------------+----------------+------------------+----------------+
| POWER          | INTAKE         | SHOOTER          | VISION         |
+----------------+----------------+------------------+----------------+
| Battery V      | Running        | Avg RPM [dial]   | Tags           |
| Power State    | RPM [dial]     | Target RPM       | HUB Detected   |
| Safe to Shoot  | Amps [bar]     | Total Amps [bar] | Cam Status     |
| Battery %      | Arm State      | Motors OK        | Distance       |
+----------------+----------------+------------------+----------------+
|                | HOOD           |                  |                |
|                | Angle, Cal, Auto                  |                |
+----------------+----------------+------------------+----------------+
| GRAPHS: Shooter RPM | Intake RPM | Battery Voltage               |
+---------------------------------------------------------------+
```

### Widget Types

| Widget | Badge | Purpose |
|--------|-------|---------|
| Dial Gauge | ![Dial](https://img.shields.io/badge/Dial-RPM%2FVoltage-blue?style=flat-square) | RPM, voltage, temperature display |
| Number Bar | ![Bar](https://img.shields.io/badge/Bar-Current-orange?style=flat-square) | Current draw, percentages |
| Boolean Box | ![Bool](https://img.shields.io/badge/Bool-Status-green?style=flat-square) | On/off states, alerts |
| Graph | ![Graph](https://img.shields.io/badge/Graph-History-purple?style=flat-square) | Real-time data trends |
| Text View | ![Text](https://img.shields.io/badge/Text-Info-gray?style=flat-square) | State names, messages |

---

## Contributing

1. Create a feature branch from main
2. Make changes and test thoroughly
3. Commit with descriptive messages
4. Push and open a pull request

### Code Style

| Rule | Badge | Description |
|------|-------|-------------|
| Architecture | ![Arch](https://img.shields.io/badge/Arch-Command_Based-blue?style=flat-square) | WPILib command-based patterns |
| Documentation | ![Docs](https://img.shields.io/badge/Docs-Javadoc-green?style=flat-square) | Javadoc comments for public APIs |
| Logging | ![Log](https://img.shields.io/badge/Log-Prefixed-yellow?style=flat-square) | Prefix with subsystem name: `[SUBSYSTEM] message` |
| Configuration | ![Config](https://img.shields.io/badge/Config-Centralized-orange?style=flat-square) | Keep Constants.java as single source of truth |

---

## License

![License](https://img.shields.io/badge/License-WPILib_BSD-blue?style=for-the-badge)

This project is licensed under the WPILib BSD License. See LICENSE file for details.

Copyright (c) FIRST and other WPILib contributors.

---

## Team

![Team](https://img.shields.io/badge/FRC-5805-0066B3?style=for-the-badge&logo=first&logoColor=white)
![Season](https://img.shields.io/badge/Season-2026-orange?style=for-the-badge)
![Robot](https://img.shields.io/badge/Robot-Alphabot-purple?style=for-the-badge)

**FRC Team 5805 - Alphabot**
