# FRC Team 5805 - Alphabot 2026 Robot Code

Competition-ready code for the 2026 FIRST Robotics Competition season.

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

## Overview

This repository contains the robot code for FRC Team 5805's 2026 competition robot. The codebase is built on WPILib's command-based framework with Phoenix 6 motor control and AprilVision 3.2 for vision processing.

### Technologies Used

- **WPILib 2026** - FRC robot framework
- **Java 17** - Programming language
- **Phoenix 6** - CTRE motor controllers (Kraken X60, Kraken X44)
- **AprilVision 3.2** - AprilTag detection and tracking
- **Shuffleboard** - Custom dashboard interface

## Features

### Vision-Guided Shooting
- AprilTag detection with multi-camera support (up to 4 cameras)
- Distance-based RPM calculation from JSON lookup tables
- Linear interpolation for smooth RPM curves between data points
- Automatic alignment with controller rumble feedback
- Failsafe operation with any number of cameras

### Adjustable Hood System
- Kraken X44 motor with CANCoder for precise position control
- Motion Magic profiling for smooth, jitter-free movement
- Anti-jitter features: deadband, low-pass filtering, settling counter
- Hardstop calibration for automatic range detection
- Vision-based automatic angle adjustment based on target distance

### Power Management
- Real-time battery monitoring with discharge rate tracking
- Multi-tier throttling system (NOMINAL, WARNING, CRITICAL, EMERGENCY)
- Partial vision throttle mode to conserve power
- Priority-based subsystem protection (drive and shooter are never throttled)

### 4-Motor Shooter
- Four Kraken X60 motors in follower configuration
- Phoenix 6 velocity control with FOC
- JSON-configurable distance-to-RPM lookup table
- Idle mode at 500 RPM for faster spin-up
- Per-motor telemetry (RPM, current, temperature)

### Custom Shuffleboard Dashboard
- Overview tab with all critical stats on one page
- Dedicated tabs for Shooter, Intake, Power, and Vision details
- Real-time updates at 50Hz
- Custom widget factory for consistent styling

## Hardware Configuration

### Motor Controllers

| Subsystem | Motor Type | Quantity | CAN IDs |
|-----------|------------|----------|---------|
| Shooter | Kraken X60 | 4 | 1-4 |
| Feeder | Kraken X60 | 2 | 5-6 |
| Intake Roller | TalonFX | 1 | 10 |
| Intake Deploy | TalonFX | 1 | 11 |
| Hood | Kraken X44 | 1 | 17 |

### Sensors

| Sensor | Type | CAN ID |
|--------|------|--------|
| Hood Encoder | CANCoder | 18 |
| IMU | Pigeon 2 | 20 |

### Vision System
- Up to 4 cameras supported via AprilVision 3.2
- Camera names configurable in Constants.java
- 3D pose estimation for distance calculation

## Software Architecture

```
RobotContainer
├── PowerManagementSubsystem    - Battery monitoring and throttle control
├── VisionSubsystem             - AprilTag detection (4 cameras)
├── MotorGroup2Subsystem        - Shooter (4x Kraken X60)
├── MotorGroup1Subsystem        - Feeder (2x Kraken X60)
├── HoodSubsystem               - Adjustable hood angle
├── IntakeSubsystem             - Intake rollers
├── IntakeDeploySubsystem       - Intake arm deployment
├── TelemetrySubsystem          - Data logging and NetworkTables
└── RobotDashboard              - Custom Shuffleboard interface
```

### Key Classes

- **ShooterConfig** - Loads distance-to-RPM lookup table from JSON
- **VisionControlCommand** - Auto-aim and shoot command with vision tracking
- **HoodSubsystem** - Position-controlled hood with calibration and anti-jitter

## Getting Started

### Prerequisites
- Java 17
- WPILib 2026
- Git

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
- Motor CAN IDs and bus configuration
- Camera names and offsets
- Power management voltage thresholds
- Hood motion profile settings

## Controls

### PS5 Controller Mapping

| Button | Action |
|--------|--------|
| Triangle | Auto-aim and shoot (vision-guided) |
| Square | Auto-deploy intake and run rollers |
| Cross | Run feeder manually |
| Circle | Eject (reverse intake) |
| L1 | Shooter forward (manual) |
| R1 | Shooter reverse (manual) |
| D-Pad Up | Deploy intake arm up |
| D-Pad Down | Deploy intake arm down |
| L3 | Toggle speed profile (Line Drive / Lob Shot) |
| Options | Reset shot counter |

## Dashboard

### Overview Tab
All critical robot stats on one page:
- Battery voltage and power state
- Intake status (running, RPM, amps, arm position)
- Shooter status (average RPM, target RPM, motor health)
- Hood status (angle, calibrated, at target, auto mode)
- Vision status (tag count, hub detected, camera connections)

### Additional Tabs
- **Shooter** - Per-motor RPM, amps, temperature, and config status
- **Intake** - Roller and deploy arm details with graphs
- **Power** - Battery health, discharge rate, and estimated life
- **Vision** - Camera connections, tag counts, and best target info

## Contributing

1. Create a feature branch from main
2. Make changes and test thoroughly
3. Commit with descriptive messages
4. Push and open a pull request

### Code Style
- Follow WPILib command-based patterns
- Use Javadoc comments for public APIs
- Prefix console output with subsystem name: `[SUBSYSTEM] message`
- Keep Constants.java as the single source for configuration

## License

This project is licensed under the WPILib BSD License. See LICENSE file for details.

Copyright (c) FIRST and other WPILib contributors.
