<![CDATA[<div align="center">

```
    __    _______    ______________  ________  _______
   / /   / ____/ |  / /  _/_  __/ / / / ____/ / ____/ /
  / /   / __/  | | / // /  / / / / / /\__ \  /___ \/ /
 / /___/ /___ | |/ // /  / / / /_/ /___/ / ____/ /_/
/_____/_____/ |___/___/ /_/  \____//____(_)____(_)

         F R C   T E A M   5 8 0 5   -   2 0 2 6
```

# Leviticus - FRC Team 5805 Robot Code

![Java](https://img.shields.io/badge/Java-17-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026-00629B?style=for-the-badge&logo=first&logoColor=white)
![Phoenix 6](https://img.shields.io/badge/Phoenix_6-25.0-FF6B00?style=for-the-badge)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen?style=for-the-badge)
![License](https://img.shields.io/badge/License-BSD-blue?style=for-the-badge)

**Competition-ready code for the 2026 FIRST Robotics Competition season**

[Features](#features) | [Quick Start](#quick-start) | [Architecture](#software-architecture) | [Controls](#controls) | [Dashboard](#dashboard)

</div>

---

## Quick Stats

<table>
<tr>
<td width="25%" align="center">

**8 Subsystems**

Power, Vision, Shooter, Feeder, Hood, Intake, Deploy, Telemetry

</td>
<td width="25%" align="center">

**9 Motors**

4x Shooter, 2x Feeder, 1x Intake, 1x Deploy, 1x Hood

</td>
<td width="25%" align="center">

**4 Cameras**

Multi-camera AprilTag vision with Kalman filtering

</td>
<td width="25%" align="center">

**5 Dashboard Tabs**

Real-time monitoring at 50Hz

</td>
</tr>
</table>

---

## Table of Contents

- [Features](#features)
- [Quick Start](#quick-start)
- [Hardware Configuration](#hardware-configuration)
- [Software Architecture](#software-architecture)
- [Configuration](#configuration)
- [Controls](#controls)
- [Dashboard](#dashboard)
- [Performance Metrics](#performance-metrics)
- [Development Workflow](#development-workflow)
- [Match Day Checklist](#match-day-checklist)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [Acknowledgments](#acknowledgments)

---

## Features

<details>
<summary><b>Vision-Guided Shooting</b> - Click to expand</summary>

![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=flat-square)
![Cameras](https://img.shields.io/badge/Cameras-4-blue?style=flat-square)
![Filter](https://img.shields.io/badge/Filter-Kalman-purple?style=flat-square)

- AprilTag detection with multi-camera support (up to 4 cameras)
- Distance-based RPM calculation from JSON lookup tables
- Linear interpolation for smooth RPM curves between data points
- Automatic alignment with controller rumble feedback
- Failsafe operation with any number of cameras
- Kalman filtering for stable distance readings

```java
// Example: Getting shooter settings for detected distance
double distance = vision.getClosestDistance();
double rpm = shooter.getRPMForDistance(distance);
double hoodAngle = shooter.getHoodAngleForDistance(distance);
```

</details>

<details>
<summary><b>Adjustable Hood System</b> - Click to expand</summary>

![Motor](https://img.shields.io/badge/Motor-Kraken_X44-FF6B00?style=flat-square)
![Encoder](https://img.shields.io/badge/Encoder-CANCoder-FF6B00?style=flat-square)
![Control](https://img.shields.io/badge/Control-Motion_Magic-9C27B0?style=flat-square)

- Kraken X44 motor with CANCoder for precise position control
- Motion Magic profiling for smooth, jitter-free movement
- Anti-jitter features: deadband, low-pass filtering, settling counter
- Hardstop calibration for automatic range detection
- Vision-based automatic angle adjustment based on target distance
- Angle range: 15 to 45 degrees

```
Hood Angle vs Distance Curve:

    45° |                                    *****
        |                              *****
    35° |                       *****
        |                *****
    25° |         *****
        |   *****
    15° |***
        +----------------------------------------
          1m      2m      3m      4m      Distance
```

</details>

<details>
<summary><b>Power Management</b> - Click to expand</summary>

![Protection](https://img.shields.io/badge/Protection-Multi_Tier-yellow?style=flat-square)
![States](https://img.shields.io/badge/States-4-orange?style=flat-square)
![Monitoring](https://img.shields.io/badge/Monitoring-Real_Time-brightgreen?style=flat-square)

- Real-time battery monitoring with discharge rate tracking
- Multi-tier throttling system (NOMINAL, WARNING, CRITICAL, EMERGENCY)
- Partial vision throttle mode to conserve power
- Priority-based subsystem protection (drive and shooter are never throttled)
- Optimistic battery life prediction with exponential smoothing

```
Power State Diagram:

  NOMINAL ──[<11.5V]──> WARNING ──[<11.0V]──> CRITICAL ──[<10.5V]──> EMERGENCY
     ^                     |                     |                      |
     |                     |                     |                      |
     └─────[>12.0V]────────┴─────[>11.5V]────────┴──────[>11.0V]────────┘
```

</details>

<details>
<summary><b>4-Motor Shooter</b> - Click to expand</summary>

![Motors](https://img.shields.io/badge/Motors-4x_Kraken_X60-FF6B00?style=flat-square)
![Control](https://img.shields.io/badge/Control-FOC-9C27B0?style=flat-square)
![Idle](https://img.shields.io/badge/Idle_RPM-500-blue?style=flat-square)

- Four Kraken X60 motors in follower configuration
- Phoenix 6 velocity control with FOC (Field Oriented Control)
- JSON-configurable distance-to-RPM lookup table
- Idle mode at 500 RPM for faster spin-up
- Per-motor telemetry (RPM, current, temperature)

```
Shooter Motor Configuration:

    ┌─────────┐     ┌─────────┐
    │ Motor 1 │────>│ Motor 2 │  (Top pair - Leader/Follower)
    │ (Lead)  │     │ (Follow)│
    └─────────┘     └─────────┘
         │               │
    ┌─────────┐     ┌─────────┐
    │ Motor 3 │────>│ Motor 4 │  (Bottom pair - Leader/Follower)
    │ (Lead)  │     │ (Follow)│
    └─────────┘     └─────────┘
```

</details>

<details>
<summary><b>Custom Shuffleboard Dashboard</b> - Click to expand</summary>

![Tabs](https://img.shields.io/badge/Tabs-5-blue?style=flat-square)
![Update](https://img.shields.io/badge/Update_Rate-50Hz-brightgreen?style=flat-square)
![Widgets](https://img.shields.io/badge/Widgets-Custom-purple?style=flat-square)

- Overview tab with all critical stats on one page
- Real-time graphs for RPM, voltage, and current
- Dedicated tabs for Shooter, Intake, Power, and Vision details
- Real-time updates at 50Hz
- Custom widget factory for consistent styling

</details>

---

## Quick Start

### Prerequisites

| Requirement | Version | Download |
|-------------|---------|----------|
| Java JDK | 17+ | [Adoptium](https://adoptium.net/) |
| WPILib | 2026 | [WPILib Releases](https://github.com/wpilibsuite/allwpilib/releases) |
| Git | Latest | [Git Downloads](https://git-scm.com/downloads) |
| VS Code | Latest | Included with WPILib |

### Installation

```bash
# Clone the repository
git clone https://github.com/TomAs-1226/Leviticus-5805-2026-code.git
cd Leviticus-5805-2026-code

# Build the project
./gradlew build

# Deploy to robot (connected via USB or radio)
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

### First-Time Setup

1. Open the project in VS Code with WPILib extension
2. Press `Ctrl+Shift+P` and select "WPILib: Set Team Number" -> Enter `5805`
3. Connect to the robot via USB or WiFi
4. Run `./gradlew deploy` to upload code

---

## Hardware Configuration

### Motor Controllers

| Subsystem | Motor | Qty | CAN IDs | Current Limit |
|-----------|-------|-----|---------|---------------|
| Shooter | Kraken X60 | 4 | 1-4 | 80A |
| Feeder | Kraken X60 | 2 | 5-6 | 60A |
| Intake Roller | TalonFX | 1 | 10 | 40A |
| Intake Deploy | TalonFX | 1 | 11 | 40A |
| Hood | Kraken X44 | 1 | 17 | 30A |

### Sensors

| Sensor | Type | CAN ID | Purpose |
|--------|------|--------|---------|
| Hood Encoder | CANCoder | 18 | Absolute position feedback |
| IMU | Pigeon 2 | 20 | Heading and orientation |

### CAN Bus Layout

```
RoboRIO
    │
    ├── CAN Bus (Main)
    │   ├── [1-4]  Shooter Motors (Kraken X60)
    │   ├── [5-6]  Feeder Motors (Kraken X60)
    │   ├── [10]   Intake Roller (TalonFX)
    │   ├── [11]   Intake Deploy (TalonFX)
    │   ├── [17]   Hood Motor (Kraken X44)
    │   ├── [18]   Hood Encoder (CANCoder)
    │   └── [20]   Pigeon 2 IMU
    │
    └── USB Cameras
        ├── cam1 (Primary - Required)
        ├── cam2 (Secondary)
        ├── cam3 (Tertiary)
        └── cam4 (Quaternary)
```

---

## Software Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                           RobotContainer                             │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                      Command Scheduler                        │   │
│  └──────────────────────────────────────────────────────────────┘   │
│         │           │           │           │           │            │
│    ┌────▼────┐ ┌────▼────┐ ┌────▼────┐ ┌────▼────┐ ┌────▼────┐     │
│    │ Vision  │ │ Shooter │ │ Feeder  │ │  Hood   │ │ Intake  │     │
│    │  Sub    │ │   Sub   │ │   Sub   │ │   Sub   │ │   Sub   │     │
│    └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘     │
│         │           │           │           │           │            │
│    ┌────▼───────────▼───────────▼───────────▼───────────▼────┐     │
│    │                  PowerManagementSubsystem                │     │
│    │              (Battery Monitoring & Throttling)           │     │
│    └────────────────────────────┬────────────────────────────┘     │
│                                 │                                    │
│    ┌────────────────────────────▼────────────────────────────┐     │
│    │                     RobotDashboard                       │     │
│    │                    (Shuffleboard UI)                     │     │
│    └──────────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────┘
```

### Data Flow

```
AprilTag Detection Pipeline:

  Cameras ──> AprilVision ──> VisionSubsystem ──> Kalman Filter ──> Distance
      │                              │                                  │
      │                              │                                  │
      └──────────────────────────────┼──────────────────────────────────┘
                                     │
                                     ▼
                              ShooterConfig.json
                                     │
                          ┌──────────┴──────────┐
                          │                     │
                          ▼                     ▼
                     Target RPM            Hood Angle
                          │                     │
                          ▼                     ▼
                    Shooter Motors        Hood Motor
```

### Key Classes

| Class | Responsibility |
|-------|----------------|
| `RobotContainer` | Subsystem instantiation and button bindings |
| `VisionControlCommand` | Auto-aim and shoot with vision tracking |
| `HoodSubsystem` | Position-controlled hood with anti-jitter |
| `ShooterConfig` | JSON-based distance-to-RPM lookup table |
| `PowerManagementSubsystem` | Battery monitoring and load shedding |
| `RobotDashboard` | Custom Shuffleboard UI with 5 tabs |

---

## Configuration

### Shooter Tuning

Edit `src/main/deploy/shooter_config.json`:

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

> **Note:** Changes take effect on next deploy without recompiling code.

### Constants Reference

All tunable parameters are centralized in `Constants.java`:

| Category | Key Constants |
|----------|---------------|
| Motor IDs | `kShooterMotor1ID` through `kHoodMotorID` |
| PID Gains | `kShooterKp`, `kShooterKi`, `kShooterKd`, etc. |
| Vision | `kCamera1Name`, `kAlignmentThreshold`, `kDistanceOffset` |
| Power | `kWarningVoltage`, `kCriticalVoltage`, `kEmergencyVoltage` |
| Hood | `kHoodMinAngle`, `kHoodMaxAngle`, `kHoodTolerance` |

---

## Controls

### PS5 Controller Layout

```
                    ┌─────────────────────────────┐
                    │        PS5 Controller        │
                    └─────────────────────────────┘

         L1: Shooter FWD          R1: Shooter REV
         L2: [Available]          R2: [Available]
         L3: Toggle Profile       R3: [Available]

              D-Pad                      Buttons
           ┌───┬───┐                   ┌───┐
           │ ↑ │   │ Arm Up      (△)  │   │ Auto-Aim & Shoot
           ├───┼───┤                   ├───┤
           │   │ → │             (○)  │   │ Eject (Reverse)
           ├───┼───┤                   ├───┤
           │ ↓ │   │ Arm Down    (×)  │   │ Run Feeder
           └───┴───┘                   ├───┤
                                  (□)  │   │ Auto-Deploy Intake
                                       └───┘

         Options: Reset Shot Counter
         Share: [Available]
```

### Button Reference

| Button | Action | Type |
|--------|--------|------|
| Triangle | Auto-aim and shoot | Vision Control |
| Square | Auto-deploy intake + run rollers | Automatic |
| Cross | Run feeder manually | Manual |
| Circle | Eject (reverse intake) | Manual |
| L1 | Shooter forward | Manual |
| R1 | Shooter reverse | Manual |
| D-Pad Up/Down | Manual arm control | Manual |
| L3 | Toggle shot profile (Lob/Line) | Config |
| Options | Reset shot counter | Config |

---

## Dashboard

### Tab Overview

| Tab | Purpose | Key Information |
|-----|---------|-----------------|
| **Overview** | At-a-glance status | Battery, RPM, all systems |
| **Shooter** | Motor details | 4x RPM, current, temperature |
| **Intake** | Roller/arm status | RPM, current, arm position |
| **Power** | Battery health | Voltage trends, life estimate |
| **Vision** | Camera status | Tag count, distance, alignment |

### Overview Tab Layout

```
┌─────────────────┬─────────────────┬─────────────────┬─────────────────┐
│     POWER       │     INTAKE      │    SHOOTER      │     VISION      │
├─────────────────┼─────────────────┼─────────────────┼─────────────────┤
│ Battery: 12.5V  │ [■] Running     │ RPM: 3500       │ Tags: 2         │
│ State: NOMINAL  │ RPM: ████░ 80%  │ Target: 3500    │ [■] HUB Seen    │
│ [■] Safe        │ Amps: ██░░ 25A  │ Amps: ████ 120A │ CAM1: [■] CAM2: │
│ Life: 8:32      │ Arm: DEPLOYED   │ [■■■■] All OK   │ Dist: 2.45m     │
├─────────────────┴────────┬────────┴─────────────────┴─────────────────┤
│          HOOD            │              GRAPHS                         │
│ Angle: 28.5°  Cal: [■]   │  RPM ▁▂▃▄▅▆▇█▇▆▅▄▃▂▁                       │
│ Target: 28°   Auto: [■]  │  Voltage ▇▇▇▇▇▆▆▆▆▅▅▅                      │
└──────────────────────────┴────────────────────────────────────────────┘
```

---

## Performance Metrics

### Shooter Performance

| Metric | Target | Actual |
|--------|--------|--------|
| Spin-up time (0 to 4000 RPM) | < 1.5s | ~1.2s |
| RPM stability (at target) | ± 50 RPM | ± 30 RPM |
| Shot consistency | > 95% | TBD |

### Vision Performance

| Metric | Target | Actual |
|--------|--------|--------|
| Tag detection latency | < 50ms | ~35ms |
| Distance accuracy | ± 5cm | ± 3cm |
| Multi-tag fusion | 4 cameras | 4 cameras |

### Power Performance

| Metric | Target | Actual |
|--------|--------|--------|
| Match duration (full load) | 2:30 | ~2:45 |
| Brownout protection | < 10.5V | Enabled |
| Recovery time | < 2s | ~1.5s |

---

## Development Workflow

### Branch Strategy

```
main (protected)
  │
  ├── feature/vision-improvements
  ├── feature/hood-tuning
  ├── bugfix/shooter-spinup
  └── release/v1.0.0
```

### Code Review Checklist

- [ ] Code compiles without errors
- [ ] No new warnings introduced
- [ ] Constants are documented
- [ ] Logging uses `[SUBSYSTEM]` prefix format
- [ ] Safety checks are in place for motor commands
- [ ] Dashboard updates don't block main loop

### Testing Commands

```bash
# Build and check for errors
./gradlew build

# Run unit tests
./gradlew test

# Generate documentation
./gradlew javadoc

# Clean build artifacts
./gradlew clean
```

---

## Match Day Checklist

### Pre-Match (30 min before)

- [ ] Battery voltage > 12.8V
- [ ] All CAN devices responding (check Phoenix Tuner)
- [ ] Cameras connected and streaming
- [ ] Hood calibration complete
- [ ] Driver station connected

### Pit Checks (between matches)

- [ ] Inspect motor temperatures (< 60°C)
- [ ] Check belt tension on shooter
- [ ] Verify CANCoder readings
- [ ] Swap battery if < 12.5V
- [ ] Clear any logged errors

### Post-Match

- [ ] Download match logs
- [ ] Note any issues encountered
- [ ] Charge batteries immediately
- [ ] Inspect for damage

---

## Troubleshooting

<details>
<summary><b>Vision not detecting tags</b></summary>

1. Check camera connections in Driver Station
2. Verify camera names in `Constants.java` match AprilVision config
3. Ensure adequate lighting on field
4. Check AprilVision pipeline is running

</details>

<details>
<summary><b>Shooter not reaching target RPM</b></summary>

1. Check battery voltage (> 12V required)
2. Verify motor temperatures (< 80°C)
3. Inspect belt tension and wear
4. Check for mechanical binding
5. Review PID gains in Constants

</details>

<details>
<summary><b>Hood jittering or unstable</b></summary>

1. Verify CANCoder connection
2. Run hardstop calibration
3. Check anti-jitter settings:
   - `kHoodUpdateDeadband`
   - `kHoodFilterAlpha`
   - `kHoodSettlingCycles`

</details>

<details>
<summary><b>CAN bus errors</b></summary>

1. Check termination resistors
2. Verify wiring integrity
3. Reduce CAN bus length if possible
4. Check for duplicate CAN IDs

</details>

<details>
<summary><b>Brownout during match</b></summary>

1. Start with fully charged battery (> 12.8V)
2. Limit simultaneous high-current operations
3. Check for mechanical binding causing stall
4. Review power throttling settings

</details>

---

## Contributing

### Getting Started

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Make your changes
4. Test thoroughly in simulation and on robot
5. Submit a pull request

### Code Style

| Rule | Description |
|------|-------------|
| Architecture | WPILib command-based patterns |
| Documentation | Javadoc for all public methods |
| Logging | Use `[SUBSYSTEM]` prefix: `System.out.println("[SHOOTER] ...")` |
| Constants | Keep in `Constants.java`, no magic numbers |
| Naming | camelCase for variables, PascalCase for classes |

---

## Acknowledgments

### Team 5805

A huge thank you to all team members, mentors, and sponsors who make this possible.

### Open Source Libraries

- [WPILib](https://github.com/wpilibsuite/allwpilib) - FRC robot framework
- [Phoenix 6](https://github.com/CrossTheRoadElec/Phoenix-Releases) - CTRE motor control
- [Jackson](https://github.com/FasterXML/jackson) - JSON parsing

### Inspiration

We learned from many other FRC teams' open-source code. Thank you to the FRC community for sharing knowledge.

---

<div align="center">

## License

This project is licensed under the WPILib BSD License.

Copyright (c) FIRST and other WPILib contributors.

---

![Team](https://img.shields.io/badge/FRC-5805-0066B3?style=for-the-badge&logo=first&logoColor=white)
![Season](https://img.shields.io/badge/Season-2026-orange?style=for-the-badge)
![Robot](https://img.shields.io/badge/Robot-Leviticus-purple?style=for-the-badge)

**FRC Team 5805 - Leviticus**

*"Excellence through engineering"*

</div>
