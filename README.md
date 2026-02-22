# FRC Team 5805 - 2026 REBUILT Robot Code

<div align="center">

![FIRST Robotics](https://img.shields.io/badge/FIRST-Robotics-0066b3?style=for-the-badge&logo=first&logoColor=white)
![Java](https://img.shields.io/badge/Java-17-ED8B00?style=for-the-badge&logo=openjdk&logoColor=white)
![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-FFA500?style=for-the-badge)
![Phoenix 6](https://img.shields.io/badge/Phoenix-6.x-FF6B00?style=for-the-badge)
![AprilVision](https://img.shields.io/badge/AprilVision-3.2-4CAF50?style=for-the-badge)

**Competition-Ready Code for FRC 2026 REBUILT™ presented by Haas**

**Vision Powered by AprilVision 3.2**

[Game Manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf) • [WPILib Docs](https://docs.wpilib.org/) • [Phoenix Docs](https://v6.docs.ctr-electronics.com/)

</div>

---

## 📋 Table of Contents

- [About the Game](#-about-the-game)
- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Technologies & Libraries](#-technologies--libraries)
- [Hardware Configuration](#-hardware-configuration)
- [Software Components](#-software-components)
- [Configuration](#-configuration)
- [Deployment](#-deployment)
- [Usage](#-usage)
- [Dashboard Layout](#-dashboard-layout)
- [Power Management](#-power-management)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎮 About the Game

**REBUILT™** is the 2026 FIRST Robotics Competition game where alliances score **FUEL** into their **HUB**, navigate obstacles (BUMP/TRENCH), and climb the **TOWER** for points and ranking bonuses.

### Game Objectives
- **Score FUEL** into the HUB (1 point each)
- **Climb the TOWER** (3 levels: 10/20/30 points)
- **Earn Ranking Points**:
  - **Energized**: ≥100 FUEL scored
  - **Supercharged**: ≥360 FUEL scored
  - **Traversal**: ≥50 TOWER points

### Field Elements
| Element | Description | Dimensions |
|---------|-------------|------------|
| HUB | Scoring structure | 47" × 47" |
| TOWER | Climbing structure | 3 rungs (27", 45", 63" from carpet) |
| BUMP | Drive-over obstacle | 6.5" tall × 73" wide |
| TRENCH | Drive-under obstacle | 40.25" tall × 65.65" wide |

---

## ✨ Features

### 🎯 Vision-Guided Shooting
- **AprilTag Detection** with AprilVision 3.2
- **Distance-Based RPM Calculation** via JSON lookup table
- **Linear Interpolation** for smooth RPM curves
- **Automatic Alignment** with controller rumble feedback
- **Multi-Camera Support** (up to 4 cameras)
- **Failsafe Operation** (works with any camera count 1-4)

### 🔋 Intelligent Power Management
- **Multi-Tier Throttling** (NOMINAL → WARNING → CRITICAL → EMERGENCY)
- **Partial Vision Throttle** (single camera mode at low battery)
- **Optimistic Battery Life Prediction** with exponential smoothing
- **Priority System**:
  - Never throttled: Swerve, Shooter, Feeder
  - Emergency only: Vision (cut), Intake (75%)

### ⚡ 4-Motor Shooter System
- **Kraken X60 Motors** (4x follower configuration)
- **Phoenix 6 Velocity Control** with FOC
- **Dynamic RPM from Distance** using interpolated JSON config
- **Idle Mode** (500 RPM) for fast spin-up
- **Per-Motor Telemetry** (RPM, current, temp)
- **Reverse Detection** with visual warnings

### 📊 Advanced Telemetry
- **Auto-Discovery** of CAN motors
- **NetworkTables Integration** for remote monitoring
- **Match Replay Data** (controller inputs, timestamps)
- **Event Logging** with session statistics
- **Odometry** with Pigeon 2 IMU
- **Performance Metrics** (loop time, CAN utilization)

### 🎛️ Operator Interface
- **PS5 Controller** support
- **Multiple Control Modes** (manual, auto-shoot)
- **Speed Profiles** (Line Drive / Lob Shot)
- **Shot Counter** with telemetry
- **Shuffleboard Dashboards** (Overview, Shooter, Vision, Power, AutoShoot)

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         RobotContainer                          │
│  (Dependency Injection & Subsystem Wiring)                      │
└───────────┬─────────────────────────────────────────────────────┘
            │
            ├─► PowerManagementSubsystem
            │   ├─ Battery monitoring (voltage, slope, life estimate)
            │   ├─ State machine (NOMINAL/WARNING/CRITICAL/EMERGENCY)
            │   └─ Throttle decisions for subsystems
            │
            ├─► VisionSubsystem
            │   ├─ AprilVision 3.2 (4 cameras, AprilTag detection)
            │   ├─ Distance calculation (3D transforms)
            │   ├─ Power-aware throttling (full/partial/off)
            │   └─ Tunable camera offsets
            │
            ├─► MotorGroup2Subsystem (Shooter)
            │   ├─ 4x Kraken X60 (follower mode)
            │   ├─ ShooterConfig (JSON distance→RPM lookup)
            │   ├─ Linear interpolation for smooth curves
            │   └─ Idle mode for fast spin-up
            │
            ├─► MotorGroup1Subsystem (Feeder)
            │   └─ 1x Kraken X60 (game piece feeding)
            │
            ├─► IntakeSubsystem
            │   ├─ 1x TalonFX (roller motor)
            │   └─ Power scaling at EMERGENCY
            │
            ├─► IntakeDeploySubsystem
            │   └─ 1x TalonFX (arm deploy/retract)
            │
            └─► TelemetrySubsystem
                ├─ Auto-discovery of motors
                ├─ NetworkTables publishers
                ├─ Match replay data
                └─ Power/battery telemetry
```

---

## 🔧 Technologies & Libraries

### Core Framework
| Technology | Version | Purpose |
|------------|---------|---------|
| **WPILib** | 2026.2.1 | FRC robot framework (command-based) |
| **Java** | 17 | Programming language |
| **Gradle** | 8.x | Build system & dependency management |
| **GradleRIO** | 2026.2.1 | Robot deployment plugin |

### Hardware Libraries
| Library | Version | Usage |
|---------|---------|-------|
| **Phoenix 6** | Latest | CTRE motor controllers (Kraken X60, TalonFX) |
| **AprilVision** | 3.2 | Computer vision for AprilTag detection |
| **REVLib** | Latest | REV hardware support (if applicable) |
| **NavX** / **Pigeon 2** | Latest | IMU for odometry |

### Data & Networking
| Technology | Purpose |
|------------|---------|
| **NetworkTables** | Real-time robot ↔ dashboard communication |
| **Jackson JSON** | JSON parsing for shooter config |
| **SmartDashboard** | Live tuning & monitoring |
| **Shuffleboard** | Advanced dashboard layouts |

### Development Tools
| Tool | Purpose |
|------|---------|
| **Git** | Version control |
| **VS Code** | IDE with WPILib extension |
| **Driver Station** | Robot control & monitoring |
| **PathPlanner** | Autonomous path planning (if used) |

---

## 🤖 Hardware Configuration

### Motor Controllers (CAN Bus)

#### Shooter System
```
MotorGroup2 (Shooter) - 4x Kraken X60
├─ Motor 1 (Leader): CAN ID configurable via Constants.java
├─ Motor 2 (Follower): Aligned with Motor 1
├─ Motor 3 (Follower): Aligned with Motor 1
└─ Motor 4 (Follower): Aligned with Motor 1
```

#### Game Piece Handling
```
MotorGroup1 (Feeder): 1x Kraken X60
Intake Roller: 1x TalonFX
Intake Deploy: 1x TalonFX
```

### Vision System
- **4x Cameras** (AprilVision 3.2)
  - Camera 1: Primary (always active, even in partial throttle)
  - Cameras 2-4: Additional coverage (disabled during partial throttle)
- **AprilTag Detection** for HUB targeting
- **3D Pose Estimation** for distance calculation

### Sensors
- **Pigeon 2 IMU** (optional) - Odometry & heading
- **TalonFX Integrated Sensors** - Position, velocity, temperature

---

## 💻 Software Components

### Subsystems

<details>
<summary><b>PowerManagementSubsystem</b> - Battery & Power Control</summary>

**Features:**
- Real-time voltage monitoring (50Hz)
- 3-tier discharge rate tracking (short-term, long-term, smoothed)
- Optimistic battery life prediction
- State machine: NOMINAL → WARNING → CRITICAL → EMERGENCY
- Partial vision throttle (10.5-10.75V)
- Full vision throttle (<10.5V)
- NetworkTables publishing for telemetry

**Key Methods:**
- `shouldThrottleVision()` - Full throttle check
- `shouldPartiallyThrottleVision()` - Partial throttle check
- `getEstimatedBatteryLife()` - Remaining time estimate
- `getBatteryHealthPercent()` - 0-100% health indicator

</details>

<details>
<summary><b>VisionSubsystem</b> - AprilTag Detection</summary>

**Features:**
- 4-camera support with failsafe operation
- AprilTag detection via AprilVision 3.2
- 3D distance calculation from camera transforms
- Tunable camera offsets (X, Y, Z, Roll, Pitch, Yaw)
- Power-aware throttling (full/partial/off)
- Best target selection (area + ambiguity)

**Key Methods:**
- `hasHubTags()` - HUB AprilTag detected
- `getClosestTagDistance()` - Distance to nearest tag
- `getBestYaw()` - Alignment angle
- `setPowerManagement()` - Wire to power subsystem

</details>

<details>
<summary><b>MotorGroup2Subsystem</b> - Shooter</summary>

**Features:**
- 4-motor Kraken X60 configuration
- JSON-based distance→RPM lookup
- Linear interpolation between data points
- Idle mode (500 RPM) for spin-up efficiency
- Per-motor telemetry (RPM, amps, temp)
- Reverse detection & warnings

**Key Methods:**
- `runAtDistance(meters)` - Distance-based shooting
- `getRPMForDistance(meters)` - Get calculated RPM
- `runAtRPM(rpm)` - Manual RPM control
- `getAverageRPM()` - Current shooter speed

</details>

<details>
<summary><b>ShooterConfig</b> - JSON Configuration Manager</summary>

**Features:**
- Loads `shooter_config.json` at startup
- Zero file I/O during matches (all data cached)
- Linear interpolation between data points
- Fallback defaults if JSON missing
- Safe error handling

**Key Methods:**
- `loadConfig()` - Load JSON (call once at init)
- `getRPMForDistance(meters)` - Interpolated RPM lookup
- `getMinDistance()` / `getMaxDistance()` - Range info

</details>

<details>
<summary><b>TelemetrySubsystem</b> - Data Logging</summary>

**Features:**
- Auto-discovery of CAN motors
- NetworkTables publishing (motors, vision, power, match)
- Match replay data (controller inputs, timestamps)
- Event logging with session stats
- Odometry integration (Pigeon 2)
- Power/battery telemetry

**Published Data:**
- `/Telemetry/Motors/*` - Auto-discovered motor data
- `/Telemetry/Vision/*` - Tag detection, alignment
- `/Telemetry/System/*` - Battery, CAN util, loop time, power state
- `/Telemetry/Match/*` - Phase, time, alliance
- `/Telemetry/Replay/*` - Frame sync, controller inputs

</details>

### Commands

<details>
<summary><b>VisionControlCommand</b> - Auto-Aim & Shoot</summary>

**Features:**
- Vision-guided shooting with HUB detection
- Distance-based RPM (JSON config) or area-based (fallback)
- Prespin delay (shooter spins before feeder)
- Anti-flicker (500ms hold time after tag loss)
- Controller rumble on target lock
- Shot counter with telemetry
- Speed profiles (Line Drive / Lob Shot)

**Shuffleboard Display:**
- Alignment arrows (<<--, -->, >>> LOCKED <<<)
- Target lock indicator
- Shooter RPM dial
- Distance display
- Shot counter

</details>

---

## ⚙️ Configuration

### Shooter RPM Tuning (`src/main/deploy/shooter_config.json`)

The shooter uses a **distance-to-RPM lookup table** that can be edited without recompiling:

```json
{
  "version": "1.0",
  "description": "Shooter RPM lookup table based on distance to HUB - 2026 REBUILT",
  "table": [
    {
      "distance_meters": 1.0,
      "rpm": 1200,
      "notes": "Point blank - touching HUB"
    },
    {
      "distance_meters": 2.0,
      "rpm": 1800,
      "notes": "Mid-range HUB shot"
    }
  ]
}
```

**How It Works:**
1. Vision calculates distance to HUB via AprilTag 3D transform
2. ShooterConfig loads JSON at startup (zero runtime overhead)
3. Linear interpolation between data points (e.g., 1.5m → 1500 RPM)
4. Shooter runs at calculated RPM

**To Tune:**
1. Edit `shooter_config.json` with tested values
2. Deploy to robot: `./gradlew deploy`
3. No code recompilation needed!

### Constants (`Constants.java`)

All robot parameters centralized in `Constants.java`:

<details>
<summary><b>Motor IDs & CAN Configuration</b></summary>

```java
// Shooter (4x Kraken X60)
kMotorGroup2Motor1ID = 1  // Leader
kMotorGroup2Motor2ID = 2  // Follower
kMotorGroup2Motor3ID = 3  // Follower
kMotorGroup2Motor4ID = 4  // Follower
kMotorGroup2CANBus = "canivore"

// Feeder (1x Kraken X60)
kMotorGroup1MotorID = 5

// Intake
kMotorGroup3MotorID = 6  // Roller
kMotorGroup4MotorID = 7  // Deploy
```

</details>

<details>
<summary><b>Vision Configuration</b></summary>

```java
// Camera Names (AprilVision 3.2)
kCamera1Name = "cam1"
kCamera2Name = "cam2"
kCamera3Name = "cam3"
kCamera4Name = "cam4"

// Camera Offsets (tunable via SmartDashboard)
kCam1OffsetX/Y/Z = 0.0  // meters
kCam1Roll/Pitch/Yaw = 0.0  // degrees

// Vision Thresholds
kMaxAmbiguityThreshold = 0.2
kAlignedYawThreshold = 3.0  // degrees
```

</details>

<details>
<summary><b>Power Management</b></summary>

```java
// Voltage Thresholds
kWarningVoltage = 11.5   // Start monitoring
kCriticalVoltage = 11.0  // Conserve power
kEmergencyVoltage = 10.5 // Full throttle

// Partial Vision Throttle: 10.5V - 10.75V
// Full Vision Throttle: < 10.5V

// Intake Scaling
kEmergencyModeIntakeScale = 0.75  // 75% power
```

</details>

---

## 🚀 Deployment

### Prerequisites
- **Java 17** installed
- **WPILib 2026** installed
- **Git** for version control
- **roboRIO** configured with team number

### Build & Deploy

```bash
# Build the project
./gradlew build

# Deploy to robot (requires Driver Station connection)
./gradlew deploy

# Deploy with verbose logging
./gradlew deploy --info

# Clean build (if needed)
./gradlew clean build
```

### Deploy JSON Config Only
```bash
# Edit shooter_config.json, then deploy
./gradlew deploy
```

The `src/main/deploy/` folder deploys to `/home/lvuser/deploy/` on the roboRIO.

---

## 🎮 Usage

### Driver Controls (PS5 Controller)

| Button | Action |
|--------|--------|
| **Triangle** | Auto-aim & shoot (vision-guided) |
| **Square** | Auto-deploy intake + run rollers |
| **Cross** | Run feeder (manual feed) |
| **Circle** | Eject (reverse intake) |
| **L1** | Shooter forward (manual) |
| **R1** | Shooter reverse (manual) |
| **D-Pad Up** | Deploy intake arm up |
| **D-Pad Down** | Deploy intake arm down |
| **L3** | Toggle speed profile (Line Drive ↔ Lob Shot) |
| **Options** | Reset shot counter |

### Auto-Shoot Workflow

1. **Press Triangle** - Command starts
2. **Align with HUB** - Vision detects AprilTag
3. **Shooter Spins Up** - RPM calculated from distance
4. **Controller Rumbles** - Target locked
5. **Feeder Starts** - FUEL shoots into HUB
6. **Shot Logged** - Counter increments, telemetry recorded

---

## 📊 Dashboard Layout

### Overview Tab
```
┌─────────────────────────────────────────────────────────────┐
│ Row 0: Battery V │ Power State │ Brownout! │ Avg RPM       │
│ Row 1: Safe Shoot│ Batt %      │ Intake    │ RPM HIGH│Rev │
│ Row 2: Hub│Tags│Vision│Yaw│M1│M2│M3│M4│Target RPM          │
│ Row 3: Intake RPM│Arm State│Arm Up│Arm Down│M1-M4 OK│Err   │
│ Row 4: Intake Amps        │ Shooter Amps                    │
└─────────────────────────────────────────────────────────────┘
```

### AutoShoot Tab
```
┌────────────────────────────────────────┐
│ Alignment: >>> LOCKED <<<              │
│ Status: SHOOTING                       │
│ TARGET LOCK: ✓                         │
│ Shots Fired: 12/∞                      │
│ Speed Profile: LINE DRIVE              │
│ Shooter RPM: [Dial showing 2400]      │
│ Distance (m): 2.0                      │
└────────────────────────────────────────┘
```

### Shooter Tab
- **4 RPM Dials** (M1-M4) with color indicators
- **Per-motor stats** (amps, temp)
- **Config status** (loaded, version, entries, range)
- **Reverse warning** (big red indicator)

### Power Tab
- **Battery voltage** (dial + trends)
- **Discharge rate** (V/s smoothed)
- **Battery life estimate** (seconds remaining)
- **Power state** (NOMINAL/WARNING/CRITICAL/EMERGENCY)
- **Session statistics** (min voltage, drop, peak amps)

---

## ⚡ Power Management

### Throttle States

| State | Voltage | Actions |
|-------|---------|---------|
| **NOMINAL** | >11.5V | All systems full power |
| **WARNING** | 11.0-11.5V | Monitoring (no throttle) |
| **CRITICAL (Upper)** | 10.75-11.0V | Monitoring |
| **CRITICAL (Lower)** | 10.5-10.75V | **Partial vision throttle** (1 camera only) |
| **EMERGENCY** | <10.5V | **Full vision cut**, intake 75% power |

### Priority System

**Never Throttled:**
- Swerve drive (mobility)
- Shooter (scoring)
- Feeder (scoring)

**Emergency Throttle:**
- Vision: Completely disabled (saves CPU + bandwidth)
- Intake: 75% power

### Battery Life Prediction

Uses **3-tier slope tracking** for accurate estimates:
1. **Short-term** (last 10 samples, ~0.2s) - Reactive
2. **Long-term** (session average) - Stable
3. **Smoothed** (80% old + 20% new) - Optimistic

Result: Stable battery life estimates that don't panic during brief current spikes.

---

## 🤝 Contributing

### Code Style
- **Command-based architecture** (WPILib 2026)
- **Subsystems** own hardware, **Commands** implement behaviors
- **Constants.java** for all configuration
- **Javadoc comments** for public APIs
- **Console logging** with subsystem prefixes `[SUBSYSTEM]`

### Testing
```bash
# Run unit tests (if available)
./gradlew test

# Simulate on desktop
./gradlew simulateJava
```

### Pull Request Process
1. Create feature branch: `git checkout -b feature/amazing-feature`
2. Make changes & test thoroughly
3. Commit: `git commit -m "Add amazing feature"`
4. Push: `git push origin feature/amazing-feature`
5. Open Pull Request with description

---

## 📄 License

This project is licensed under the **WPILib BSD License**.

Copyright (c) FIRST and other WPILib contributors. Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

---

## 🏆 Acknowledgments

- **FIRST Robotics Competition** - For the amazing game and platform
- **WPILib Team** - For the incredible framework
- **CTRE Electronics** - For Phoenix 6 and Kraken motors
- **AprilVision Team** - For AprilVision 3.2 vision processing software
- **Team 5805 Mentors & Students** - For building this awesome robot!

---

<div align="center">

**Good luck at competition! 🤖🏆**

![Team 5805](https://img.shields.io/badge/FRC-5805-blue?style=for-the-badge)

</div>
#   L e v i t i c u s - 5 8 0 5 - 2 0 2 6 - c o d e  
 