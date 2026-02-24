# Telemetry API Documentation

Quick reference for all the data the robot publishes.

**Author:** Baichen Yu
**Team:** 5805 Alphabot
**Last Updated:** 2026

---

## Connecting

```python
from networktables import NetworkTables

NetworkTables.initialize(server='10.58.05.2')  # Or 'roboRIO-5805-FRC.local'
```

---

## SmartDashboard Data

All this stuff shows up on SmartDashboard automatically.

### Shooter (4 motors)

| Key | Type | What it is |
|-----|------|------------|
| `Shooter` | string | Status: "IDLE", "STOPPED", etc |
| `Shooter Avg RPM` | double | Average across all 4 motors |
| `Shooter Target RPM` | double | What we're trying to hit |
| `Shooter RPM Error` | double | How far off we are |
| `Shooter Total Amps` | double | Combined current draw |
| `Shooter Peak RPM` | double | Highest RPM this session |
| `Shooter Peak Amps` | double | Highest current this session |
| `Shooter Spinup Count` | int | How many times we've spun up |
| `Shooter Idle Active` | bool | Is idle mode on? |
| `Shooter All Alive` | bool | All 4 motors responding? |
| `Shooter M1 RPM` | double | Motor 1 speed |
| `Shooter M2 RPM` | double | Motor 2 speed |
| `Shooter M3 RPM` | double | Motor 3 speed |
| `Shooter M4 RPM` | double | Motor 4 speed |
| `Shooter M1 Amps` | double | Motor 1 current |
| `Shooter M2 Amps` | double | Motor 2 current |
| `Shooter M3 Amps` | double | Motor 3 current |
| `Shooter M4 Amps` | double | Motor 4 current |
| `Shooter M1 Temp` | double | Motor 1 temp (°C) |
| `Shooter M2 Temp` | double | Motor 2 temp (°C) |
| `Shooter M3 Temp` | double | Motor 3 temp (°C) |
| `Shooter M4 Temp` | double | Motor 4 temp (°C) |

### Intake Rollers

| Key | Type | What it is |
|-----|------|------------|
| `Intake` | string | Status: "SUCKING", "SPITTING", "STOPPED" |
| `Intake RPM` | double | Roller speed |
| `Intake Amps` | double | Current draw |
| `Intake Temp C` | double | Motor temperature |
| `Intake Voltage` | double | Applied voltage |
| `Intake Power %` | double | Current power level |
| `Intake Peak Amps` | double | Highest current this session |
| `Intake Total Rotations` | double | Total rotations run |
| `Intake Running` | bool | Is it spinning? |
| `Intake Alive` | bool | Motor responding? |
| `Intake Uptime` | double | Seconds since startup |

### Intake Deploy Arm

| Key | Type | What it is |
|-----|------|------------|
| `Deploy` | string | Status: "GOING DOWN", "GOING UP" |
| `Deploy State` | string | "UP", "DOWN", "MOVING_UP", "MOVING_DOWN" |
| `Deploy Amps` | double | Current draw |
| `Deploy Moving` | bool | Is it moving? |
| `Auto Intake` | string | Auto-deploy status |

### Power Management

| Key | Type | What it is |
|-----|------|------------|
| `Power State` | string | "NOMINAL", "WARNING", "CRITICAL", "EMERGENCY" |
| `Battery V` | double | Current battery voltage |
| `Discharge V/s` | double | How fast voltage is dropping |
| `Min V (Session)` | double | Lowest voltage this session |
| `Brownout Risk` | bool | Is brownout predicted? |
| `Max Motor °C` | double | Hottest motor temp |
| `Thermal Warning` | bool | Any motors too hot? |

### Vision

| Key | Type | What it is |
|-----|------|------------|
| `Vision Status` | string | Current state |
| `Vision Tag Count` | int | How many tags visible |
| `Vision Best Area` | double | Size of best tag (%) |
| `Vision Best Yaw` | double | Angle to best tag (°) |
| `Alignment` | string | "ALIGNED", "CLOSE", "OFF" |
| `Shot Profile` | string | "LINE_DRIVE" or "LOB" |
| `Shots Fired` | int | Total shots this session |

---

## NetworkTables Data

For coprocessor integration. Published under `Telemetry/` root.

### Telemetry/Odometry

| Key | Type | Description |
|-----|------|-------------|
| `x_meters` | double | X position on field |
| `y_meters` | double | Y position on field |
| `heading_deg` | double | Robot heading |
| `pose` | double[] | [x, y, heading] combined |
| `status` | string | "ACTIVE", "DISABLED", etc |

### Telemetry/Motors

Motors are auto-discovered on CAN bus (IDs 1-8).

| Key Pattern | Type | Description |
|-------------|------|-------------|
| `discovered_count` | int | Motors found |
| `discovered_ids` | string | List of CAN IDs |
| `motor_{id}/rpm` | double | Speed |
| `motor_{id}/amps` | double | Current |
| `motor_{id}/temp_c` | double | Temperature |
| `motor_{id}/status` | string | "OK" or "FAULT" |

**Motor CAN IDs:**
- 1, 2: Feeder
- 3, 4, 6, 7: Shooter (4 motors)
- 5: Intake rollers
- 8: Intake deploy arm

### Telemetry/Vision

| Key | Type | Description |
|-----|------|-------------|
| `tag_count` | int | Visible tags |
| `best_tag_area` | double | Size in frame (%) |
| `best_tag_yaw` | double | Angle (°) |
| `status` | string | "HUB_LOCKED", "NO_TAGS" |

### Telemetry/System

| Key | Type | Description |
|-----|------|-------------|
| `battery_voltage` | double | Battery V |
| `loop_time_ms` | double | Code loop time |
| `brownout_count` | int | Brownouts this session |

### Telemetry/Controller

For match replay. Logs all controller inputs.

| Key | Type | Description |
|-----|------|-------------|
| `left_x`, `left_y` | double | Left stick |
| `right_x`, `right_y` | double | Right stick |
| `l2_axis`, `r2_axis` | double | Triggers |
| `button_state` | int | Bitmask of buttons |

**Button bitmask:**
```
Bit 0: Square    Bit 4: L1      Bit 8: Create    Bit 12: PS
Bit 1: Cross     Bit 5: R1      Bit 9: Options   Bit 13: Touchpad
Bit 2: Circle    Bit 6: L2      Bit 10: L3
Bit 3: Triangle  Bit 7: R2      Bit 11: R3
```

### Telemetry/Replay

Frame sync for match replay.

| Key | Type | Description |
|-----|------|-------------|
| `frame_number` | int | Frame counter |
| `frame_timestamp` | double | FPGA time |
| `match_timestamp` | double | Time since start |
| `active_commands` | string | Running commands |

---

## Quick Examples

### Python - Read shooter data

```python
from networktables import NetworkTables

NetworkTables.initialize(server='10.58.05.2')
sd = NetworkTables.getTable('SmartDashboard')

rpm = sd.getNumber('Shooter Avg RPM', 0)
amps = sd.getNumber('Shooter Total Amps', 0)
print(f"Shooter: {rpm:.0f} RPM @ {amps:.1f}A")
```

### Python - Check power state

```python
state = sd.getString('Power State', 'UNKNOWN')
voltage = sd.getNumber('Battery V', 12.0)

if state == 'CRITICAL':
    print(f"LOW BATTERY: {voltage:.2f}V!")
```

### Python - Record match

```python
import json
import time

frames = []
replay = NetworkTables.getTable('Telemetry/Replay')
controller = NetworkTables.getTable('Telemetry/Controller')

def capture():
    frames.append({
        'frame': int(replay.getNumber('frame_number', 0)),
        'time': replay.getNumber('match_timestamp', 0),
        'buttons': int(controller.getNumber('button_state', 0)),
        'left_x': controller.getNumber('left_x', 0),
        'left_y': controller.getNumber('left_y', 0),
    })

# Run capture() in a loop at 50Hz, then save
with open('match.json', 'w') as f:
    json.dump(frames, f)
```

---

## Thresholds

| What | Good | Warning | Bad |
|------|------|---------|-----|
| Battery | >12V | 11-12V | <11V |
| Motor Temp | <60°C | 60-70°C | >70°C |
| Loop Time | <20ms | 20-40ms | >40ms |
| CAN Usage | <70% | 70-85% | >85% |

---

## Troubleshooting

**No data?**
1. Check robot is on
2. Verify IP (10.58.05.2)
3. Check firewall

**Missing motor?**
- Check CAN wiring
- Verify CAN ID in Phoenix Tuner
- Motor must be in expected IDs list (1-8)

**Odometry broken?**
- `DISABLED`: Pigeon ID set to -1 in Constants
- `NOT_FOUND`: Pigeon not on CAN bus
- `ERROR`: Check wiring

---

## Version History

| Version | Changes |
|---------|---------|
| 1.1.0 | Added per-motor telemetry, intake stats, power management |
| 1.0.0 | Initial release |
