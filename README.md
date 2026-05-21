# Nereo FC Firmware

Firmware for Nereo's Flight Controller board (STM32F469VIT6). Receives command velocity from the Raspberry Pi over micro-ROS serial, computes PWM values for 8 thrusters via a mixing matrix, and drives the ESCs. Supports manual control and PID-based stabilization modes.

## Requirements

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Docker (used by the Makefile to build the micro-ROS static library)
- ROS 2 Humble + micro-ROS agent on the Raspberry Pi

## Build & Flash

Clone with submodules:
```bash
git clone --recurse-submodules <repo-url>
```

Open the project in STM32CubeIDE, then build with `Ctrl+B`. Docker pulls the micro-ROS builder automatically on the first build.

Flash with `Ctrl+F11` via ST-Link.

## Running

Start the micro-ROS agent on the Pi:
```bash
sudo chmod a+rw /dev/ttyAMA0
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyAMA0
```

The firmware connects automatically. If the agent is restarted, the firmware reconnects on its own — no need to power-cycle the ROV.

## ROS 2 Commands

**Arm / Disarm:**
```bash
ros2 topic pub --once /set_arm_mode std_msgs/msg/Bool "data: true"
ros2 topic pub --once /set_arm_mode std_msgs/msg/Bool "data: false"
```

**Check arm state:**
```bash
ros2 topic echo /rov_armed
```

**Send command velocity** `[surge, sway, heave, roll, pitch, yaw]`:
```bash
ros2 topic pub --once /nereo_cmd_vel nereo_interfaces/msg/CommandVelocity "cmd_vel: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
```

**Test singol motor** (bypass mixing matrix, indici 0-7):
```bash
ros2 topic pub --once /thruster_pwm_test std_msgs/msg/Int32MultiArray "data: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]"
```
Mapping indici → motori fisici:

| Indice | Motore | Posizione |
|--------|--------|-----------|
| 0 | 1 | Front DX orizzontale |
| 1 | 5 | Front DX verticale |
| 2 | 7 | Rear DX verticale |
| 3 | 6 | Front SX verticale |
| 4 | 2 | Front SX orizzontale |
| 5 | 8 | Rear SX verticale |
| 6 | 3 | Rear DX orizzontale |
| 7 | 4 | Rear SX orizzontale |

> **Nota:** inviare qualsiasi cmd_vel disattiva automaticamente la modalità test motori.

**Stato topics e servizi:**
```bash
ros2 topic list
ros2 service list
```

The ROV always starts disarmed and returns to disarmed state on agent disconnection. Re-arming requires an explicit command.

## Docs

- [Project structure](docs/project_structure.md)
- [ROS 2 interfaces](docs/interfaces.md)
- [Navigation modes](docs/nav_mode.md)
