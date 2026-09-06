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

If you already cloned without `--recurse-submodules`, populate them now:
```bash
git submodule update --init --recursive
```
`micro_ros_stm32cubemx_utils/` must not be empty. When it is, the pre-build step fails inside the container with `dos2unix: /project/.../library_generation.sh: No such file or directory`, which does not name the real cause.

Make sure Docker Desktop is running, then open the project in STM32CubeIDE and build with `Ctrl+B`. Docker pulls the micro-ROS builder automatically on the first build.

Flash with `Ctrl+F11` via ST-Link.

### Custom ROS 2 messages

The firmware uses `nereo_interfaces`, which has to be compiled into the micro-ROS static library. It is declared in `microros_component/extra_packages/extra_packages.repos`, which `library_generation.sh` reads as a user extension point.

Declare custom packages **there**, never in the copy inside `micro_ros_stm32cubemx_utils/` — that file belongs to a pinned submodule, so edits to it cannot be committed and are lost on the next clone or submodule update.

The static library is only generated when it is missing, so after changing that file force a rebuild:
```bash
rm -rf micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
```

### Known build gotchas

- **Checkout path**: the pre-build step mounts the project into Docker. The path is quoted, so characters like `&` are safe — but if you edit the pre-build step (Project → Properties → C/C++ Build → Settings → Build Steps), keep the quotes and keep `${workspace_loc:/${ProjName}}` rather than pasting an absolute path, or the project stops building for everyone else.
- **`Debug/makefile` is generated.** Never fix build problems by editing it; change the corresponding setting in the project properties instead.
- Each containerised pre-build runs `dos2unix` on the submodule's `library_generation.sh`, which leaves `micro_ros_stm32cubemx_utils` showing as modified. That is expected; discard it with `git -C micro_ros_stm32cubemx_utils checkout -- .`

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
