# Project Structure

```
nereo_FC_firmware/
├── Core/
│   ├── Inc/
│   │   ├── FC_app.h                  # Includes, typedefs, macros and function prototypes
│   │   └── navigation/
│   │       ├── navigation.h          # PWM computation (mixing matrix)
│   │       └── stabilize_mode.h      # PID stabilization
│   └── Src/
│       ├── freertos.cpp              # Main task: micro-ROS state machine, control loop
│       └── navigation/
│           ├── ThrusterConfigurationMatrix.c  # Thruster configuration matrix
│           ├── navigation.c          # PWM computation implementation
│           └── stabilize_mode.c      # PID and stabilization implementation
│
├── Drivers/                          # ST HAL and CMSIS (CubeMX-generated)
├── Middlewares/                      # FreeRTOS and ARM DSP library
├── micro_ros_stm32cubemx_utils/      # micro-ROS library (git submodule)
│   └── microros_static_library_ide/
│       └── libmicroros/              # Precompiled static library (built by Docker)
├── docs/                             # Documentation
│   ├── interfaces.md                 # ROS 2 topics (publishers and subscribers)
│   ├── nav_mode.md                   # Available navigation modes
│   └── project_structure.md          # This file
└── README.md
```

## Key files

### `Core/Src/freertos.cpp`
Main FreeRTOS task. Implements the micro-ROS agent connection state machine (`WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED`) and the 40 Hz control loop.

### `Core/Inc/FC_app.h`
Pulls in all required headers (micro-ROS, HAL, ROS 2 messages), defines typedefs (`NavigationModes`, `RovArmModes`), configuration macros and function prototypes.

### `Core/Src/navigation/ThrusterConfigurationMatrix.c`
Maps the 6 DOFs (surge, sway, heave, roll, pitch, yaw) to the 8 thrusters. Update this file if the physical thruster layout changes.

### `Core/Src/navigation/navigation.c`
Multiplies the configuration matrix by the command vector to produce per-thruster PWM values.

### `Core/Src/navigation/stabilize_mode.c`
PID controllers for stabilization modes. Gains are set directly in code.
