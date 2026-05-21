# ROS 2 Interfaces

All topics use the global namespace. The micro-ROS node is named `fc_node`.

---

## Subscriptions

### `/nereo_cmd_vel` — `nereo_interfaces/msg/CommandVelocity`
Command velocity. Array of 6 floats in `[-1.0, 1.0]`:

| Index | DOF   | Description          |
|-------|-------|----------------------|
| 0     | surge | Forward/backward     |
| 1     | sway  | Left/right           |
| 2     | heave | Up/down              |
| 3     | roll  | Roll                 |
| 4     | pitch | Pitch                |
| 5     | yaw   | Yaw                  |

Receiving a message on this topic disables thruster test mode.

### `/imu_data` — `sensor_msgs/msg/Imu`
ROV orientation, linear acceleration and angular velocity. Used only in stabilization modes.

### `/water_pressure` — `sensor_msgs/msg/FluidPressure`
External water pressure. Used for depth stabilization.

### `/set_arm_mode` — `std_msgs/msg/Bool`
Arms (`true`) or disarms (`false`) the ROV. When disarmed all thrusters go to idle PWM (1500µs). The ROV starts disarmed and returns to disarmed on agent disconnection.

### `/set_nav_mode` — `std_msgs/msg/Int32`
Sets the navigation mode. See [nav_mode.md](nav_mode.md) for available values.

### `/thruster_pwm_test` — `std_msgs/msg/Int32MultiArray`
Test mode: sends 8 PWM values directly to the thrusters, bypassing the mixing matrix. Array of 8 integers in µs (typically 1100–1900). Deactivated on the next `/nereo_cmd_vel` message.

---

## Publications

### `/thruster_status` — `nereo_interfaces/msg/ThrusterStatuses`
Current PWM values for all 8 thrusters in µs. Published at every task cycle (40 Hz).

### `/rov_armed` — `std_msgs/msg/Bool`
Current arm state (`true` = armed).
