# ROS 2 Interfaces

All topics use the global namespace. The micro-ROS node is named `fc_node`.

## Reliability QoS

The two sensor subscriptions, `/imu_data` and `/barometer_pressure`, are
**best-effort**, matching the Raspberry Pi's sensor publishers, which use
`getSensorQoS()` (best-effort, volatile). The four command subscriptions are
**reliable**, matching the workstation's command publishers, which use the
rclpy default.

This is a compatibility requirement, not a preference. A reliable subscription
and a best-effort publisher are QoS-incompatible: DDS matches nothing and the
subscription receives no messages at all, silently, while both sides look
healthy. Changing either end means changing the other.

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

### `/barometer_pressure` — `sensor_msgs/msg/FluidPressure`
External water pressure, published by the Raspberry Pi's barometer
node on a 300 ms wall timer. Used for depth stabilization. The
firmware judges the freshness of its own cached copy of the latest
message rather than trusting that a message has recently arrived —
see `/pressure_data_valid` below for how a consumer can tell the
difference between "at the surface" and "sensor dead".

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

### `/pressure_data_valid` — `std_msgs/msg/Bool`
Whether the firmware's cached barometer reading is fresh enough to
control the depth axis on. Published at every task cycle (40 Hz), the
same rate as `/thruster_status` and `/rov_armed`.

- **`true`** — the firmware has received a `/barometer_pressure`
  message within the last 900 ms (three times the Raspberry Pi's
  300 ms publish period), and the depth axis is running its normal
  PID control on that reading.
- **`false`** — the firmware's cached pressure is older than 900 ms,
  or has never been received since boot. The depth axis has degraded
  to pilot passthrough (the depth PID is not evaluated; the pilot's
  own heave command passes through unmodified), while roll, pitch and
  yaw stabilization continue running normally. `false` is distinct
  from a valid physical pressure reading of zero — this is the whole
  reason the topic exists: a consumer cannot tell "at the surface"
  from "sensor dead" from the raw pressure value alone.
- **Absence of the topic** means the firmware is not connected to the
  micro-ROS agent or is not publishing. Absence must never be read as
  the sensor being healthy — a consumer that treats a missing topic
  as good news reintroduces the exact defect this topic exists to
  remove.

This is a firmware-side constant (`PRESSURE_STALENESS_BUDGET_MS` in
`Core/Inc/FC_app.h`, currently 900 ms); a consumer should read this
topic rather than hardcode the budget independently.

This signal is distinct from the Raspberry Pi's own
`barometer_diagnostic` topic, which reports whether its I2C
transaction to the physical MS5837 sensor succeeded this cycle.
`/pressure_data_valid` answers a different question: whether *this*
control loop has received a usable value recently enough to control
on. The best-effort micro-ROS bridge between the Pi and the firmware
can drop every message while the Pi's I2C read stays perfectly
healthy, so neither topic substitutes for the other.
