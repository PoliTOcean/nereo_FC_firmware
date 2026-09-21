# Navigation Modes

Set the mode by publishing an integer to `/set_nav_mode` (`std_msgs/msg/Int32`).

> **These modes are not in use, and nothing sets them.** The vehicle boots in
> `NAVIGATION_MODE_MANUAL` and stays there: no node in `nereo_ros2_code` or
> `ros2_controller_tuning_aid` publishes `/set_nav_mode` or calls the
> `SetNavigationMode` service, verified 2026-09-21. Manual is pass-through of
> `/nereo_cmd_vel`, which `safety_node` feeds from either the joystick or
> `nereo_controller_node`.
>
> **The vehicle's real attitude and depth controller is
> `nereo_controller_node` on the workstation**, not this firmware's
> `stabilize_mode.c` (operator decision 2026-09-21; see the project's Key
> Decisions). The firmware's PID is a second implementation of the same job
> that was left behind when control moved to the ROS side: commit `e3e59be`
> removed the parameter server that fed it, so all four of its gains are zero
> and no production code path sets them. Selecting mode 15 today therefore
> behaves like manual with corrections of exactly zero.
>
> **Do not restore gains to it without reopening that decision.** Two
> controllers commanding the same eight thrusters is how you get behaviour
> nobody can explain at the bench — and the trap got closer on 2026-09-21,
> when a QoS fix connected `/imu_data` to the firmware for the first time, so
> the input this PID had always lacked is now present.
>
> Modes other than 0 and 15 are unimplemented: the safety arbiter maps them to
> `ARBITER_UNKNOWN_MODE`, which writes neutral to all eight thrusters.

| Value | Name                              | Description                                  |
|-------|-----------------------------------|----------------------------------------------|
| `0`   | `NAVIGATION_MODE_MANUAL`          | Manual control, no stabilization             |
| `1`   | `NAVIGATION_MODE_STABILIZE_DEPTH` | Depth (heave) stabilization only             |
| `6`   | `NAVIGATION_MODE_STABILIZE_R_P`   | Roll and pitch stabilization                 |
| `7`   | `NAVIGATION_MODE_STABILIZE_ANGLES`| Roll, pitch and yaw stabilization            |
| `15`  | `NAVIGATION_MODE_STABILIZE_FULL`  | Full stabilization: depth, roll, pitch, yaw  |
| `16`  | `NAVIGATION_MODE_STABILIZE_CS`    | Reserved (control system — not implemented)  |

Values are a bitmask of the stabilized DOFs:

```
bit 0 = heave
bit 1 = roll
bit 2 = pitch
bit 3 = yaw
bit 4 = CS
```

### Examples

```bash
ros2 topic pub --once /set_nav_mode std_msgs/msg/Int32 "data: 0"   # manual
ros2 topic pub --once /set_nav_mode std_msgs/msg/Int32 "data: 15"  # full stabilization
```
