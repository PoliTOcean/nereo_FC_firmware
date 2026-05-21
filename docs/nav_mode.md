# Navigation Modes

Set the mode by publishing an integer to `/set_nav_mode` (`std_msgs/msg/Int32`).

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
