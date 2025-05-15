# Command Velocity
The FC is subscribed to a topic "/nereo_cmd_vel" of type nereo_interfaces/msg/CommandVelocity. By publishing data on this topic it is possible to control the ROV. The command velocity, which is a vector of 6 floats, is composed of: 
- 3 floats for the linear velocity: surge, sway, heave
- 3 floats for the angular velocity: roll, pitch, yaw
Each float input must be in [-1, 1] interval, where 1 is the maximum speed foward, 0 is no speed and -1 is the maximum speed backwards.
# Thruster Status
The FC publishes to a topic "/thruster_pwm" of type nereo_interfaces/msg/ThrusterStatus. The message contains 8 floats, one for each thruster. Each float is the PWM value calculated for the corresponding thruster, given the inputs.
# IMU
The FC subscribes to a topic "/imu_data" of type sensor_msgs/msg/Imu. The message contains the orientation, linear acceleration and angular velocity of the ROV. This is only used when performing stabilization.
# Depth readings
The FC subscribes to a topic "/water_pressure" of type sensor_msgs/msg/FluidPressure. The message contains the pressure of the water outside of the ROV. This is only used when performing stabilization.
# Arm ROV service
The FC provides a service "/set_rov_arm_mode" of type std_srvs/srv/SetBool. This service allows to arm or disarm the ROV by setting the boolean input to true (arm rov) or false (disarm rov).
# Change navigation mode service
The FC provides a service "/set_rov_navigation_mode" of type nereo_interfaces/srv/SetNavigationMode. This service allows to change the navigation mode by setting the input to the desired mode: refer to [navigation_modes.mcl](nav_mode.md) for the list of available modes and their description.
