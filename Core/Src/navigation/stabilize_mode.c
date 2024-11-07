/*
 * stabilize_mode.c
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 */


#include "navigation/stabilize_mode.h"

// tolerance: if a joystick input (in [-1,1]) is < TOLERANCE, it is considered as 0
const float TOLERANCE = 0.05;

float setpoints[4];

// PIDs controllers, respectively for z, pitch, roll, yaw
arm_pid_instance_f32 pids[4] = {0};

void calculate_rpy_from_quaternion(const Quaternion *quaternion, float roll_pitch_yaw_radians[3])
{
	// roll (x-axis rotation)
	float sinr_cosp = 2 * (quaternion->w * quaternion->x + quaternion->y * quaternion->z);
	float cosr_cosp = 1 - 2 * (quaternion->x * quaternion->x + quaternion->y * quaternion->y);
	roll_pitch_yaw_radians[0] = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	float sinp, cosp;
	arm_sqrt_f32(1 + 2 * (quaternion->w * quaternion->y - quaternion->x * quaternion->z), &sinp);
	arm_sqrt_f32(1 - 2 * (quaternion->w * quaternion->y - quaternion->x * quaternion->z), &cosp);
	roll_pitch_yaw_radians[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (quaternion->w * quaternion->z + quaternion->x * quaternion->y);
	float cosy_cosp = 1 - 2 * (quaternion->y * quaternion->y + quaternion->z * quaternion->z);
	roll_pitch_yaw_radians[2] = atan2(siny_cosp, cosy_cosp);
}

uint8_t update_setpoints(const float input_values[6], const Quaternion *quat, const float *water_pressure)
{
	uint8_t count = 0;
	float rpy_rads[3];
	calculate_rpy_from_quaternion(quat, rpy_rads);
	// updates setpoints for angles
	for(uint8_t i = 0; i < 3; i++)
	{
		if(fabsf(input_values[i+3]) < TOLERANCE)
		{
			setpoints[i+1] = rpy_rads[i+1];
			count++;
		}
	}
	/*
	 * Updates depth setpoint
	 * In order for the setpoint to be update, I have to check the role each axis plays in changing the depth,
	 * and updating the setpoint only if all of the corresponding input values are 0
	 */


	return count;
}

void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER])
{
    for(uint8_t i = 0; i < PID_NUMBER; i++)
    {
    	pids[i].Kp = kps[i];
    	pids[i].Ki = kis[i];
    	pids[i].Kd = kds[i];
        arm_pid_init_f32(&pids[i], 0);
    }
}

// The order for 4-elements arrays is: z, pitch, roll, yaw
uint8_t calculate_pwm_with_pid(const float joystick_input[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure) {
	// calculate current values
	float current_values[4];
	calculate_rpy_from_quaternion(orientation_quaternion, &current_values[1]);

	// TODO conversion from water pressure to depth
	current_values[0] = *water_pressure;

	update_setpoints(joystick_input, orientation_quaternion, water_pressure);
	// PID in action!
	float input_values[6];
	for(uint8_t i = 0; i < 6; i++) input_values[i] = joystick_input[i];

	float pitch_pid_feedback = arm_pid_f32(&pids[1], setpoints[1] - current_values[1]);
	float roll_pid_feedback = arm_pid_f32(&pids[2], setpoints[2] - current_values[2]);
	float yaw_pid_feedback = arm_pid_f32(&pids[3], setpoints[3] - current_values[3]);

	/* **************
	 * Depth
	 * The z axis we can get measures of is in the fixed-body-frame:
	 * we need to convert the output of the PID to the body frame in order to modify the input, in order to achieve the desired depth hold.
	*/
	float z_out = arm_pid_f32(&pids[0], setpoints[0] - current_values[0]);

	// Applies the inverse rotation of the body-frame from the fixed-body-frame ( described by the orientation quaternion ),
	// in order to compute the coordinates of the z_out vector with respect to the body-frame
	Quaternion z_out_q;
	z_out_q.w = 0;
	z_out_q.x = 0;
	z_out_q.y = 0;
	z_out_q.z = z_out;
	Quaternion q_inv = {0};
	invert_quaternion(orientation_quaternion, &q_inv);
	
	// applies the inverse rotation to the z_out_q vector
	Quaternion intermediate_result = {0};
	Quaternion z_out_body_frame = {0};
	multiply_quaternions(&q_inv, &z_out_q, &intermediate_result);
	multiply_quaternions(&intermediate_result, orientation_quaternion, &z_out_body_frame);

	// apply the feedback on x y z axis if and only if either the feedback is approx 0, or the input value by the user is approx 0.
	// This condition must be met for every axis value
	uint8_t y_condition = fabsf(z_out_body_frame.y) < TOLERANCE || fabsf(input_values[0] < TOLERANCE);
	uint8_t x_condition = fabsf(z_out_body_frame.x) < TOLERANCE || fabsf(input_values[1] < TOLERANCE);
	uint8_t z_condition = fabsf(z_out_body_frame.z) < TOLERANCE || fabsf(input_values[2] < TOLERANCE);

	if (x_condition && y_condition && z_condition)
	{
		input_values[0] += z_out_body_frame.y;
		input_values[1] += z_out_body_frame.x;
		input_values[2] += z_out_body_frame.z;
	}

	// pitch
	if (fabsf(pitch_pid_feedback) < TOLERANCE || fabsf(input_values[3] < TOLERANCE))
	{
		input_values[3] += pitch_pid_feedback;
	}
	// roll
	if (fabsf(roll_pid_feedback) < TOLERANCE || fabsf(input_values[4] < TOLERANCE))
	{
		input_values[4] += roll_pid_feedback;
	}
	// yaw
	if (fabsf(yaw_pid_feedback) < TOLERANCE || fabsf(input_values[5] < TOLERANCE))
	{
		input_values[5] += yaw_pid_feedback;
	}

	uint8_t code = calculate_pwm(&input_values, pwm_output);
	return code;
}
