/*
 * stabilize_mode.h
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 *
 */

#ifndef CORE_INC_STABILIZE_MODE_H_
#define CORE_INC_STABILIZE_MODE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "navigation.h"
#include "arm_math.h"
#include <stdint.h>
#include <math.h>

#define PID_NUMBER 4

extern arm_pid_instance_f32 pids[PID_NUMBER];

/**
 * @brief Updates setpoints for angles and depth based on joystick input and current orientation/pressure data.
 *
 * @param input_values Array of 6 joystick input values: [surge, sway, heave, roll, pitch, yaw]
 * @param quat Pointer to a Quaternion structure representing the current orientation.
 * @param water_pressure Pointer to the current water pressure measurement.
 * @return uint8_t The count of updated setpoints (roll, pitch, yaw, or depth).
 */
uint8_t update_setpoints(const float input_values[6], const Quaternion *quat, const float *water_pressure);

void calculate_rpy_from_quaternion(const Quaternion *quaternion, float roll_pitch_yaw_radians[3]);

void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER]);

/**
 * @brief Calculates the PWM output using PID control to adjust orientation and depth based on joystick input.
 *
 * @param joystick_input Array of 6 cmd velocity values: [surge, sway, heave, roll, pitch, yaw].
 * @param pwm_output Array to store the 8 calculated PWM output values (thrusters 1 to 8, referring to BlueRobotics' Blue Rov Heavy)
 * @param orientation_quaternion Pointer to the current orientation as a quaternion.
 * @param water_pressure Pointer to the current water pressure (used to estimate depth).
 * @return int8_t A status code indicating the success or failure of the PWM calculation.
 */
arm_status calculate_pwm_with_pid(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure);

arm_status calculate_pwm_with_pid_anti_windup(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure);

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_STABILIZE_MODE_H_ */
