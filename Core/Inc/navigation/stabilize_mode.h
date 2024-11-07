/*
 * stabilize_mode.h
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 *
 */

#ifndef CORE_INC_STABILIZE_MODE_H_
#define CORE_INC_STABILIZE_MODE_H_

#include "navigation.h"
#include "arm_math.h"
#include <stdint.h>
#include <math.h>

#define PID_NUMBER 4

extern arm_pid_instance_f32 pids[PID_NUMBER];

// returns the number of setpoints updated
uint8_t update_setpoints(const float input_values[6], const Quaternion *quat, const float *water_pressure);

void calculate_rpy_from_quaternion(const Quaternion *quaternion, float roll_pitch_yaw_radians[3]);

void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER]);

uint8_t calculate_pwm_with_pid(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure);

#endif /* CORE_INC_STABILIZE_MODE_H_ */
