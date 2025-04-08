/*
 * cs_control_stabilize_mode.cpp
 *
 *  Created on: Mar 26, 2025
 *      Author: michelecarenini
 */
/*
#include "stabilize_mode.h"

static float Kx0[] = {198.0952, 468.0585};
static float Kx1[] = {3.8191, 6.0003};
static float Kx2[] = {5.2577, 11.8206};
static float Ki0 = -359.2481;
static float Ki1 = 0;
static float Ki2 = -16.7389;

ControlSystem controllers[3];

void controllers_init() {
	controllers[0] = ControlSystem::ControlSystem(-60, 80, Kx0, Ki0); // heave
	controllers[1] = ControlSystem::ControlSystem(-30, 30, Kx1, Ki1); // roll
	controllers[2] = ControlSystem::ControlSystem(-30, 30, Kx2, Ki2); // pitch
}

arm_status calculate_pwm_cs_controller(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure) {
	update_setpoints(cmd_vel, orientation_quaternion, water_pressure);

	float heave_measurements[2] = {water_pressure};
	float heave_correction = controllers[0].calculateU(setpoints[0], *water_pressure, heave_measurements);

	return calculate_pwm(cmd_vel, pwm_output);
}


*/
