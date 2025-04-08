/**
 * @file navigation.h
 * @brief Provides functions and data types for navigation-related calculations.
 *
 * This header file defines enumerations and data structures for handling quaternions and
 * calculating PWM values from joystick inputs. It also declares functions for inverting
 * quaternions and multiplying quaternions.
 *
 * @author michelecarenini
 * @date May 23, 2024
 */
#ifndef CORE_INC_NAVIGATION_H_
#define CORE_INC_NAVIGATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "interpolations.h"
#include <navigation/ThrusterConfigurationMatrix.h>
#include "arm_math.h"

/**
 * @brief Enumeration of PWM value constants.
 */
typedef enum {
	PWM_IDLE = 1500,
	PWM_MAX = 1670,
	PWM_MIN = 1330,
} PwmValues;

/**
 * @brief Data structure representing a quaternion.
 */
typedef struct {
	float w; // Scalar component of the quaternion
	float x; // Vector component x of the quaternion
	float y; // Vector component y of the quaternion
	float z; // Vector component z of the quaternion
} Quaternion;

/**
 * @brief Calculates PWM output values from command velocity
 *
 * @param cmd_vel: surge, sway, heave, roll, pitch, yaw
 * @param pwm_output: thrusters 1 to 8
 * @return eventual error code
 */
arm_status calculate_pwm(const float cmd_vel[6], uint32_t pwm_output[8]);

/**
 * @brief Inverts a quaternion.
 *
 * @param q Pointer to the input quaternion.
 * @param q_inv Pointer to the output inverted quaternion.
 */
void invert_quaternion(const Quaternion *q, Quaternion *q_inv);

/**
 * @brief Multiplies two quaternions.
 *
 * @param q1 Pointer to the first quaternion.
 * @param q2 Pointer to the second quaternion.
 * @param qResult Pointer to the output quaternion result.
 */
void multiply_quaternions(const Quaternion *q1, const Quaternion *q2, Quaternion *qResult);

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_NAVIGATION_H_ */
