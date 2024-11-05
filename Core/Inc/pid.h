
/**
 * @file pid.h
 * @brief Provides a PID controller implementation.
 * @author michele carenini
 * @date May, 2024
 *
 * This header file defines a PID (Proportional-Integral-Derivative) controller
 * structure and functions to initialize, update, and set the constants of the
 * PID controller.
 *
 * @param kp Proportional gain constant.
 * @param ki Integral gain constant.
 * @param kd Derivative gain constant.
 * @param integral Accumulated integral error.
 * @param prev_error Previous error value.
 *
 * @function PID_Init Initializes the PID controller with the given gain constants.
 * @function PID_Update Calculates the PID controller output based on the current value, setpoint, and time delta.
 * @function PID_set_constants Sets the PID constants.
 */

#ifndef CORE_INC_PID_H_
#define CORE_INC_PID_H_

#include <stdio.h>

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd);

float PID_Update(PID *pid, float current_value, float setpoint, float dt);

void PID_set_constants(PID *pid, float kp, float ki, float kd);

#endif /* CORE_INC_PID_H_ */
