/*
 * pid.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#include "pid.h"

// Initialize the PID controller
/**
 * Initializes a PID controller with the given parameters.
 *
 * @param pid       Pointer to the PID controller struct to initialize.
 * @param kp        Proportional gain.
 * @param ki        Integral gain.
 * @param kd        Derivative gain.
 */
void PID_Init(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// Update the PID controller
/**
 * Updates the PID controller with the current value and returns the new output.
 *
 * @param pid The PID controller to update.
 * @param current_value The current value to use in the PID calculation.
 * @param setpoint The desired setpoint or target value.
 * @param dt The time step or sample period.
 * @return The new output value from the PID controller.
 */
float PID_Update(PID *pid, float current_value, float setpoint, float dt)
{
    float error = setpoint - current_value;

    float proportional_term_out = pid->kp * error;

    pid->integral += error * dt;
    float integral_term_out = pid->ki * pid->integral;

    float derivative = (error - pid->prev_error) / dt;
    float derivative_term_out = pid->kd * derivative;

    float output = proportional_term_out + integral_term_out + derivative_term_out;

    pid->prev_error = error;

    return output;
}

/**
 * Sets the proportional, integral, and derivative gain constants for a PID controller.
 *
 * @param pid Pointer to the PID controller struct to update.
 * @param kp Proportional gain.
 * @param ki Integral gain.
 * @param kd Derivative gain.
 */
void PID_set_constants(PID *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
