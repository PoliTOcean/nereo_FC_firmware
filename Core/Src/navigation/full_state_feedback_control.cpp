/*
 * full_state_feedback_control.cpp
 *
 *  Created on: Dec 3, 2024
 *      Author: A guy from Control System Area
 */

#include "navigation/full_state_feedback_control.h"

ControlSystem::ControlSystem(float minForce, float maxForce, float * Kx, float Ki):


    minForce(80),
    maxForce(-60),
    maxErrorIntegral(80),
    minErrorIntegral(-60),
    Kx(Kx),
    Ki(Ki)
{
    this->ErrorIntegral= 0.0;
}

// Computes the output of the controller
float ControlSystem::calculateU(float reference, float y_measurement, float * x_measurement)
{
    float error = reference - y_measurement;

    ErrorIntegral += error;

    // Saturate error
    if(ErrorIntegral > maxErrorIntegral)
    {
        ErrorIntegral = maxErrorIntegral;
    }
    else if(ErrorIntegral < minErrorIntegral)
    {
        ErrorIntegral = minErrorIntegral;
    }

    // Calculate force
    float unSatForce = -Kx[0] * x_measurement[0] -Kx[1] * x_measurement[1] - Ki * ErrorIntegral;

    // Saturate force
    if(unSatForce > maxForce) {
        unSatForce = maxForce;
    }
    else if(unSatForce < minForce) {
        unSatForce = minForce;
    }

    return unSatForce;
}
