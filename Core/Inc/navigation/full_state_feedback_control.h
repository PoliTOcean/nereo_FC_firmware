/*
 * full_state_feedback_control.h
 *
 *  Created on: Dec 3, 2024
 *      Author: root
 */

#ifndef INC_NAVIGATION_FULL_STATE_FEEDBACK_CONTROL_H_
#define INC_NAVIGATION_FULL_STATE_FEEDBACK_CONTROL_H_

class ControlSystem
{
    public:

        //Constructor
        ControlSystem(float minForce, float maxForce, float Kx[2], float Ki);

        //Returns the output of the controller given a reference signal and the actual measured state of the system
        float calculateU(float reference, float y_measurement, float * x_measurement);

    public:

        float minForce;  //for control input saturation purposes
        float maxForce;  //for control input saturation purposes
        float maxErrorIntegral; //for integrator anti-windup
        float minErrorIntegral; //for integrator anti-windup

        // Variable for control laws's coefficients
        float Kx[2];
        float Ki;

        //Controller memory
        float ErrorIntegral;
};

#endif /* INC_NAVIGATION_FULL_STATE_FEEDBACK_CONTROL_H_ */
