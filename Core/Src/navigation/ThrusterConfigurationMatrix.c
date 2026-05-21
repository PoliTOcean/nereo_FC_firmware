/*
 * mixing_matrix.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#include <navigation/ThrusterConfigurationMatrix.h>

// Columns: surge, sway, heave, roll, pitch, yaw
// Rows: pwms[0..7] → motors 1,5,7,6,2,8,3,4
// Vertical signs verified by bench test. Horizontal signs need testing.
__attribute__((aligned(4))) float FIXED_MIXING_MATRIX[8][6] = {
    { 0.7071, -0.7071,  0,  0,  0, -1},  // pwms[0] Motor 1 - FR orizz
    { 0,       0,       1, -1,  1,  0},   // pwms[1] Motor 5 - FR vert  (↑ con PWM+)
    { 0,       0,      -1,  1,  1,  0},   // pwms[2] Motor 7 - RR vert  (↓ con PWM+)
    { 0,       0,      -1, -1, -1,  0},   // pwms[3] Motor 6 - FL vert  (↓ con PWM+)
    { 0.7071,  0.7071,  0,  0,  0,  1},   // pwms[4] Motor 2 - FL orizz
    { 0,       0,       1,  1, -1,  0},   // pwms[5] Motor 8 - RL vert  (↑ con PWM+)
    { 0.7071,  0.7071,  0,  0,  0, -1},   // pwms[6] Motor 3 - RR orizz
    { 0.7071, -0.7071,  0,  0,  0,  1},   // pwms[7] Motor 4 - RL orizz
};
