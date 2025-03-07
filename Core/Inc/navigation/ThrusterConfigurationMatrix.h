/*
 * mixing_matrix.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_MIXING_MATRIX_H_
#define CORE_INC_MIXING_MATRIX_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>
#include "arm_math.h"

/**
 * The fixed 8x6 mixing matrix: the columns are, respectively: surge, sway, heave, roll, pitch, yaw.
 * The rows are, respectively, thrusters from 1 to 8. Refer to BlueRobotics' Blue ROV Heavy configuration.
 */
__attribute__((aligned(4))) extern float FIXED_MIXING_MATRIX[8][6];

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_MIXING_MATRIX_H_ */
