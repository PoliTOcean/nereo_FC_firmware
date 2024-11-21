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

/**
 * @file mixing_matrix.h
 * @brief Provides functions for calculating rotation matrices and the fixed mixing matrix.
 *
 * This file defines a fixed mixing matrix and functions for calculating 3D and 6D rotation matrices from quaternions.
 *
 * @param FIXED_MIXING_MATRIX The 8x6 fixed mixing matrix.
 */

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
