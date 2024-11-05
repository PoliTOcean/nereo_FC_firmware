/*
 * mixing_matrix.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#ifndef CORE_INC_MIXING_MATRIX_H_
#define CORE_INC_MIXING_MATRIX_H_

/**
 * @file mixing_matrix.h
 * @brief Provides functions for calculating rotation matrices and the fixed mixing matrix.
 *
 * This file defines a fixed mixing matrix and functions for calculating 3D and 6D rotation matrices from quaternions.
 *
 * @param FIXED_MIXING_MATRIX The 8x6 fixed mixing matrix.
 * @param calculate_rotation_matrix_6d Calculates a 6x6 rotation matrix from a 4-element quaternion.
 * @param init_fixed_mixing_matrix Initializes the FIXED_MIXING_MATRIX.
 */

#include <stdint.h>
#include <math.h>
#include "arm_math.h"

__attribute__((aligned(4))) extern float FIXED_MIXING_MATRIX[8][6];

void init_fixed_mixing_matrix(float mixing_matrix[8][6]);

#endif /* CORE_INC_MIXING_MATRIX_H_ */
