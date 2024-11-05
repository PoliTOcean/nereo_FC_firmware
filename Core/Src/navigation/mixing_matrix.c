/*
 * mixing_matrix.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#include "navigation/mixing_matrix.h"

const float PIGRECO = 3.141592;


/**
 * The fixed 8x6 mixing matrix: the columns are, respectively: foward, sway, heave, pitch, roll, yaw. (have to double check with the other code and my calculation)
 * The rows are, respectively, thrusters from 0 to 7.
 */
__attribute__((aligned(4))) float FIXED_MIXING_MATRIX[8][6] = {
    {0.7071067812, 0.7071067812, 0, 0, 0, 1},
    {-0.7071067812, 0.7071067812, 0, 0, 0, -1},
    {-0.7071067812, 0.7071067812, 0, 0, 0, -1},
    {0.7071067812, 0.7071067812, 0, 0, 0, 1},
    {0, 0, 1, 1, -1, 0},
    {0, 0, 1, 1, 1, 0},
    {0, 0, 1, -1, -1, 0},
    {0, 0, 1, -1, 1, 0}
};

/**
 * Calculates a 6x6 rotation matrix from a 4-element quaternion.
 *
 * The 6x6 rotation matrix is constructed by first calculating a 3x3 rotation
 * matrix from the quaternion, and then placing that 3x3 matrix in the top-left
 * and bottom-right corners of the 6x6 matrix, with the remaining elements set
 * to 0.
 * As of now this is not used, may use it in the future.
 *
 * @param rotation_matrix The 6x6 rotation matrix to be calculated.
 * @param quaternion The 4-element quaternion to use for the calculation.
 */
void calculate_rotation_matrix_6d(float rotation_matrix[6][6], float quaternion[4])
{
    float rotation_matrix_3d[3][3];
    quaternion_to_rotation_matrix_3d(rotation_matrix_3d, quaternion);

    for (uint8_t i = 0; i < 3; i++)
    {
        for (uint8_t j = 0; j < 3; j++)
        {
            rotation_matrix[i][j] = rotation_matrix_3d[i][j];
            rotation_matrix[i][j + 3] = 0;
            rotation_matrix[i + 3][j] = 0;
            rotation_matrix[i + 3][j + 3] = rotation_matrix_3d[i][j];
        }
    }
}


