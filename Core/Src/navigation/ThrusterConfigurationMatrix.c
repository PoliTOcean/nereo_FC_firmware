/*
 * mixing_matrix.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */

#include <navigation/ThrusterConfigurationMatrix.h>

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

