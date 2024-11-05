/*
 * interpolations.h
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */
#ifndef CORE_INC_INTERPOLATIONS_H_
#define CORE_INC_INTERPOLATIONS_H_

#include <stdint.h>
#include <math.h>

/**
 * @brief Performs linear interpolation between two values.
 *
 * @param input_value The input value to interpolate.
 * @param input_min The minimum input value.
 * @param input_max The maximum input value.
 * @param output_min The minimum output value.
 * @param output_max The maximum output value.
 * @return The interpolated output value.
 */
float linear_interpolation(float input_value, float input_min, float input_max, float output_min, float output_max);

/**
 * @brief Performs quadratic interpolation, where the interpolazion in symmetric with respect to the origin,
 * i.e. max_output = -min_output, max_input = -min_input.
 *
 * @param input_value The input value to interpolate.
 * @param input_max The maximum input value.
 * @param output_max The maximum output value.
 * @return The interpolated output value.
 */
float symmetric_quadratic_interpolation(float input_value, float input_max, float output_max);

/**
 * @brief Normalizes a vector of floating-point values, if and only if the maximum absolute value is greater than one.
 *
 * @param input_array The input array to normalize.
 * @param output_array The output array to store the normalized values.
 * @param size The size of the input and output arrays.
 */
void normalize_vector(const float *input_array, float *output_array, uint8_t size);

#endif /* CORE_INC_INTERPOLATIONS_H_ */
