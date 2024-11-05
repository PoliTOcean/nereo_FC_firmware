/*
 * interpolations.c
 *
 *  Created on: May 23, 2024
 *      Author: michelecarenini
 */
#include "interpolations.h"

/**
 * Linearly interpolates a value between a given input range and output range.
 *
 * @param input_value The input value to interpolate.
 * @param input_min The minimum value of the input range.
 * @param input_max The maximum value of the input range.
 * @param output_min The minimum value of the output range.
 * @param output_max The maximum value of the output range.
 * @return The interpolated output value.
 */
float linear_interpolation(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    return ((output_max - output_min) / (input_max - input_min)) * (input_value - input_min) + output_min;
}

/**
 * Performs a quadratic interpolation, supposing input_max == -input_min and output_max == -output_min,
 * i.e. the interpolation is centered on the origin (0,0)
 *
 * @param input_value The input value to be interpolated.
 * @param input_max The maximum input value, it MUST equal -input_min.
 * @param output_max The maximum output value, it MUST equal -output_min.
 * @return The interpolated output value.
 */
float symmetric_quadratic_interpolation(float input_value, float input_max, float output_max)
{
    double a;
    if(input_value > 0)
        a = (output_max / input_max) / input_max ;
    else
        a = -(output_max / input_max) / input_max;

    return (a * input_value) * input_value;
}

/**
 * Normalizes the values in the input array to the range [-1, 1] by dividing each value by the maximum absolute value in the array;
 * IF AND ONLY IF THE MAXIMUM OF THE ABSOLUTE VALUES IN THE ARRAY IS GREATER THAN ONE
 *
 * @param input_array The input array of floating-point values to be normalized.
 * @param output_array The output array where the normalized values will be stored.
 * @param size The number of elements in the input and output arrays.
 */
void normalize_vector(const float *input_array, float *output_array, uint8_t size)
{
    float max_abs_value = 0.0f;
    for (uint8_t i = 0; i < size; i++)
    {
        float abs_value = fabsf(input_array[i]);
        if (abs_value > max_abs_value)
        {
            max_abs_value = abs_value;
        }
    }

	if (max_abs_value > 1)
	{
		for (uint8_t i = 0; i < size; i++)
		{
			output_array[i] = input_array[i] / max_abs_value;
		}
	}
}



