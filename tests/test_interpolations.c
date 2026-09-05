/**
 * @file test_interpolations.c
 * @brief Host-based Ceedling/Unity tests for the joystick command
 * conversion functions declared in interpolations.h (TEST-04).
 *
 * @author PoliTOcean
 * @date Sep 5, 2026
 */
#include "unity.h"
#include "interpolations.h"

void setUp(void)
{
}

void tearDown(void)
{
}

/**
 * linear_interpolation must map the midpoint of its input range to the
 * midpoint of its output range. Tolerance is 1e-4: the output range used
 * here spans hundreds of PWM counts, so 1e-4 is far tighter than any
 * meaningful PWM resolution while still absorbing float rounding noise
 * from the interpolation arithmetic itself.
 */
void test_linear_interpolation_maps_input_midpoint_to_output_midpoint(void)
{
    float result = linear_interpolation(0.0f, -1.0f, 1.0f, 1000.0f, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1500.0f, result);
}
