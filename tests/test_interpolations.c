/**
 * @file test_interpolations.c
 * @brief Host-based Ceedling/Unity tests for the joystick command
 * conversion functions declared in interpolations.h (TEST-04).
 *
 * @author Davide Colabella
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

/**
 * At input_value == input_min the function must return exactly output_min
 * (the low endpoint of the documented contract). Tolerance 1e-3: PWM-scale
 * output (thousands of counts), far tighter than a single PWM count, only
 * absorbing float arithmetic noise.
 */
void test_linear_interpolation_at_input_min_returns_output_min(void)
{
    float result = linear_interpolation(-1.0f, -1.0f, 1.0f, 1000.0f, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, result);
}

/**
 * At input_value == input_max the function must return exactly output_max
 * (the high endpoint). Same PWM-scale tolerance rationale as the input_min
 * case above.
 */
void test_linear_interpolation_at_input_max_returns_output_max(void)
{
    float result = linear_interpolation(1.0f, -1.0f, 1.0f, 1000.0f, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2000.0f, result);
}

/**
 * One step inside the low end of the input range must land strictly
 * between output_min and the output midpoint. The exact expected value is
 * hand-computed from the documented linear formula, so this both proves
 * strict betweenness (1050 is between 1000 and 1500) and pins the formula
 * itself. Tolerance 1e-3: same PWM-scale rationale as above.
 */
void test_linear_interpolation_one_step_inside_min_end(void)
{
    float result = linear_interpolation(-0.9f, -1.0f, 1.0f, 1000.0f,
                                         2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1050.0f, result);
}

/**
 * One step inside the high end of the input range must land strictly
 * between the output midpoint and output_max. Expected value hand-computed
 * the same way as the min-end case above.
 */
void test_linear_interpolation_one_step_inside_max_end(void)
{
    float result = linear_interpolation(0.9f, -1.0f, 1.0f, 1000.0f, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1950.0f, result);
}

/**
 * Outside the declared input range the function extrapolates linearly --
 * it does not clamp. This is the existing, documented contract (see
 * interpolations.h), pinned here rather than assumed. input_value == 2.0f
 * is one full range-width past input_max == 1.0f, so the extrapolated
 * result (2500) lies past output_max == 2000f, proving no clamping
 * occurred. Tolerance 1e-3: same PWM-scale rationale as above.
 */
void test_linear_interpolation_extrapolates_outside_input_range(void)
{
    float result = linear_interpolation(2.0f, -1.0f, 1.0f, 1000.0f, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2500.0f, result);
}

/**
 * A descending output range (output_min > output_max) must produce a
 * descending mapping: the low input endpoint maps to the now-larger
 * output_min value, and the high input endpoint maps to the now-smaller
 * output_max value -- the two endpoint mappings are swapped relative to
 * the ascending case above. Tolerance 1e-3: same PWM-scale rationale.
 */
void test_linear_interpolation_descending_output_range(void)
{
    float at_input_min = linear_interpolation(-1.0f, -1.0f, 1.0f, 2000.0f,
                                               1000.0f);
    float at_input_max = linear_interpolation(1.0f, -1.0f, 1.0f, 2000.0f,
                                               1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2000.0f, at_input_min);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1000.0f, at_input_max);
}

/**
 * At input_value == 0 the negative-coefficient branch is taken (the
 * condition is a strict input_value > 0), yet the result must still be
 * exactly 0 -- the point where both branches of the sign selection meet.
 * Tolerance 1e-4: output scale here is ~100 (a representative joystick
 * axis scaled to a PWM-adjacent range), so 1e-4 is far tighter than any
 * meaningful resolution at that scale.
 */
void test_symmetric_quadratic_interpolation_at_zero_returns_zero(void)
{
    float result = symmetric_quadratic_interpolation(0.0f, 10.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, result);
}

/**
 * At input_value == input_max (positive branch) the function must return
 * exactly output_max. Same output-scale tolerance rationale as above.
 */
void test_symmetric_quadratic_interpolation_at_positive_max(void)
{
    float result = symmetric_quadratic_interpolation(10.0f, 10.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 100.0f, result);
}

/**
 * At input_value == -input_max (negative branch) the function must return
 * exactly -output_max -- the two branches are symmetric about the origin.
 * Same output-scale tolerance rationale as above.
 */
void test_symmetric_quadratic_interpolation_at_negative_max(void)
{
    float result = symmetric_quadratic_interpolation(-10.0f, 10.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -100.0f, result);
}

/**
 * At half of input_max the result must be one quarter of output_max --
 * the quadratic shape (proportional to input_value squared), not a
 * linear one, which would instead give half of output_max. Same
 * output-scale tolerance rationale as above.
 */
void test_symmetric_quadratic_interpolation_half_max_is_quarter_output(void)
{
    float result = symmetric_quadratic_interpolation(5.0f, 10.0f, 100.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 25.0f, result);
}

/**
 * With size == 0, normalize_vector must write nothing to the output array
 * and read nothing from the input array. The output array is poisoned
 * with a recognisable sentinel before the call so a buggy implementation
 * that zero-fills the output would be caught -- an already-zero output
 * would let such a bug pass vacuously. Tolerance 1e-6: this is an exact
 * untouched-memory check, not a computed value, so the tolerance only
 * absorbs the possibility of the sentinel itself being re-stored bit-for
 * -bit (no arithmetic occurs on this path at all).
 */
void test_normalize_vector_size_zero_leaves_output_untouched(void)
{
    const float input[3] = {5.0f, -5.0f, 3.0f};
    float output[3] = {1234.5f, 1234.5f, 1234.5f};

    normalize_vector(input, output, 0);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1234.5f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1234.5f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1234.5f, output[2]);
}

/**
 * When every element's absolute value is at or below 1, the output must
 * be an unchanged copy of the input -- no division applied. Tolerance
 * 1e-5: these are normalized joystick-axis-scale values in [-1, 1], which
 * warrants a much tighter tolerance than the PWM-scale interpolation
 * tests above.
 */
void test_normalize_vector_below_one_copies_unchanged(void)
{
    const float input[3] = {0.5f, -1.0f, 0.3f};
    float output[3] = {0.0f, 0.0f, 0.0f};

    normalize_vector(input, output, 3);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, output[2]);
}

/**
 * The maximum absolute value being exactly 1.0 is on the copy-through
 * side of the boundary: the implementation's guard is a strict
 * (max_abs_value > 1) comparison, so exactly 1.0 must NOT be divided.
 * This is the documented contract, not an oversight. Tolerance 1e-5:
 * same normalized-axis-scale rationale as above.
 */
void test_normalize_vector_exactly_one_is_not_divided(void)
{
    const float input[3] = {1.0f, -0.4f, 0.2f};
    float output[3] = {0.0f, 0.0f, 0.0f};

    normalize_vector(input, output, 3);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.4f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, output[2]);
}

/**
 * When the maximum absolute value exceeds 1, every element must be
 * divided by that maximum, so the resulting maximum absolute value is
 * 1.0 -- and each element keeps its original index and sign. Tolerance
 * 1e-5: same normalized-axis-scale rationale as above; the divisor here
 * (4.0) divides evenly into each input value with no rounding residue.
 */
void test_normalize_vector_above_one_divides_preserving_index_and_sign(void)
{
    const float input[3] = {2.0f, -4.0f, 1.0f};
    float output[3] = {0.0f, 0.0f, 0.0f};

    normalize_vector(input, output, 3);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.25f, output[2]);
}

/**
 * Two elements of equal absolute value (one the maximum, opposite signs),
 * plus a distinct third element, must not be reordered or merged by the
 * tie -- each output index is asserted individually against its own
 * expected value rather than an aggregate property. Tolerance 1e-5: same
 * normalized-axis-scale rationale as above.
 */
void test_normalize_vector_equal_abs_tie_preserves_order(void)
{
    const float input[3] = {3.0f, -3.0f, 1.0f};
    float output[3] = {0.0f, 0.0f, 0.0f};

    normalize_vector(input, output, 3);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, output[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, output[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, (1.0f / 3.0f), output[2]);
}
