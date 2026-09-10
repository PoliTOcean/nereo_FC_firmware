/**
 * @file test_arm_pid_stand_in.c
 * @brief Pins tests/support/arm_math.h's host stand-in to the semantics
 *        of the vendored CMSIS-DSP implementation.
 *
 * Every expected value below is hand-computed from the documented
 * difference equation and its coefficient derivation, reproduced from
 * the in-tree Middlewares/ST/ARM/DSP/Inc/arm_math.h (CMSIS-DSP
 * v1.7.0, Apache-2.0, Copyright (c) 2010-2019 Arm Limited). No
 * expected value here was obtained by running the stand-in: a test
 * whose expectations come from the implementation under test cannot
 * detect that implementation being wrong, which is exactly the
 * failure this file exists to catch.
 *
 * @author Davide Colabella
 * @date Sep 6, 2026
 */
#include "unity.h"
#include "arm_math.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_arm_pid_init_f32_derives_coefficients_and_resets_state(void)
{
    arm_pid_instance_f32 pid;

    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.25f;
    pid.state[0] = 9.0f;
    pid.state[1] = 9.0f;
    pid.state[2] = 9.0f;

    arm_pid_init_f32(&pid, 1);

    /* A0 = Kp+Ki+Kd, A1 = -Kp-2Kd, A2 = Kd (arm_math.h:2773-2782,
     * cross-checked against freertos.cpp:381-383's independent
     * derivation). Small dimensionless gain quantities: tight
     * tolerance so a wrong coefficient factor cannot hide. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, pid.A0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.5f, pid.A1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f, pid.A2);

    /* Non-zero reset flag zeroes all three state elements. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.state[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.state[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.state[2]);
}

void test_arm_pid_f32_first_call_returns_hand_computed_output(void)
{
    arm_pid_instance_f32 pid;

    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.25f;
    arm_pid_init_f32(&pid, 1);

    /* y0 = A0*1.0 + A1*0 + A2*0 + 0 = 1.75*1.0 = 1.75, state having
     * just been zeroed by the reset above. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, arm_pid_f32(&pid, 1.0f));
}

void test_arm_pid_f32_sequence_returns_hand_computed_outputs(void)
{
    arm_pid_instance_f32 pid;
    float32_t out1;
    float32_t out2;
    float32_t out3;

    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.25f;
    arm_pid_init_f32(&pid, 1);

    out1 = arm_pid_f32(&pid, 1.0f);
    out2 = arm_pid_f32(&pid, 2.0f);
    out3 = arm_pid_f32(&pid, 0.0f);

    /* Hand-computed by carrying each call's state update forward
     * through the difference equation (see arm_math_stub.c's comment
     * above arm_pid_f32):
     *   y0 = 1.75*1               = 1.75
     *   y1 = 1.75*2 -1.5*1 +1.75  = 3.75
     *   y2 = 1.75*0 -1.5*2 +0.25*1 +3.75 = 1.00
     */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, out1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.75f, out2);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, out3);
}

void test_arm_pid_f32_state_update_order_is_asserted(void)
{
    arm_pid_instance_f32 pid;

    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.25f;
    arm_pid_init_f32(&pid, 1);

    (void)arm_pid_f32(&pid, 1.0f);
    (void)arm_pid_f32(&pid, 2.0f);
    (void)arm_pid_f32(&pid, 0.0f);

    /* After the third call: state[0] holds the most recent input
     * (0.0), state[1] holds the input before that (2.0), and
     * state[2] holds the last output (1.0f, hand-computed above).
     * The update order itself is under test, not just the return
     * value. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid.state[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, pid.state[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, pid.state[2]);
}

void test_arm_pid_init_f32_zero_reset_flag_preserves_state(void)
{
    arm_pid_instance_f32 pid;

    pid.Kp = 1.0f;
    pid.Ki = 0.5f;
    pid.Kd = 0.25f;
    pid.state[0] = 5.0f;
    pid.state[1] = 6.0f;
    pid.state[2] = 7.0f;

    arm_pid_init_f32(&pid, 0);

    /* Coefficients are still recomputed even with resetStateFlag == 0. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.75f, pid.A0);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -1.5f, pid.A1);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.25f, pid.A2);

    /* Existing state elements must be left untouched. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5.0f, pid.state[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 6.0f, pid.state[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 7.0f, pid.state[2]);
}

void test_arm_pid_f32_zero_gains_yield_zero_output_always(void)
{
    arm_pid_instance_f32 pid;

    pid.Kp = 0.0f;
    pid.Ki = 0.0f;
    pid.Kd = 0.0f;
    arm_pid_init_f32(&pid, 1);

    /* A0 = A1 = A2 = 0, so every term vanishes except state[2],
     * which is itself the (always-zero) prior output -- the
     * degenerate case does not drift regardless of input. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, arm_pid_f32(&pid, 1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, arm_pid_f32(&pid, -5.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, arm_pid_f32(&pid, 1000.0f));
}

void test_arm_mat_init_f32_assigns_dimensions_and_data_pointer(void)
{
    float32_t data[6];
    arm_matrix_instance_f32 mat;

    arm_mat_init_f32(&mat, 8, 6, data);

    TEST_ASSERT_EQUAL_UINT16(8, mat.numRows);
    TEST_ASSERT_EQUAL_UINT16(6, mat.numCols);
    TEST_ASSERT_EQUAL_PTR(data, mat.pData);
}

void test_arm_mat_mult_f32_produces_hand_computed_product(void)
{
    /* A[i][j] = 10*i + j, row-major, so rows are distinguishable and
     * the selecting vector below picks out column index 2 exactly,
     * mirroring navigation.c's calculate_pwm() 8x6 by 6x1 shape. */
    float32_t a_data[48] = {
         0,  1,  2,  3,  4,  5,
        10, 11, 12, 13, 14, 15,
        20, 21, 22, 23, 24, 25,
        30, 31, 32, 33, 34, 35,
        40, 41, 42, 43, 44, 45,
        50, 51, 52, 53, 54, 55,
        60, 61, 62, 63, 64, 65,
        70, 71, 72, 73, 74, 75,
    };
    float32_t b_data[6] = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    float32_t dst_data[8] = { 0 };
    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 b;
    arm_matrix_instance_f32 dst;
    arm_status status;

    arm_mat_init_f32(&a, 8, 6, a_data);
    arm_mat_init_f32(&b, 6, 1, b_data);
    arm_mat_init_f32(&dst, 8, 1, dst_data);

    status = arm_mat_mult_f32(&a, &b, &dst);

    TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
    /* b selects column 2 of a, so output[i] = a[i][2] = 10*i + 2,
     * hand-computed, never run through the stand-in to produce. */
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, dst_data[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 12.0f, dst_data[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 22.0f, dst_data[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 32.0f, dst_data[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 42.0f, dst_data[4]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 52.0f, dst_data[5]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 62.0f, dst_data[6]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 72.0f, dst_data[7]);
}

void test_arm_mat_mult_f32_identity_matrix_returns_original_operand(void)
{
    float32_t identity_data[36] = {
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1,
    };
    float32_t v_data[6] = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };
    float32_t dst_data[6] = { 0 };
    arm_matrix_instance_f32 identity;
    arm_matrix_instance_f32 v;
    arm_matrix_instance_f32 dst;
    arm_status status;

    arm_mat_init_f32(&identity, 6, 6, identity_data);
    arm_mat_init_f32(&v, 6, 1, v_data);
    arm_mat_init_f32(&dst, 6, 1, dst_data);

    status = arm_mat_mult_f32(&identity, &v, &dst);

    TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, dst_data[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, dst_data[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, dst_data[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 4.0f, dst_data[3]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 5.0f, dst_data[4]);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 6.0f, dst_data[5]);
}

void test_arm_mat_mult_f32_col_row_mismatch_returns_size_mismatch(void)
{
    /* pSrcA is 2x3 (numCols=3); pSrcB is deliberately 2x2
     * (numRows=2), so pSrcA->numCols != pSrcB->numRows -- the
     * documented mismatch condition, not a bug in the operand data
     * itself. */
    float32_t a_data[6] = { 1, 2, 3, 4, 5, 6 };
    float32_t b_data[4] = { 1, 2, 3, 4 };
    float32_t dst_data[4];
    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 b;
    arm_matrix_instance_f32 dst;
    arm_status status;
    uint8_t i;

    for (i = 0; i < 4; i++) {
        /* Poison sentinel: not a value any valid product of the
         * small operands above could produce, so "still the
         * sentinel" proves the destination was never written. */
        dst_data[i] = -12345.0f;
    }

    arm_mat_init_f32(&a, 2, 3, a_data);
    arm_mat_init_f32(&b, 2, 2, b_data);
    arm_mat_init_f32(&dst, 2, 2, dst_data);

    status = arm_mat_mult_f32(&a, &b, &dst);

    TEST_ASSERT_EQUAL_INT(ARM_MATH_SIZE_MISMATCH, status);
    for (i = 0; i < 4; i++) {
        /* Zero tolerance: the claim under test is that the sentinel
         * is bit-identical to what was written before the call, not
         * merely close to it. */
        TEST_ASSERT_FLOAT_WITHIN(0.0f, -12345.0f, dst_data[i]);
    }
}

void test_arm_mat_mult_f32_dst_dimension_mismatch_returns_size_mismatch(void)
{
    /* pSrcA (2x3) * pSrcB (3x2) has a valid 2x2 product, but pDst is
     * deliberately declared 3x2 -- pDst->numRows != pSrcA->numRows. */
    float32_t a_data[6] = { 1, 2, 3, 4, 5, 6 };
    float32_t b_data[6] = { 1, 2, 3, 4, 5, 6 };
    float32_t dst_data[6];
    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 b;
    arm_matrix_instance_f32 dst;
    arm_status status;
    uint8_t i;

    for (i = 0; i < 6; i++) {
        dst_data[i] = -12345.0f;
    }

    arm_mat_init_f32(&a, 2, 3, a_data);
    arm_mat_init_f32(&b, 3, 2, b_data);
    arm_mat_init_f32(&dst, 3, 2, dst_data);

    status = arm_mat_mult_f32(&a, &b, &dst);

    TEST_ASSERT_EQUAL_INT(ARM_MATH_SIZE_MISMATCH, status);
    for (i = 0; i < 6; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.0f, -12345.0f, dst_data[i]);
    }
}

void test_arm_mat_mult_f32_same_multiply_twice_is_reproducible(void)
{
    float32_t a_data[48] = {
         0,  1,  2,  3,  4,  5,
        10, 11, 12, 13, 14, 15,
        20, 21, 22, 23, 24, 25,
        30, 31, 32, 33, 34, 35,
        40, 41, 42, 43, 44, 45,
        50, 51, 52, 53, 54, 55,
        60, 61, 62, 63, 64, 65,
        70, 71, 72, 73, 74, 75,
    };
    float32_t b_data[6] = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    float32_t dst1_data[8] = { 0 };
    float32_t dst2_data[8] = { 0 };
    arm_matrix_instance_f32 a;
    arm_matrix_instance_f32 b;
    arm_matrix_instance_f32 dst1;
    arm_matrix_instance_f32 dst2;
    uint8_t i;

    arm_mat_init_f32(&a, 8, 6, a_data);
    arm_mat_init_f32(&b, 6, 1, b_data);
    arm_mat_init_f32(&dst1, 8, 1, dst1_data);
    arm_mat_init_f32(&dst2, 8, 1, dst2_data);

    (void)arm_mat_mult_f32(&a, &b, &dst1);
    (void)arm_mat_mult_f32(&a, &b, &dst2);

    /* Zero tolerance is correct here: the claim under test is
     * bit-identical reproducibility of the fixed accumulation order
     * across two independent runs over identical inputs, not merely
     * numerical closeness. Do not widen this tolerance. */
    for (i = 0; i < 8; i++) {
        TEST_ASSERT_FLOAT_WITHIN(0.0f, dst1_data[i], dst2_data[i]);
    }
}
