/**
 * @file arm_math.h
 * @brief Hand-written host stand-in for CMSIS-DSP's arm_math.h (D-06).
 *
 * This is NOT a vendored copy of Middlewares/ST/ARM/DSP/Inc/arm_math.h.
 * It is a from-scratch reproduction, hand-written for the host test
 * build, of exactly the seven symbols this firmware's control math
 * actually uses: the arm_status enum, the arm_pid_instance_f32 and
 * arm_matrix_instance_f32 struct layouts, and the arm_pid_init_f32,
 * arm_pid_f32, arm_mat_init_f32 and arm_mat_mult_f32 functions.
 *
 * The struct layouts and enum values below are reproduced from the
 * in-tree Middlewares/ST/ARM/DSP/Inc/arm_math.h, CMSIS-DSP v1.7.0,
 * Apache-2.0 (Copyright (c) 2010-2019 Arm Limited), so that this
 * header's types stay drop-in compatible with the vendored one. No
 * function body, no comment and no line of code was copied out of
 * that file or out of the prebuilt ARM archive
 * Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a -- every
 * implementation is written independently against the documented
 * CMSIS-DSP contract.
 *
 * This substitution is a recorded decision (D-06,
 * .planning/phases/02-firmware-test-harness/02-CONTEXT.md), not an
 * accident: TEST-02 forbids vendoring CMSIS-DSP source for the host
 * build, and the matrix routines have no portable C source in this
 * repository to vendor even if it did not.
 *
 * @author PoliTOcean
 * @date Sep 6, 2026
 */
#ifndef TESTS_SUPPORT_ARM_MATH_H_
#define TESTS_SUPPORT_ARM_MATH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

/** @brief Single-precision floating-point type, aliasing float. */
typedef float float32_t;

/**
 * @brief Error status returned by the stand-in functions below.
 *
 * All seven enumerators are declared with their exact vendored values
 * so this enum is a drop-in, type-compatible replacement rather than
 * a subset: a call site returning an undeclared status would
 * otherwise still type-check while meaning something different.
 */
typedef enum
{
    ARM_MATH_SUCCESS        =  0, /**< No error. */
    ARM_MATH_ARGUMENT_ERROR = -1, /**< One or more arguments incorrect. */
    ARM_MATH_LENGTH_ERROR   = -2, /**< Length of data buffer incorrect. */
    ARM_MATH_SIZE_MISMATCH  = -3, /**< Matrix sizes incompatible. */
    ARM_MATH_NANINF         = -4, /**< NaN or infinity generated. */
    ARM_MATH_SINGULAR       = -5, /**< Input matrix is singular. */
    ARM_MATH_TEST_FAILURE   = -6  /**< Test failed. */
} arm_status;

/**
 * @brief Instance structure for the floating-point PID control.
 *
 * Member order matches the vendored arm_pid_instance_f32 exactly:
 * A0, A1, A2, state[3], Kp, Ki, Kd.
 */
typedef struct
{
    float32_t A0;       /**< Derived gain, A0 = Kp + Ki + Kd. */
    float32_t A1;       /**< Derived gain, A1 = -(Kp + 2Kd). */
    float32_t A2;       /**< Derived gain, A2 = Kd. */
    float32_t state[3]; /**< State array of length 3. */
    float32_t Kp;       /**< Proportional gain. */
    float32_t Ki;       /**< Integral gain. */
    float32_t Kd;       /**< Derivative gain. */
} arm_pid_instance_f32;

/**
 * @brief Instance structure for the floating-point matrix.
 *
 * Member order matches the vendored arm_matrix_instance_f32 exactly:
 * numRows, numCols, pData.
 */
typedef struct
{
    uint16_t numRows;  /**< Number of rows of the matrix. */
    uint16_t numCols;  /**< Number of columns of the matrix. */
    float32_t *pData;  /**< Points to the matrix's backing data. */
} arm_matrix_instance_f32;

/**
 * @brief Initializes/derives the PID gains and optionally resets state.
 *
 * @param[in,out] S              Instance to initialize.
 * @param[in]     resetStateFlag Non-zero zeroes all three state
 *                                elements; zero recomputes only the
 *                                coefficients, leaving state alone.
 * @return None.
 */
void arm_pid_init_f32(arm_pid_instance_f32 * S, int32_t resetStateFlag);

/**
 * @brief Single-step floating-point PID process function.
 *
 * @param[in,out] S  Instance of the PID control structure.
 * @param[in]     in Input sample to process.
 * @return The processed output sample.
 */
float32_t arm_pid_f32(arm_pid_instance_f32 * S, float32_t in);

/**
 * @brief Floating-point matrix initialization.
 *
 * @param[in,out] S     Instance to initialize.
 * @param[in]     nRows Number of rows in the matrix.
 * @param[in]     nCols Number of columns in the matrix.
 * @param[in]     pData Points to the matrix's backing data array.
 * @return None.
 */
void arm_mat_init_f32(
    arm_matrix_instance_f32 * S,
    uint16_t nRows,
    uint16_t nCols,
    float32_t * pData);

/**
 * @brief Floating-point matrix multiplication: pDst = pSrcA * pSrcB.
 *
 * @param[in]  pSrcA Points to the first input matrix structure.
 * @param[in]  pSrcB Points to the second input matrix structure.
 * @param[out] pDst  Points to the output matrix structure.
 * @return ARM_MATH_SIZE_MISMATCH if pSrcA's column count does not
 *         match pSrcB's row count, or if pDst's dimensions do not
 *         match the product's; ARM_MATH_SUCCESS otherwise.
 */
arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 * pSrcA,
    const arm_matrix_instance_f32 * pSrcB,
    arm_matrix_instance_f32 * pDst);

#ifdef __cplusplus
}
#endif
#endif /* TESTS_SUPPORT_ARM_MATH_H_ */
