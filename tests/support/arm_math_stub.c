/**
 * @file arm_math_stub.c
 * @brief Hand-written host stand-in implementing arm_math.h (D-06).
 *
 * No globals, no I/O; every function operates only on its parameters
 * and caller-supplied outputs, mirroring interpolations.c's pure-math
 * body style (Core/Src/interpolations.c).
 *
 * @author PoliTOcean
 * @date Sep 6, 2026
 */
#include "arm_math.h"

/**
 * @brief Derives the three PID coefficients from the gains.
 *
 * Coefficient derivation reproduced from the in-tree vendored header
 * Middlewares/ST/ARM/DSP/Inc/arm_math.h (CMSIS-DSP v1.7.0,
 * Apache-2.0, Copyright (c) 2010-2019 Arm Limited), whose
 * arm_pid_instance_f32 field comments state A0 = Kp + Ki + Kd,
 * A1 = -Kp - 2Kd, A2 = Kd. Cross-checked against this firmware's own
 * independent derivation in update_pid_constants()
 * (Core/Src/freertos.cpp:376-384), which computes the identical
 * three coefficients from the identical three gains -- a divergence
 * between the two would mean the host and the target run different
 * controllers.
 *
 * @param[in,out] S              Instance to initialize.
 * @param[in]     resetStateFlag Non-zero zeroes all three state
 *                                elements; zero recomputes only the
 *                                coefficients.
 * @return None.
 */
void arm_pid_init_f32(arm_pid_instance_f32 * S, int32_t resetStateFlag)
{
    S->A0 = S->Kp + S->Ki + S->Kd;
    S->A1 = -S->Kp - 2.0f * S->Kd;
    S->A2 = S->Kd;

    if (resetStateFlag != 0) {
        S->state[0] = 0.0f;
        S->state[1] = 0.0f;
        S->state[2] = 0.0f;
    }
}

/**
 * @brief Single-step floating-point PID process function.
 *
 * Body reproduced from the in-tree vendored header
 * Middlewares/ST/ARM/DSP/Inc/arm_math.h:5891-5909 (CMSIS-DSP v1.7.0,
 * Apache-2.0, Copyright (c) 2010-2019 Arm Limited). That body is a
 * __STATIC_FORCEINLINE function -- its entire implementation is
 * vendored in-tree as header-inline C, not a symbol linked out of a
 * prebuilt library -- so this stand-in reproduces the exact
 * difference equation the target would otherwise execute, dropping
 * only the force-inline attribute (the body uses no CMSIS-specific
 * intrinsic):
 *
 *   y[n] = y[n-1] + A0*x[n] + A1*x[n-1] + A2*x[n-2]
 *
 * @param[in,out] S  Instance of the PID control structure.
 * @param[in]     in Input sample to process.
 * @return The processed output sample.
 */
float32_t arm_pid_f32(arm_pid_instance_f32 * S, float32_t in)
{
    float32_t out;

    out = (S->A0 * in) +
        (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    return out;
}

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
    float32_t * pData)
{
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData = pData;
}

/**
 * @brief Floating-point matrix multiplication: pDst = pSrcA * pSrcB.
 *
 * Written from scratch: arm_mat_mult_f32 is declared but not defined
 * in the vendored header -- its body exists only inside the prebuilt
 * ARM archive Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a,
 * which cannot link on this host, so no portable C source exists in
 * this repository to reproduce.
 *
 * Validates the documented size contract first: pSrcA's column count
 * must equal pSrcB's row count, and pDst's dimensions must equal the
 * product's (pSrcA's row count by pSrcB's column count). On any
 * mismatch, returns ARM_MATH_SIZE_MISMATCH and writes nothing to
 * pDst.
 *
 * Accumulation order is fixed and documented here rather than left
 * to the compiler: destination rows outermost, destination columns
 * next, the shared inner index innermost, accumulating into a single
 * local before storing. Floating-point addition is not associative,
 * so an unspecified order would make results irreproducible in the
 * last bits between runs or compilers. This stand-in's order is
 * reproducible with itself; it cannot be claimed bit-identical to
 * the ARM library's own accumulation order, which exists only inside
 * a prebuilt binary and is therefore unknowable here.
 *
 * @param[in]  pSrcA Points to the first input matrix structure.
 * @param[in]  pSrcB Points to the second input matrix structure.
 * @param[out] pDst  Points to the output matrix structure.
 * @return ARM_MATH_SIZE_MISMATCH or ARM_MATH_SUCCESS.
 */
arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 * pSrcA,
    const arm_matrix_instance_f32 * pSrcB,
    arm_matrix_instance_f32 * pDst)
{
    uint16_t i;
    uint16_t j;
    uint16_t k;
    uint16_t inner;

    if (pSrcA->numCols != pSrcB->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (pDst->numRows != pSrcA->numRows) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (pDst->numCols != pSrcB->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }

    inner = pSrcA->numCols;

    for (i = 0; i < pDst->numRows; i++) {
        for (j = 0; j < pDst->numCols; j++) {
            float32_t sum = 0.0f;
            for (k = 0; k < inner; k++) {
                sum += pSrcA->pData[i * pSrcA->numCols + k] *
                    pSrcB->pData[k * pSrcB->numCols + j];
            }
            pDst->pData[i * pDst->numCols + j] = sum;
        }
    }

    return ARM_MATH_SUCCESS;
}
