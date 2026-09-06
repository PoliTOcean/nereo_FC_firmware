/**
 * @file thruster_safe_state.c
 * @brief Implements the single source of truth for the fault-safe thruster
 * stop declared in thruster_safe_state.h.
 *
 * @author PoliTOcean
 * @date Sep 5, 2026
 */
#include "main.h"
#include "safety/thruster_safe_state.h"

/*
 * Neutral pulse width, translation-unit-local. The authoritative source for
 * this numeric value is navigation/navigation.h's PwmValues enum
 * (PWM_IDLE = 1500). Defined here as a private macro -- never in the header
 * -- because including navigation.h would pull in arm_math.h (CMSIS-DSP)
 * and violate this module's dependency-free requirement (D-18), and because
 * exporting this macro from the header would collide with navigation.h's
 * own PWM_IDLE enumerator wherever both headers are included together
 * (e.g. freertos.cpp, which reaches navigation.h via FC_app.h) -- the
 * preprocessor would rewrite the enum's own definition line into a syntax
 * error. Keeping the macro private to this one .c file avoids that
 * collision entirely.
 */
#define PWM_IDLE 1500

/**
 * Forces all eight thruster PWM channels to trimmed neutral. See
 * thruster_safe_state.h for the full FAULT CONTRACT (SAFE-02) rationale.
 *
 * The eight CCR writes below are copied verbatim, in the same order
 * (TIM2 CCR1-4, then TIM3 CCR1-4), from the trim table previously defined
 * in freertos.cpp's set_pwm_idle() -- this is the single authoritative copy
 * of the per-motor trim offsets (D-02, D-18).
 */
void thruster_force_neutral(void)
{
	TIM2 -> CCR1 = PWM_IDLE - 25;  // Motor 2
	TIM2 -> CCR2 = PWM_IDLE + 46;  // Motor 6
	TIM2 -> CCR3 = PWM_IDLE + 46;  // Motor 5
	TIM2 -> CCR4 = PWM_IDLE + 46;  // Motor 1
	TIM3 -> CCR1 = PWM_IDLE + 44;  // Motor 4
	TIM3 -> CCR2 = PWM_IDLE + 47;  // Motor 8
	TIM3 -> CCR3 = PWM_IDLE + 47;  // Motor 7
	TIM3 -> CCR4 = PWM_IDLE + 42;  // Motor 3
}
