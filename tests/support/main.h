/**
 * @file main.h
 * @brief Hand-written host test shim for the real Core/Inc/main.h.
 *
 * This file is hand-written test infrastructure (D-04): no content is
 * derived from, or copied out of, any vendor HAL or CMSIS file. It exists
 * purely so that Core/Src/safety/thruster_safe_state.c compiles and runs on
 * the host with no change to the firmware source (D-05).
 *
 * It provides exactly the surface thruster_safe_state.c needs and nothing
 * more: a TIM_TypeDef-shaped struct with the four capture/compare register
 * members that function writes, and the two timer instances (TIM2, TIM3)
 * it writes them through.
 *
 * The CCR member width is uint32_t deliberately -- the target's real
 * capture/compare registers are 32 bits wide, so a value that would
 * truncate or wrap on the STM32 truncates or wraps identically here. A
 * narrower or wider field would make the host observation disagree with
 * the target for out-of-range values.
 *
 * This header deliberately does NOT define any neutral-pulse-width macro,
 * under any name. Core/Src/safety/thruster_safe_state.c keeps that constant
 * translation-unit-local precisely because navigation/navigation.h declares
 * an enumerator of the same name -- a macro visible in both translation
 * units would rewrite the enum's own definition line into a syntax error.
 * Expected neutral values belong in the test file as literals, not in this
 * shim.
 *
 * @author PoliTOcean
 * @date Sep 6, 2026
 */
#ifndef TESTS_SUPPORT_MAIN_H_
#define TESTS_SUPPORT_MAIN_H_

#include <stdint.h>

/**
 * @brief Shim stand-in for the real STM32 TIM_TypeDef.
 *
 * Only the four capture/compare registers thruster_safe_state.c writes are
 * modelled; there is no unused surface to diverge from the real register
 * map.
 */
typedef struct {
	uint32_t CCR1;
	uint32_t CCR2;
	uint32_t CCR3;
	uint32_t CCR4;
} TIM_TypeDef;

extern TIM_TypeDef TIM2_Instance;
extern TIM_TypeDef TIM3_Instance;

#define TIM2 (&TIM2_Instance)
#define TIM3 (&TIM3_Instance)

#endif /* TESTS_SUPPORT_MAIN_H_ */
