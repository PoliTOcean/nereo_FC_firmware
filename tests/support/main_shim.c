/**
 * @file main_shim.c
 * @brief Backing storage for the tests/support/main.h shim's TIM instances.
 *
 * Hand-written test infrastructure (D-04): defines the two TIM_TypeDef
 * objects declared extern in main.h. This is the observable memory the
 * fault-injection test reads back after Core/Src/safety/
 * thruster_safe_state.c's direct register writes.
 *
 * @author PoliTOcean
 * @date Sep 6, 2026
 */
#include "main.h"

TIM_TypeDef TIM2_Instance;
TIM_TypeDef TIM3_Instance;
