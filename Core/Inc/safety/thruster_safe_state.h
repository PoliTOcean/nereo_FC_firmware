/**
 * @file thruster_safe_state.h
 * @brief Single source of truth for the fault-safe thruster stop (SAFE-01/02).
 *
 * FAULT CONTRACT (SAFE-02): this vehicle fails safe, it does not attempt
 * recovery (D-05). thruster_force_neutral() is called from Error_Handler(),
 * from every Cortex-M hardware fault ISR (NMI_Handler, HardFault_Handler,
 * MemManage_Handler, BusFault_Handler, UsageFault_Handler) and from
 * FreeRTOS's configASSERT -- six paths total (D-17) -- as the first action,
 * before interrupts disable and before each handler's infinite loop. None
 * of the six refresh the independent watchdog (D-06): a persistent fault
 * resets the MCU via the IWDG rather than holding a faulted-but-not-reset
 * state.
 *
 * The safe state itself is a held trimmed-neutral PWM pulse via direct CCR
 * register writes, not a stopped timer (D-04): this mirrors the vehicle's
 * proven disarm and AGENT_DISCONNECTED safe-state convention, since
 * behaviour with no PWM pulse at all is ESC-firmware-dependent and
 * unverified on this hardware.
 *
 * @author PoliTOcean
 * @date Sep 5, 2026
 */
#ifndef CORE_INC_SAFETY_THRUSTER_SAFE_STATE_H_
#define CORE_INC_SAFETY_THRUSTER_SAFE_STATE_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Forces all eight thruster PWM channels to trimmed neutral.
 *
 * Writes the same eight per-motor trimmed-neutral CCR values used by
 * set_pwm_idle() directly to TIM2/TIM3, unconditionally, regardless of any
 * command write that may have been in progress at fault entry. Performs
 * only direct register stores -- no HAL calls, no RTOS services -- so it is
 * safe to call with interrupts disabled or from within a fault handler.
 *
 * Takes no parameters.
 *
 * @return None
 */
void thruster_force_neutral(void);

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_SAFETY_THRUSTER_SAFE_STATE_H_ */
