/**
 * @file safety_arbiter.h
 * @brief Decides which PWM computation path the 40 Hz control loop should
 * take, given the vehicle's arm state, thruster-test override and selected
 * navigation mode (TEST-05).
 *
 * This is a pure decision function: it reads only its parameters and
 * touches no global state. The three control-state globals this decision
 * depends on -- rov_arm_mode, navigation_mode, thruster_test_mode, all
 * declared with the storage-class qualifier that marks them as written from
 * an interrupt/task context outside the compiler's own control flow -- stay
 * declared exactly where they were in freertos.cpp: this extraction moves
 * the decision, never the state (D-10). freertos.cpp reads the current
 * values of those globals at its call site and passes them in; ownership of
 * the control state itself is out of scope for this phase and belongs to a
 * later phase's control-state encapsulation work.
 *
 * The navigation mode is taken as a plain int rather than the vehicle
 * control code's own NavigationModes enum, and deliberately so: that enum
 * (and RovArmModes) is typedef'd inside a header that also pulls in the
 * micro-ROS client library and its executor layer, the CMSIS-DSP math
 * substitution's real counterpart, and five message-type headers. A header
 * that reached for those enum types "for type safety" would inherit that
 * entire middleware include chain and make this translation unit
 * dependency-free in name only -- the one property this extraction exists
 * to establish, so it can be built and tested on a host with no ARM
 * cross-compiler.
 *
 * @author PoliTOcean
 * @date Sep 6, 2026
 */
#ifndef CORE_INC_ARBITER_SAFETY_ARBITER_H_
#define CORE_INC_ARBITER_SAFETY_ARBITER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * One enumerator per branch the pre-extraction inline arbitration code
 * could take.
 */
typedef enum {
	ARBITER_IDLE,           /* disarmed -- caller should call
	                         * set_pwm_idle(), regardless of thruster-test
	                         * mode or navigation mode: disarmed dominates
	                         * both. */
	ARBITER_THRUSTER_TEST,  /* armed, thruster-test mode set -- caller
	                         * should copy the raw thruster_test_pwms[8]
	                         * values, regardless of navigation mode:
	                         * thruster-test mode dominates navigation
	                         * mode. */
	ARBITER_MANUAL,         /* armed, thruster-test mode clear, navigation
	                         * mode is the manual mode (numeric 0). */
	ARBITER_STABILIZE_FULL, /* armed, thruster-test mode clear, navigation
	                         * mode is the full-stabilize mode (numeric
	                         * 15). */
	ARBITER_UNKNOWN_MODE,   /* armed, thruster-test mode clear, navigation
	                         * mode is anything else -- this corresponds
	                         * exactly to the pre-extraction switch's
	                         * default: branch, which produces a flat,
	                         * untrimmed PWM 1500 on all eight channels.
	                         * This is deliberately NOT the trimmed neutral
	                         * thruster_force_neutral() writes on the
	                         * fault path; the two "neutral-looking"
	                         * values are not the same state. Preserving
	                         * this discrepancy exactly is required
	                         * because this extraction is a pure movement,
	                         * not a fix (D-11) -- reconciling it is a
	                         * later phase's decision. */
} ArbiterDecision;

/**
 * @brief Decides which PWM computation path the control loop should take.
 *
 * Pure function: reads only its parameters, touches no global state.
 * Precedence is: disarmed dominates everything; within armed,
 * thruster-test mode dominates navigation mode; within armed and not
 * thruster-test, the navigation mode selects the decision, with numeric 0
 * a named, handled mode (not absent or unset).
 *
 * @param armed true if the vehicle's arm-mode global equals its armed
 *              enumerator, false otherwise.
 * @param thruster_test_mode current value of the thruster-test-mode global.
 * @param nav_mode current value of the navigation-mode global, taken as its
 *                 raw int value rather than the vehicle control code's own
 *                 enum type (see file header comment for why).
 * @return the decision the caller should switch on.
 */
ArbiterDecision arbiter_decide(bool armed, bool thruster_test_mode,
		int nav_mode);

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_ARBITER_SAFETY_ARBITER_H_ */
