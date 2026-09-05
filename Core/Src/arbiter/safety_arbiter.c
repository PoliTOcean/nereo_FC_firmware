/**
 * @file safety_arbiter.c
 * @brief Implements the pure arm/test/navigation-mode decision declared in
 * safety_arbiter.h.
 *
 * Includes only its own header and, transitively through it, <stdbool.h> --
 * nothing else. In particular this file does NOT include the header where
 * NavigationModes and RovArmModes actually live, nor the navigation module's
 * own header (which also pulls in the CMSIS-DSP math substitution's real
 * counterpart): reaching for either would drag the whole micro-ROS client
 * library and CMSIS-DSP include chain onto this translation unit and defeat
 * the dependency-free property this extraction exists to establish (D-09).
 * The two handled navigation-mode numeric values are instead defined as
 * private constants below, mirroring how thruster_safe_state.c defines its
 * own private neutral-pulse constant rather than including the navigation
 * module's header for the same reason.
 *
 * @author Davide Colabella
 * @date Sep 6, 2026
 */
#include "arbiter/safety_arbiter.h"

/*
 * File-local navigation-mode numeric values, one per case this function
 * handles. Kept private (never in the header) for the same reason
 * thruster_safe_state.c keeps its own neutral-pulse constant private:
 * exporting these, or including the header where the named enumerators
 * actually live to obtain them directly, would pull in the whole
 * micro-ROS/CMSIS-DSP chain this module is dependency-free of. Values
 * verified against the vehicle control code's NavigationModeFlags and
 * NavigationModes enum definitions.
 */
#define ARBITER_NAV_MODE_MANUAL 0          /* NAVIGATION_MODE_MANUAL */
#define ARBITER_NAV_MODE_STABILIZE_FULL 15 /* NAVIGATION_MODE_STABILIZE_FULL */

/**
 * Decides which PWM computation path the control loop should take. See
 * safety_arbiter.h for the full precedence contract and the ArbiterDecision
 * enumerators' meanings.
 */
ArbiterDecision arbiter_decide(bool armed, bool thruster_test_mode,
		int nav_mode)
{
	if (!armed) {
		return ARBITER_IDLE;
	}

	if (thruster_test_mode) {
		return ARBITER_THRUSTER_TEST;
	}

	switch (nav_mode) {
	case ARBITER_NAV_MODE_MANUAL:
		return ARBITER_MANUAL;
	case ARBITER_NAV_MODE_STABILIZE_FULL:
		return ARBITER_STABILIZE_FULL;
	default:
		return ARBITER_UNKNOWN_MODE;
	}
}
