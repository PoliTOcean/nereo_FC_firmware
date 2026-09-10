/**
 * @file test_safety_arbiter.c
 * @brief Host-based Ceedling/Unity decision-table tests for the safety
 * arbiter declared in arbiter/safety_arbiter.h.
 *
 * The arbiter is a pure function over three primitive parameters (a bool,
 * a bool and an int), so this test file needs no shim: same zero-shim
 * shape as test_interpolations.c. It does not include the header where
 * the vehicle's own navigation-mode enum is declared -- reaching for that
 * enum for "type safety" is exactly what the extraction avoids, and
 * pulling it into a test file would reintroduce the whole dependency
 * chain the extraction exists to eliminate. Navigation-mode arguments are
 * therefore written as their bare numeric literals below, each with the
 * corresponding enum name in a comment on the same line, so a reader sees
 * both the value the arbiter actually compares and the name it stands
 * for.
 *
 * Covers every cell of the three-input decision table: the disarmed case
 * (which must dominate both other inputs), the thruster-test case (which
 * must dominate navigation mode), all six declared navigation-mode
 * values, two out-of-range values, and statelessness across calls.
 *
 * @author Davide Colabella
 * @date Sep 6, 2026
 */
#include "unity.h"
#include "arbiter/safety_arbiter.h"
#include <stdbool.h>

void setUp(void)
{
}

void tearDown(void)
{
}

/* ---------------------------------------------------------------------
 * Disarmed dominates everything.
 * ------------------------------------------------------------------- */

/**
 * Disarmed, thruster-test off, manual mode (0): the baseline disarmed
 * case yields the idle decision.
 */
void test_arbiter_disarmed_manual_mode_yields_idle(void)
{
    /* nav_mode 0 == NAVIGATION_MODE_MANUAL */
    ArbiterDecision result = arbiter_decide(false, false, 0);
    TEST_ASSERT_EQUAL(ARBITER_IDLE, result);
}

/**
 * Disarmed, thruster-test ON: disarmed still yields idle. Proves the
 * disarmed check is evaluated first and short-circuits thruster-test
 * mode entirely.
 */
void test_arbiter_disarmed_thruster_test_on_still_yields_idle(void)
{
    /* nav_mode 0 == NAVIGATION_MODE_MANUAL */
    ArbiterDecision result = arbiter_decide(false, true, 0);
    TEST_ASSERT_EQUAL(ARBITER_IDLE, result);
}

/**
 * Disarmed with the full-stabilize navigation mode selected: disarmed
 * still yields idle. Proves disarmed dominates navigation mode too, not
 * only thruster-test mode.
 */
void test_arbiter_disarmed_stabilize_full_mode_still_yields_idle(void)
{
    /* nav_mode 15 == NAVIGATION_MODE_STABILIZE_FULL */
    ArbiterDecision result = arbiter_decide(false, false, 15);
    TEST_ASSERT_EQUAL(ARBITER_IDLE, result);
}

/**
 * Disarmed with an unrecognised navigation-mode value: still idle.
 * Disarmed dominates regardless of what navigation mode would otherwise
 * resolve to.
 */
void test_arbiter_disarmed_unrecognised_mode_still_yields_idle(void)
{
    /* nav_mode 99: not a declared navigation mode */
    ArbiterDecision result = arbiter_decide(false, false, 99);
    TEST_ASSERT_EQUAL(ARBITER_IDLE, result);
}

/* ---------------------------------------------------------------------
 * Armed + thruster-test dominates navigation mode.
 * ------------------------------------------------------------------- */

/**
 * Armed, thruster-test on, manual mode selected: thruster-test still
 * wins over navigation mode.
 */
void test_arbiter_armed_thruster_test_dominates_manual_mode(void)
{
    /* nav_mode 0 == NAVIGATION_MODE_MANUAL */
    ArbiterDecision result = arbiter_decide(true, true, 0);
    TEST_ASSERT_EQUAL(ARBITER_THRUSTER_TEST, result);
}

/**
 * Armed, thruster-test on, full-stabilize mode selected: thruster-test
 * still wins.
 */
void test_arbiter_armed_thruster_test_dominates_stabilize_full_mode(void)
{
    /* nav_mode 15 == NAVIGATION_MODE_STABILIZE_FULL */
    ArbiterDecision result = arbiter_decide(true, true, 15);
    TEST_ASSERT_EQUAL(ARBITER_THRUSTER_TEST, result);
}

/**
 * Armed, thruster-test on, an unrecognised navigation mode selected:
 * thruster-test still wins even over a mode that would otherwise be
 * unknown.
 */
void test_arbiter_armed_thruster_test_dominates_unrecognised_mode(void)
{
    /* nav_mode 99: not a declared navigation mode */
    ArbiterDecision result = arbiter_decide(true, true, 99);
    TEST_ASSERT_EQUAL(ARBITER_THRUSTER_TEST, result);
}

/* ---------------------------------------------------------------------
 * Armed, thruster-test off: selection by navigation mode.
 * ------------------------------------------------------------------- */

/**
 * Armed, thruster-test off, navigation mode 0: yields the manual
 * decision. Numeric zero is a named, handled mode (NAVIGATION_MODE_MANUAL)
 * and must not be treated as absent or unset -- a bug that skipped
 * dispatch on a falsy nav_mode value would route manual piloting into the
 * unknown-mode branch instead.
 */
void test_arbiter_armed_manual_mode_zero_is_handled_not_absent(void)
{
    /* nav_mode 0 == NAVIGATION_MODE_MANUAL */
    ArbiterDecision result = arbiter_decide(true, false, 0);
    TEST_ASSERT_EQUAL(ARBITER_MANUAL, result);
}

/**
 * Armed, thruster-test off, navigation mode 15: yields the stabilize-full
 * decision.
 */
void test_arbiter_armed_stabilize_full_mode_is_handled(void)
{
    /* nav_mode 15 == NAVIGATION_MODE_STABILIZE_FULL */
    ArbiterDecision result = arbiter_decide(true, false, 15);
    TEST_ASSERT_EQUAL(ARBITER_STABILIZE_FULL, result);
}

/*
 * Five of the six declared navigation modes are not handled by the
 * pre-extraction switch and reach its default branch, which this
 * extraction preserves as ARBITER_UNKNOWN_MODE. That branch produces a
 * flat, untrimmed PWM 1500 on all eight channels -- NOT the trimmed
 * neutral the fault path writes. Both facts are pre-existing behaviour,
 * deliberately preserved rather than fixed by this extraction;
 * reconciling the discrepancy is a later phase's decision. The five tests
 * below pin each of those five modes individually, by numeric value, so
 * the surprise is documented rather than discovered during a wet test.
 */

/**
 * Navigation mode 16 (NAVIGATION_MODE_STABILIZE_CS) is a case label whose
 * body is commented out in the pre-extraction switch and falls straight
 * through to default: -- it yields the unknown-mode decision, the same
 * as a genuinely unrecognised value.
 */
void test_arbiter_armed_stabilize_cs_mode_yields_unknown(void)
{
    /* nav_mode 16 == NAVIGATION_MODE_STABILIZE_CS */
    ArbiterDecision result = arbiter_decide(true, false, 16);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/**
 * Navigation mode 1 (NAVIGATION_MODE_STABILIZE_DEPTH) is declared but not
 * named in the pre-extraction switch: yields the unknown-mode decision.
 */
void test_arbiter_armed_stabilize_depth_mode_yields_unknown(void)
{
    /* nav_mode 1 == NAVIGATION_MODE_STABILIZE_DEPTH */
    ArbiterDecision result = arbiter_decide(true, false, 1);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/**
 * Navigation mode 6 (NAVIGATION_MODE_STABILIZE_R_P) is declared but not
 * named in the pre-extraction switch: yields the unknown-mode decision.
 */
void test_arbiter_armed_stabilize_r_p_mode_yields_unknown(void)
{
    /* nav_mode 6 == NAVIGATION_MODE_STABILIZE_R_P */
    ArbiterDecision result = arbiter_decide(true, false, 6);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/**
 * Navigation mode 14 (NAVIGATION_MODE_STABILIZE_ANGLES) is declared but
 * not named in the pre-extraction switch: yields the unknown-mode
 * decision. This value is also one of the three consecutive integers
 * (14, 15, 16) exercised together below, to catch an off-by-one
 * comparison specifically.
 */
void test_arbiter_armed_stabilize_angles_mode_yields_unknown(void)
{
    /* nav_mode 14 == NAVIGATION_MODE_STABILIZE_ANGLES */
    ArbiterDecision result = arbiter_decide(true, false, 14);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/**
 * The three consecutive integers 14, 15 and 16 are asserted together,
 * each against its own distinct expected decision, so an off-by-one
 * comparison in the arbiter's handled-mode constant (e.g. comparing
 * against 14 or 16 instead of 15) is caught. Only 15 is handled; its two
 * integer neighbours are not, and this test proves they do not silently
 * share 15's decision.
 */
void test_arbiter_consecutive_modes_14_15_16_are_distinguished(void)
{
    /* 14 == NAVIGATION_MODE_STABILIZE_ANGLES (unhandled) */
    ArbiterDecision mode_14 = arbiter_decide(true, false, 14);
    /* 15 == NAVIGATION_MODE_STABILIZE_FULL (handled) */
    ArbiterDecision mode_15 = arbiter_decide(true, false, 15);
    /* 16 == NAVIGATION_MODE_STABILIZE_CS (unhandled) */
    ArbiterDecision mode_16 = arbiter_decide(true, false, 16);

    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, mode_14);
    TEST_ASSERT_EQUAL(ARBITER_STABILIZE_FULL, mode_15);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, mode_16);
}

/* ---------------------------------------------------------------------
 * Out-of-range navigation-mode values.
 * ------------------------------------------------------------------- */

/**
 * A negative navigation-mode value: yields the unknown-mode decision,
 * same as any other unrecognised value.
 */
void test_arbiter_armed_negative_mode_yields_unknown(void)
{
    ArbiterDecision result = arbiter_decide(true, false, -1);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/**
 * A large out-of-range navigation-mode value: yields the unknown-mode
 * decision.
 */
void test_arbiter_armed_large_out_of_range_mode_yields_unknown(void)
{
    ArbiterDecision result = arbiter_decide(true, false, 999999);
    TEST_ASSERT_EQUAL(ARBITER_UNKNOWN_MODE, result);
}

/* ---------------------------------------------------------------------
 * Statelessness.
 * ------------------------------------------------------------------- */

/**
 * Calling the function twice with identical arguments yields identical
 * decisions, and calling it with different arguments in between does not
 * change that -- the function carries no state across calls.
 */
void test_arbiter_decide_is_stateless_across_calls(void)
{
    /* nav_mode 15 == NAVIGATION_MODE_STABILIZE_FULL */
    ArbiterDecision first_call = arbiter_decide(true, false, 15);
    /* nav_mode 0 == NAVIGATION_MODE_MANUAL */
    ArbiterDecision interleaved_call = arbiter_decide(false, true, 0);
    /* nav_mode 15 == NAVIGATION_MODE_STABILIZE_FULL */
    ArbiterDecision second_call = arbiter_decide(true, false, 15);

    TEST_ASSERT_EQUAL(ARBITER_STABILIZE_FULL, first_call);
    TEST_ASSERT_EQUAL(ARBITER_IDLE, interleaved_call);
    TEST_ASSERT_EQUAL(ARBITER_STABILIZE_FULL, second_call);
}
