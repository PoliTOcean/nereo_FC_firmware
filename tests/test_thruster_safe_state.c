/**
 * @file test_thruster_safe_state.c
 * @brief Fault-injection regression test for the fault-safe stop.
 *
 * This is the automated counterpart of Phase 1's bench-and-scope
 * verification: it exercises thruster_force_neutral(),
 * the single function every one of Phase 1's six fault paths invokes as
 * their first action -- not the fault paths themselves. It deliberately
 * does not exercise the vehicle's real interrupt-disabling fault entry
 * point: that path performs a full HAL/clock init chain and spins
 * forever, which this host harness cannot and does not attempt to
 * reproduce. Only thruster_force_neutral()'s register-write semantics are
 * under test here.
 *
 * @author Davide Colabella
 * @date Sep 6, 2026
 */
#include "unity.h"
#include "safety/thruster_safe_state.h"
#include "main.h"

/*
 * Sentinel poisoned into every CCR field before each test. Chosen far
 * outside any valid PWM pulse width on this vehicle (roughly 1000-2000),
 * so a passing assertion proves the write happened rather than proving
 * the struct was already zero or already correct.
 */
#define CCR_SENTINEL 0xDEADBEEFu

void setUp(void)
{
	TIM2_Instance.CCR1 = CCR_SENTINEL;
	TIM2_Instance.CCR2 = CCR_SENTINEL;
	TIM2_Instance.CCR3 = CCR_SENTINEL;
	TIM2_Instance.CCR4 = CCR_SENTINEL;
	TIM3_Instance.CCR1 = CCR_SENTINEL;
	TIM3_Instance.CCR2 = CCR_SENTINEL;
	TIM3_Instance.CCR3 = CCR_SENTINEL;
	TIM3_Instance.CCR4 = CCR_SENTINEL;
}

void tearDown(void)
{
}

/*
 * The eight expected values below are copied verbatim, in the exact
 * arithmetic form thruster_safe_state.c uses (a neutral base of 1500 plus
 * or minus the per-motor trim), from
 * Core/Src/safety/thruster_safe_state.c. This duplication is deliberate:
 * this test is the mechanism by which a change to the trim table becomes
 * visible, so the values must not be imported from the code under test --
 * doing so would make the test agree with any trim table, including a
 * wrong one.
 *
 * None of the eight values below is a flat 1500: this test does not
 * accept an untrimmed neutral for any channel. The control loop's
 * unknown-navigation-mode branch writes a flat 1500 to all eight
 * channels, which looks like a "safe default" but is a different state
 * from this trimmed neutral -- this test asserts the trimmed values only,
 * and a future reader should not "fix" it to accept 1500.
 *
 * CCR registers are integers, so TEST_ASSERT_EQUAL_UINT32 (plain integer
 * equality) is the correct assertion here -- this is not a
 * floating-point comparison and carries no explicit tolerance
 * requirement.
 *
 * Each channel is asserted against its own named register so a
 * permutation of the write targets (e.g. two swapped CCR fields whose
 * trim values differ) is detected, not just a change to the value
 * multiset.
 */
void test_thruster_force_neutral_writes_trimmed_values_in_fixed_order(void)
{
	thruster_force_neutral();

	TEST_ASSERT_EQUAL_UINT32(1500 - 25, TIM2_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR4);
	TEST_ASSERT_EQUAL_UINT32(1500 + 44, TIM3_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 42, TIM3_Instance.CCR4);
}

/*
 * Proves the poisoned sentinel from setUp() is actually overwritten by
 * the call, not merely coincidentally already correct -- a passing
 * assertion here shows all eight channels were written, not just the
 * ones that happened to already hold the right value.
 */
void test_thruster_force_neutral_overwrites_every_poisoned_channel(void)
{
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR4);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR4);

	thruster_force_neutral();

	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR1);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR2);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR3);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM2_Instance.CCR4);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR1);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR2);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR3);
	TEST_ASSERT_NOT_EQUAL_UINT32(CCR_SENTINEL, TIM3_Instance.CCR4);
}

/*
 * thruster_force_neutral() carries no state of its own: calling it twice
 * in succession must leave the same eight trimmed values, proving it is
 * idempotent rather than accumulating or toggling on repeated calls.
 */
void test_thruster_force_neutral_is_idempotent(void)
{
	thruster_force_neutral();
	thruster_force_neutral();

	TEST_ASSERT_EQUAL_UINT32(1500 - 25, TIM2_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR4);
	TEST_ASSERT_EQUAL_UINT32(1500 + 44, TIM3_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 42, TIM3_Instance.CCR4);
}

/*
 * Pre-loads plausible mid-flight command values (not the poison
 * sentinel) into every channel, then calls the function under test. The
 * eight trimmed values must still result, proving that whatever a
 * command write had just placed there at fault entry is discarded, not
 * blended with or preserved alongside the safe state.
 */
void test_thruster_force_neutral_discards_prior_command_values(void)
{
	TIM2_Instance.CCR1 = 1620u;
	TIM2_Instance.CCR2 = 1380u;
	TIM2_Instance.CCR3 = 1550u;
	TIM2_Instance.CCR4 = 1450u;
	TIM3_Instance.CCR1 = 1500u;
	TIM3_Instance.CCR2 = 1700u;
	TIM3_Instance.CCR3 = 1300u;
	TIM3_Instance.CCR4 = 1600u;

	thruster_force_neutral();

	TEST_ASSERT_EQUAL_UINT32(1500 - 25, TIM2_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 46, TIM2_Instance.CCR4);
	TEST_ASSERT_EQUAL_UINT32(1500 + 44, TIM3_Instance.CCR1);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR2);
	TEST_ASSERT_EQUAL_UINT32(1500 + 47, TIM3_Instance.CCR3);
	TEST_ASSERT_EQUAL_UINT32(1500 + 42, TIM3_Instance.CCR4);
}
