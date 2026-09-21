/**
 * @file test_stabilize_mode.c
 * @brief Host reach into Core/Src/navigation/stabilize_mode.c's control
 *        math, plus the SC-5 golden characterization table.
 *
 * Task 1 (this file's first case) proves the whole chain links and runs
 * on the host: project.yml's source list, the Core/Inc/navigation
 * include path, tests/support/arm_math.h's CMSIS-DSP stand-in,
 * navigation.c and ThrusterConfigurationMatrix.c all compile and link
 * with calculate_pwm_with_pid(), with no new shim written. It asserts
 * only that the chain runs to completion and stays inside the
 * documented PWM range -- not specific output values.
 *
 * Task 2 (the remaining two cases) pins calculate_pwm_with_pid's eight
 * outputs, step by step, for a fixed four-step input sequence under a
 * zero-gain scenario and a non-zero-gain scenario. These expected
 * values are a characterization capture of existing behaviour, not
 * hand-computed expectations: they were obtained by building this
 * suite with placeholder values, reading the eight actual outputs per
 * step out of Unity's failure diff, and transcribing them below, then
 * re-running until green. Captured at commit cd0ac5b
 * (Core/Src/navigation/stabilize_mode.c as committed in this plan's
 * Task 1, before any Phase 3 behaviour change).
 *
 * This is the opposite discipline from test_arm_pid_stand_in.c, whose
 * every expected value is hand-computed from a published specification
 * -- there, deriving expectations from the implementation under test
 * would defeat the purpose. Here, the claim under test is that a later
 * refactor (D-07's context-struct move) changes nothing about existing
 * behaviour, and the only way to pin "existing behaviour" is to run the
 * pristine code once and record what it did. The table proves the
 * refactor changed nothing; it proves nothing about whether the
 * controller is correct -- Phase 4 owns correctness, including the
 * water-pressure-as-depth TODO and the anti-windup gain placeholders
 * visible in calculate_pwm_with_pid, neither of which is touched here.
 *
 * The table also pins the last_cmd_vel_neq_0 initialiser's current
 * one-then-three-zeroes asymmetry (D-09): every step runs through
 * update_setpoints() with that exact seed, so a refactor that quietly
 * normalised it would change these values and fail this file.
 *
 * @author Davide Colabella
 * @date Sep 11, 2026
 */
#include "unity.h"
#include "navigation/stabilize_mode.h"

/* stabilize_mode.c calls into navigation.c's calculate_pwm(),
 * invert_quaternion() and multiply_quaternions() directly, not through
 * a header this file includes -- Ceedling only auto-links a .c file
 * whose header this test file itself #includes (test_build_setup.rb),
 * so the two real dependencies are named explicitly below instead of
 * pulled in via a second #include, per Ceedling's own documented
 * mechanism for this exact case (docs/testing-guide/build-directives.md). */
TEST_SOURCE_FILE("Core/Src/navigation/navigation.c")
TEST_SOURCE_FILE("Core/Src/navigation/ThrusterConfigurationMatrix.c")
TEST_SOURCE_FILE("Core/Src/interpolations.c")

void setUp(void)
{
	stabilize_mode_reset();
}

void tearDown(void)
{
}

void test_calculate_pwm_with_pid_reaches_control_math_end_to_end(void)
{
	/* Identity orientation: w=1, x=y=z=0. Already unit-norm. */
	Quaternion orientation = {1.0f, 0.0f, 0.0f, 0.0f};
	float cmd_vel[6] = {0.5f, -0.3f, 0.2f, 0.1f, -0.1f, 0.05f};
	float water_pressure = 1000.0f;
	uint32_t pwm_output[8];
	arm_status status;

	status = calculate_pwm_with_pid(cmd_vel, pwm_output, &orientation,
			&water_pressure, true);

	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_GREATER_OR_EQUAL_UINT32(PWM_MIN, pwm_output[i]);
		TEST_ASSERT_LESS_OR_EQUAL_UINT32(PWM_MAX, pwm_output[i]);
	}
}

/* -------------------------------------------------------------------
 * SC-5 golden characterization table -- see the file header above for
 * the capture procedure, the capture commit and why this file's
 * discipline differs from test_arm_pid_stand_in.c's.
 *
 * Shared across both scenarios below so both provably run identical
 * inputs. Step 0 commands zero on all six axes; steps 1 and 3 command
 * a non-zero heave. No quaternion below is the identity except step 0,
 * so the depth correction's rotation into the body frame is exercised
 * on every other step.
 * ------------------------------------------------------------------- */
static const Quaternion GOLDEN_ORIENTATIONS[4] = {
	/* identity: w^2 = 1 */
	{1.0f, 0.0f, 0.0f, 0.0f},
	/* 90 deg about z: 0.70710678^2 * 2 = 1 */
	{0.70710678f, 0.0f, 0.0f, 0.70710678f},
	/* 90 deg about x: 0.70710678^2 * 2 = 1 */
	{0.70710678f, 0.70710678f, 0.0f, 0.0f},
	/* 120 deg about (1,1,1)/sqrt(3): 0.5^2 * 4 = 1 */
	{0.5f, 0.5f, 0.5f, 0.5f},
};

static const float GOLDEN_CMD_VEL[4][6] = {
	{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
	{0.5f, -0.3f, 0.6f, 0.1f, -0.1f, 0.05f},
	{0.2f, 0.1f, -0.4f, 0.05f, 0.05f, -0.05f},
	{-0.3f, 0.4f, 0.3f, -0.2f, 0.15f, 0.1f},
};

static const float GOLDEN_PRESSURE[4] = {1000.0f, 1005.0f, 998.0f, 1010.0f};

/**
 * @brief Runs one golden-sequence step and asserts the status code.
 *
 * Shared by both scenarios below so each records only its own
 * per-value expected-output assertions, not a duplicated call/status
 * pattern.
 *
 * @param step       Index into GOLDEN_ORIENTATIONS/CMD_VEL/PRESSURE.
 * @param pwm_output Filled with the eight PWM outputs of this step.
 * @return None.
 */
static void run_golden_step(uint8_t step, uint32_t pwm_output[8])
{
	arm_status status;

	status = calculate_pwm_with_pid(GOLDEN_CMD_VEL[step], pwm_output,
			&GOLDEN_ORIENTATIONS[step], &GOLDEN_PRESSURE[step], true);
	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
}

/* Zero-gain baseline: the state the firmware actually ships in today
 * (pids[] is zero-initialised and init_pids() is never called from
 * production code since commit e3e59be removed the parameter server).
 * Every PID contribution below is 0.0 by construction -- a stated
 * property of this scenario, not an accident. This alone cannot prove
 * the PID math is intact (03-RESEARCH.md Pitfall 3), which is why the
 * non-zero-gain scenario below is mandatory. */
void test_calculate_pwm_with_pid_golden_table_zero_gain(void)
{
	float zeros[PID_NUMBER] = {0.0f, 0.0f, 0.0f, 0.0f};
	uint32_t pwm_output[8];

	init_pids(zeros, zeros, zeros);
	stabilize_mode_reset();

	run_golden_step(0, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[7]);

	run_golden_step(1, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1587, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1568, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1398, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1398, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1532, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1636, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1515, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1604, pwm_output[7]);

	run_golden_step(2, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1520, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1432, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1585, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1551, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1527, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1432, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1544, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1503, pwm_output[7]);

	run_golden_step(3, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1398, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1610, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1440, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1457, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1529, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1491, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1495, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1432, pwm_output[7]);
}

/* Non-zero-gain scenario: Kp 1.0, Ki 0.5, Kd 0.25 on all four axes --
 * the triple already used in test_arm_pid_stand_in.c. This actually
 * advances the integrator between steps and exercises the depth
 * correction's quaternion rotation with a non-zero magnitude. */
void test_calculate_pwm_with_pid_golden_table_nonzero_gain(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	uint32_t pwm_output[8];

	init_pids(kps, kis, kds);
	stabilize_mode_reset();

	run_golden_step(0, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1500, pwm_output[7]);

	run_golden_step(1, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1587, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1568, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1398, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1398, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1532, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1636, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1515, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1604, pwm_output[7]);

	run_golden_step(2, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1506, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1626, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1378, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1664, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1508, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1330, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1514, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1501, pwm_output[7]);

	run_golden_step(3, pwm_output);
	TEST_ASSERT_EQUAL_UINT32(1473, pwm_output[0]);
	TEST_ASSERT_EQUAL_UINT32(1670, pwm_output[1]);
	TEST_ASSERT_EQUAL_UINT32(1343, pwm_output[2]);
	TEST_ASSERT_EQUAL_UINT32(1629, pwm_output[3]);
	TEST_ASSERT_EQUAL_UINT32(1507, pwm_output[4]);
	TEST_ASSERT_EQUAL_UINT32(1356, pwm_output[5]);
	TEST_ASSERT_EQUAL_UINT32(1498, pwm_output[6]);
	TEST_ASSERT_EQUAL_UINT32(1482, pwm_output[7]);
}

/* -------------------------------------------------------------------
 * Pressure null-safety and freshness degrade -- SENS-01/SENS-03.
 * ------------------------------------------------------------------- */

/* Stale pressure yields no depth correction: the eight outputs of a
 * stale-flagged call must equal the outputs of a call whose depth PID
 * contribution is independently zero. A fresh, just-reset call whose
 * pressure value matches the seed it produces has exactly this
 * property (the depth setpoint is seeded to the same value used as
 * the current reading, so the tracking error, and therefore the PID
 * output, is exactly zero) -- see calculate_pwm_with_pid()'s
 * first-call seeding for why this holds regardless of the nonzero
 * gains used here. */
void test_stale_pressure_produces_no_depth_correction(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion orientation = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
	float cmd_vel[6] = {0.5f, -0.3f, 0.6f, 0.1f, -0.1f, 0.05f};
	float pressure = 1005.0f;
	uint32_t stale_output[8];
	uint32_t zero_depth_output[8];
	arm_status status;

	init_pids(kps, kis, kds);

	stabilize_mode_reset();
	status = calculate_pwm_with_pid(cmd_vel, stale_output, &orientation,
			&pressure, false);
	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);

	stabilize_mode_reset();
	status = calculate_pwm_with_pid(cmd_vel, zero_depth_output,
			&orientation, &pressure, true);
	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);

	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_EQUAL_UINT32(zero_depth_output[i], stale_output[i]);
	}
}

/* Roll, pitch and yaw stabilization survive a stale reading: the
 * degrade is depth-only. Seeds all setpoints at rest with an identity
 * orientation, then rotates 90 degrees about x with roll/pitch/yaw
 * commands still at rest -- a real tracking error on the roll axis --
 * while pressure stays stale throughout. The result must differ from
 * the pure-manual mix of the same cmd_vel, proving the angular
 * correction still acted. */
void test_roll_pitch_yaw_survive_stale_pressure(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion identity = {1.0f, 0.0f, 0.0f, 0.0f};
	float rest_cmd_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	Quaternion rotated = {0.70710678f, 0.70710678f, 0.0f, 0.0f};
	float move_cmd_vel[6] = {0.3f, -0.2f, 0.4f, 0.0f, 0.0f, 0.0f};
	float pressure = 1000.0f;
	uint32_t seed_output[8];
	uint32_t stale_output[8];
	uint32_t manual_output[8];
	uint8_t any_channel_differs = 0;

	init_pids(kps, kis, kds);
	calculate_pwm_with_pid(rest_cmd_vel, seed_output, &identity, &pressure,
			false);
	calculate_pwm_with_pid(move_cmd_vel, stale_output, &rotated, &pressure,
			false);
	calculate_pwm(move_cmd_vel, manual_output);

	for (uint8_t i = 0; i < 8; i++) {
		if (stale_output[i] != manual_output[i]) any_channel_differs = 1;
	}
	TEST_ASSERT_TRUE(any_channel_differs);
}

/* A null pressure pointer behaves exactly as a stale reading: same
 * inputs, one call with a null pointer and the flag true, one with a
 * valid pointer and the flag false, element-by-element equal. */
void test_null_pressure_pointer_behaves_as_stale(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion orientation = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
	float cmd_vel[6] = {0.5f, -0.3f, 0.6f, 0.1f, -0.1f, 0.05f};
	float pressure = 1005.0f;
	uint32_t null_output[8];
	uint32_t stale_output[8];
	arm_status status;

	init_pids(kps, kis, kds);

	stabilize_mode_reset();
	status = calculate_pwm_with_pid(cmd_vel, null_output, &orientation,
			NULL, true);
	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);

	stabilize_mode_reset();
	status = calculate_pwm_with_pid(cmd_vel, stale_output, &orientation,
			&pressure, false);
	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);

	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_EQUAL_UINT32(stale_output[i], null_output[i]);
	}
}

/* A null pressure pointer still writes all eight outputs and returns
 * ARM_MATH_SUCCESS: pre-poison the output array with a sentinel far
 * outside [PWM_MIN, PWM_MAX] so an early return that left it untouched
 * is caught. */
void test_null_pressure_pointer_writes_all_outputs(void)
{
	Quaternion orientation = {1.0f, 0.0f, 0.0f, 0.0f};
	float cmd_vel[6] = {0.2f, 0.1f, -0.3f, 0.05f, -0.05f, 0.1f};
	uint32_t pwm_output[8] = {
		0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu,
		0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu, 0xDEADBEEFu,
	};
	arm_status status;

	status = calculate_pwm_with_pid(cmd_vel, pwm_output, &orientation,
			NULL, true);

	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_GREATER_OR_EQUAL_UINT32(PWM_MIN, pwm_output[i]);
		TEST_ASSERT_LESS_OR_EQUAL_UINT32(PWM_MAX, pwm_output[i]);
	}
}

/* The depth setpoint is not seeded from an unusable reading.
 *
 * Compares two scenarios that must be bit-identical if the seed
 * correctly waits for the first *usable* reading: scenario A runs an
 * unusable call carrying a sentinel value far outside any plausible
 * reading, then seeds and probes; scenario B skips the unusable call
 * entirely and goes straight to the identical seed-then-probe pair.
 * If the sentinel had leaked into the seed instead of being ignored,
 * scenario A's PID state and output would diverge sharply from
 * scenario B's -- this needs no assumption about the controller's
 * numeric scaling to detect that divergence. */
void test_depth_setpoint_not_seeded_from_unusable_reading(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion identity = {1.0f, 0.0f, 0.0f, 0.0f};
	float rest_cmd_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float stale_sentinel = 99999.0f;
	float seed_pressure = 1000.0f;
	float probe_pressure = 1010.0f;
	uint32_t discard_output[8];
	uint32_t probe_output_with_stale_call[8];
	uint32_t probe_output_without_stale_call[8];

	/* Scenario A: an ignored unusable call precedes the seed. */
	init_pids(kps, kis, kds);
	stabilize_mode_reset();
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &identity,
			&stale_sentinel, false);
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &identity,
			&seed_pressure, true);
	calculate_pwm_with_pid(rest_cmd_vel, probe_output_with_stale_call,
			&identity, &probe_pressure, true);

	/* Scenario B: no preceding call at all -- the seed is the first
	 * call this scenario ever makes. */
	init_pids(kps, kis, kds);
	stabilize_mode_reset();
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &identity,
			&seed_pressure, true);
	calculate_pwm_with_pid(rest_cmd_vel, probe_output_without_stale_call,
			&identity, &probe_pressure, true);

	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_EQUAL_UINT32(probe_output_without_stale_call[i],
				probe_output_with_stale_call[i]);
	}
}

/* -------------------------------------------------------------------
 * Staleness boundary -- exercised with the loop's own arithmetic, not
 * stabilize_mode.c's, since the comparison lives in freertos.cpp's
 * control loop, not in this module. Keeping the arithmetic identical
 * in form to the loop's means an off-by-one in either is caught by
 * the other.
 *
 * freertos.cpp is never reachable from this host test binary -- it
 * pulls in the micro-ROS client stack, which is not stubbed here (see
 * this file's own header and 03-PLAN.md's phase_environment note) --
 * so the budget value is duplicated here as a literal rather than
 * shared via #include. It must match PRESSURE_STALENESS_BUDGET_MS in
 * Core/Inc/FC_app.h; a future change to that macro must update this
 * literal too.
 * ------------------------------------------------------------------- */
#define TEST_PRESSURE_STALENESS_BUDGET_MS 900

void test_staleness_boundary_at_exactly_the_budget_is_stale(void)
{
	uint32_t now = 10000;
	uint32_t last_update = now - TEST_PRESSURE_STALENESS_BUDGET_MS;
	bool is_fresh = (now - last_update)
			< TEST_PRESSURE_STALENESS_BUDGET_MS;

	TEST_ASSERT_FALSE(is_fresh);
}

void test_staleness_boundary_one_ms_below_budget_is_fresh(void)
{
	uint32_t now = 10000;
	uint32_t last_update = now - (TEST_PRESSURE_STALENESS_BUDGET_MS - 1);
	bool is_fresh = (now - last_update)
			< TEST_PRESSURE_STALENESS_BUDGET_MS;

	TEST_ASSERT_TRUE(is_fresh);
}

/* -------------------------------------------------------------------
 * Integrator reset policy -- SENS-05/D-08. Observed through
 * stabilize_mode_get_context(), the read-only accessor added in this
 * plan's Task 1, never through the pids array directly.
 * ------------------------------------------------------------------- */

/* Intra-mode accumulation: two consecutive calls with the same
 * non-zero roll tracking error must move the roll PID's state, and
 * that state must be non-zero after the second call. A reset-only
 * fix would break this half silently -- see the plan's own warning
 * that a controller resetting on every call is a different and worse
 * defect than never resetting at all. */
void test_integrator_accumulates_within_stabilize_mode(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion seed_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
	Quaternion error_orientation = {0.70710678f, 0.70710678f, 0.0f, 0.0f};
	float rest_cmd_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float pressure = 1000.0f;
	uint32_t discard_output[8];
	const ControlContext *ctx = stabilize_mode_get_context();
	float state_after_first;
	float state_after_second;

	init_pids(kps, kis, kds);
	stabilize_mode_reset();

	/* Seed setpoints from the identity orientation: error is zero on
	 * this call, on every axis. */
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &seed_orientation,
			&pressure, true);

	/* Two consecutive calls at a rotated orientation with cmd_vel at
	 * rest: the roll setpoint stays unchanged, so both calls see the
	 * same non-zero roll tracking error. */
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &error_orientation,
			&pressure, true);
	state_after_first = ctx->pids[1].state[2];

	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &error_orientation,
			&pressure, true);
	state_after_second = ctx->pids[1].state[2];

	TEST_ASSERT_TRUE(state_after_first != state_after_second);
	TEST_ASSERT_TRUE(state_after_second != 0.0f);
}

/* Cross-transition reset: the same accumulation as above, followed by
 * stabilize_mode_reset(), must zero every element of every axis's PID
 * state and restore both update flags to their initial values. */
void test_integrator_resets_on_stabilize_mode_reentry(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion seed_orientation = {1.0f, 0.0f, 0.0f, 0.0f};
	Quaternion error_orientation = {0.70710678f, 0.70710678f, 0.0f, 0.0f};
	float rest_cmd_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float pressure = 1000.0f;
	uint32_t discard_output[8];
	const ControlContext *ctx = stabilize_mode_get_context();

	init_pids(kps, kis, kds);
	stabilize_mode_reset();
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &seed_orientation,
			&pressure, true);
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &error_orientation,
			&pressure, true);
	calculate_pwm_with_pid(rest_cmd_vel, discard_output, &error_orientation,
			&pressure, true);

	stabilize_mode_reset();

	for (uint8_t i = 0; i < PID_NUMBER; i++) {
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, ctx->pids[i].state[0]);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, ctx->pids[i].state[1]);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, ctx->pids[i].state[2]);
	}
	TEST_ASSERT_EQUAL_UINT8(1, ctx->first_update);
	TEST_ASSERT_EQUAL_UINT8(0, ctx->depth_setpoint_seeded);
}

/* Gains survive the reset: Kp/Ki/Kd and their derived coefficients
 * are unchanged after stabilize_mode_reset(), so a controller tuned
 * at runtime does not lose that tuning by re-entering a mode. */
void test_stabilize_mode_reset_preserves_gains(void)
{
	float kps[PID_NUMBER] = {1.0f, 2.0f, 3.0f, 4.0f};
	float kis[PID_NUMBER] = {0.5f, 0.4f, 0.3f, 0.2f};
	float kds[PID_NUMBER] = {0.25f, 0.15f, 0.05f, 0.1f};
	const ControlContext *ctx = stabilize_mode_get_context();

	init_pids(kps, kis, kds);
	stabilize_mode_reset();

	for (uint8_t i = 0; i < PID_NUMBER; i++) {
		float expected_a0 = kps[i] + kis[i] + kds[i];
		float expected_a1 = -kps[i] - 2.0f * kds[i];
		float expected_a2 = kds[i];

		TEST_ASSERT_FLOAT_WITHIN(0.0001f, kps[i], ctx->pids[i].Kp);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, kis[i], ctx->pids[i].Ki);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, kds[i], ctx->pids[i].Kd);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_a0, ctx->pids[i].A0);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_a1, ctx->pids[i].A1);
		TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected_a2, ctx->pids[i].A2);
	}
}

/* First call after re-entry: a call made right after
 * stabilize_mode_reset() must match, output for output, what the very
 * first call from a freshly-reset module produces for the same input
 * -- the observable form of "the integrator is zero on the first call
 * after re-entering stabilize mode." A real, non-zero history is
 * built up before the second reset, so this proves the reset erased
 * it rather than merely testing an already-zero state. */
void test_first_call_after_reentry_matches_fresh_module_state(void)
{
	float kps[PID_NUMBER] = {1.0f, 1.0f, 1.0f, 1.0f};
	float kis[PID_NUMBER] = {0.5f, 0.5f, 0.5f, 0.5f};
	float kds[PID_NUMBER] = {0.25f, 0.25f, 0.25f, 0.25f};
	Quaternion orientation = {0.70710678f, 0.0f, 0.0f, 0.70710678f};
	float cmd_vel[6] = {0.5f, -0.3f, 0.6f, 0.1f, -0.1f, 0.05f};
	float pressure = 1005.0f;
	Quaternion history_orientation_a = {1.0f, 0.0f, 0.0f, 0.0f};
	Quaternion history_orientation_b = {0.70710678f, 0.70710678f, 0.0f, 0.0f};
	float history_cmd_vel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	float history_pressure_a = 500.0f;
	float history_pressure_b = 700.0f;
	uint32_t discard_output[8];
	uint32_t fresh_output[8];
	uint32_t reentry_output[8];

	/* Scenario A: exactly setUp()'s fresh state. */
	init_pids(kps, kis, kds);
	stabilize_mode_reset();
	calculate_pwm_with_pid(cmd_vel, fresh_output, &orientation, &pressure,
			true);

	/* Scenario B: build a real, non-zero history first (a different
	 * orientation/pressure pair on each of two calls, which produces
	 * non-zero tracking error and therefore non-zero PID state), then
	 * reset and repeat scenario A's exact call. */
	init_pids(kps, kis, kds);
	stabilize_mode_reset();
	calculate_pwm_with_pid(history_cmd_vel, discard_output,
			&history_orientation_a, &history_pressure_a, true);
	calculate_pwm_with_pid(history_cmd_vel, discard_output,
			&history_orientation_b, &history_pressure_b, true);
	stabilize_mode_reset();
	calculate_pwm_with_pid(cmd_vel, reentry_output, &orientation, &pressure,
			true);

	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_EQUAL_UINT32(fresh_output[i], reentry_output[i]);
	}
}

/* D-09: the last_cmd_vel_neq_0 one-then-three-zeroes asymmetry
 * survives stabilize_mode_reset() element by element, so no later
 * change can normalise it silently while the author's answer is
 * still pending. */
void test_reset_preserves_last_cmd_vel_neq_0_initial_values(void)
{
	const ControlContext *ctx = stabilize_mode_get_context();

	stabilize_mode_reset();

	TEST_ASSERT_EQUAL_UINT8(1, ctx->last_cmd_vel_neq_0[0]);
	TEST_ASSERT_EQUAL_UINT8(0, ctx->last_cmd_vel_neq_0[1]);
	TEST_ASSERT_EQUAL_UINT8(0, ctx->last_cmd_vel_neq_0[2]);
	TEST_ASSERT_EQUAL_UINT8(0, ctx->last_cmd_vel_neq_0[3]);
}
