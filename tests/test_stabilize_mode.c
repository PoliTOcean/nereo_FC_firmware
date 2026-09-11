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
 * Later cases in this file pin specific output values as a golden
 * characterization capture, not as hand-computed expectations -- see
 * the block comment above those cases for why that discipline differs
 * from test_arm_pid_stand_in.c's hand-computed one.
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
			&water_pressure);

	TEST_ASSERT_EQUAL_INT(ARM_MATH_SUCCESS, status);
	for (uint8_t i = 0; i < 8; i++) {
		TEST_ASSERT_GREATER_OR_EQUAL_UINT32(PWM_MIN, pwm_output[i]);
		TEST_ASSERT_LESS_OR_EQUAL_UINT32(PWM_MAX, pwm_output[i]);
	}
}
