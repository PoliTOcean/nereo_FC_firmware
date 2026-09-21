/*
 * stabilize_mode.h
 *
 *  Created on: May 28, 2024
 *      Author: michelecarenini
 *
 */

#ifndef CORE_INC_STABILIZE_MODE_H_
#define CORE_INC_STABILIZE_MODE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "navigation.h"
#include "arm_math.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
//#include "full_state_feedback_control.h"

#define PID_NUMBER 4

//extern ControlSystem controllers[3];

/**
 * @brief Owns every piece of navigation/PID control state the 40 Hz
 *        control loop mutates across cycles.
 *
 * Exactly one instance of this type exists: `control_ctx`, declared
 * `static` at file scope inside stabilize_mode.c. It is not passed by
 * pointer from the caller, deliberately: calculate_pwm_with_pid() and
 * calculate_pwm_with_pid_anti_windup() are both called from exactly one
 * place inside freertos.cpp's control loop, in a single `while(1)` task
 * with no recursion and no concurrent access, and the storage must be
 * `static` somewhere regardless to survive across loop iterations -- a
 * pointer passed in on every call would add an indirection with no
 * safety benefit. If a future phase ever needs multiple simultaneous
 * control contexts, converting to a passed pointer is a small, local
 * change at that point; building for it now is speculative.
 *
 * This state is touched only by functions reached from
 * `StartDefaultTask`: `rclc_executor_spin_some()` dispatches every
 * subscription callback on the calling task, so there is no second
 * thread and no interrupt context ever touching this struct, and no
 * `volatile` qualifier or critical section is warranted. A future phase
 * that adds a second writer to this state is breaking that single-task
 * assumption, and should re-derive whether synchronization is needed
 * from scratch rather than assuming today's absence of it still holds.
 */
typedef struct {
	/* Depth, roll, pitch, yaw setpoints, in that order -- was: static
	 * float setpoints[4]. */
	float setpoints[4];
	/* Per-axis "was the last commanded velocity non-zero" latch, in
	 * depth/roll/pitch/yaw order -- was: static uint8_t
	 * last_cmd_vel_neq_0[4] = {1}. D-09: the asymmetric {1, 0, 0, 0}
	 * initial value is preserved verbatim pending the original
	 * author's answer and must not be normalised. */
	uint8_t last_cmd_vel_neq_0[4];
	/* One-shot latch seeding the three angular setpoints on first
	 * entry -- was: static uint8_t first_update = 1. */
	uint8_t first_update;
	/* Tracks whether the depth setpoint has yet been seeded from a
	 * usable pressure reading -- was: static uint8_t
	 * depth_setpoint_seeded = 0 (added by plan 03-02). */
	uint8_t depth_setpoint_seeded;
	/* PID controllers, respectively for z, roll, pitch, yaw -- was:
	 * arm_pid_instance_f32 pids[4] = {0}, with external linkage. */
	arm_pid_instance_f32 pids[PID_NUMBER];
} ControlContext;

/**
 * @brief Read-only access to the module's single control-state instance.
 *
 * Exists so tests can observe integrator and setpoint state without the
 * module exposing a writable symbol. The returned pointer is `const`
 * precisely so no caller can use it to mutate control state; production
 * code has no reason to call this function.
 *
 * @return Pointer to the single file-scope ControlContext instance.
 */
const ControlContext *stabilize_mode_get_context(void);

/**
 * @brief Updates setpoints for angles and depth based on joystick input and current orientation/pressure data.
 *
 * @param input_values Array of 6 joystick input values: [surge, sway, heave, roll, pitch, yaw]
 * @param quat Pointer to a Quaternion structure representing the current orientation.
 * @param water_pressure Pointer to the current water pressure measurement.
 * @param pressure_is_fresh false means the firmware cannot vouch for
 *        water_pressure -- either it is NULL or the reading is older
 *        than the staleness budget. The depth setpoint is not seeded
 *        and not updated when false; roll, pitch and yaw setpoints are
 *        unaffected.
 * @return uint8_t The count of updated setpoints (roll, pitch, yaw, or depth).
 */
uint8_t update_setpoints(const float input_values[6], const Quaternion *quat, const float *water_pressure,
		bool pressure_is_fresh);

void calculate_rpy_from_quaternion(const Quaternion *quaternion, float roll_pitch_yaw_radians[3]);

void init_pids(float kps[PID_NUMBER], float kis[PID_NUMBER], float kds[PID_NUMBER]);

/**
 * @brief Resets setpoints and update flags, and zeroes PID integrator
 *        state, without touching the tuned gains.
 *
 * Policy (reviewed): the integrator accumulates across consecutive
 * control cycles for as long as stabilize mode stays continuously
 * active, and is zeroed together with the setpoints on every entry
 * into stabilize mode, so that re-entering the mode never discharges
 * correction accumulated before the mode was left. The gains are
 * deliberately not cleared by this reset, so a controller tuned at
 * runtime does not lose its tuning by re-entering a mode.
 *
 * Zeroes the four setpoints and restores the update flags
 * (`first_update`, `depth_setpoint_seeded` and `last_cmd_vel_neq_0`) to
 * their declared initial values, then zeroes each PID's integrator
 * state by calling
 * `arm_pid_init_f32()` with a non-zero reset flag. `Kp`, `Ki` and `Kd`
 * are left exactly as they are, so a caller that has already tuned the
 * controller via `init_pids()` does not lose that tuning by re-entering
 * a mode. Called by `freertos.cpp`'s control loop on the transition
 * into `ARBITER_STABILIZE_FULL`, and by the host test suite.
 *
 * @return None.
 */
void stabilize_mode_reset(void);

/**
 * @brief Calculates the PWM output using PID control to adjust orientation and depth based on joystick input.
 *
 * @param joystick_input Array of 6 cmd velocity values: [surge, sway, heave, roll, pitch, yaw].
 * @param pwm_output Array to store the 8 calculated PWM output values (thrusters 1 to 8, referring to BlueRobotics' Blue Rov Heavy)
 * @param orientation_quaternion Pointer to the current orientation as a quaternion.
 * @param water_pressure Pointer to the current water pressure (used to estimate depth).
 * @param pressure_is_fresh false means the firmware cannot vouch for
 *        water_pressure -- either it is NULL or the reading is older
 *        than the staleness budget. The depth axis degrades to pilot
 *        passthrough when false: the depth PID is not evaluated and
 *        contributes 0.0 to the correction, while roll, pitch and yaw
 *        stabilization keep running.
 * @return int8_t A status code indicating the success or failure of the PWM calculation.
 */
arm_status calculate_pwm_with_pid(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure, bool pressure_is_fresh);

arm_status calculate_pwm_with_pid_anti_windup(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure, bool pressure_is_fresh);

arm_status calculate_pwm_cs_controller(const float cmd_vel[6], uint32_t pwm_output[8], const Quaternion *orientation_quaternion,
		const float *water_pressure);

#ifdef __cplusplus
}
#endif
#endif /* CORE_INC_STABILIZE_MODE_H_ */
