/*
 * freertos.h
 *
 *  Created on: Nov 12, 2024
 *      Author: root
 */

#ifndef INC_FREERTOS_H_
#define INC_FREERTOS_H_
#ifdef __cplusplus
extern "C" {
#endif
/*
 * INCLUDES
 */
#include "usart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rclc_parameter/rclc_parameter.h>

#include "micro_ros_utilities/type_utilities.h"

#include <sensor_msgs/msg/Imu.h>
#include <sensor_msgs/msg/Joy.h>
#include <sensor_msgs/msg/Fluid_Pressure.h>
#include <sensor_msgs/msg/Temperature.h>

#include <std_srvs/srv/set_bool.h>

#include <nereo_interfaces/msg/thruster_statuses.h>
#include <nereo_interfaces/msg/command_velocity.h>
#include <nereo_interfaces/srv/set_navigation_mode.h>

#include "navigation/stabilize_mode.h"
#include "navigation/navigation.h"

/*
 * BEGIN TYPEDEF
 */
/*
 * Manual mode: no stabilization used
 * Stabilize full: stabilizes depth, pitch, roll, yaw
 * Stabilize depth: stabilizes only depth
 * Stabilize_r_p: only stabilizes pitch and roll
 * Stabilize angles: only stabilizes pitch, roll and yaw
 */
typedef enum {
	NAVIGATION_MODE_MANUAL,
    NAVIGATION_MODE_STABILIZE_FULL,
	NAVIGATION_MODE_STABILIZE_DEPTH,
	NAVIGATION_MODE_STABILIZE_R_P,
	NAVIGATION_MODE_STABILIZE_ANGLES
} NavigationModes;

typedef enum {
	ROV_DISARMED,
	ROV_ARMED,
} RovArmModes;
/*
 * END TYPEDEF
 */

/*
 * BEGIN MACROS
 */
// combined number of timers, services and subscriptions: needed for initializing the executor: make sure to udpate this if adding any timer or subscription
#define NUMBER_SUBS_TIMS_SRVS 6
#define DEFAULT_TASK_FREQUENCY_HZ 100
#define TS_DEFAULT_TASK_MS (1000/DEFAULT_TASK_FREQUENCY_HZ)
/*
 * END MACROS
 */

/*
 * BEGIN FUNCTIONS PROTOTYPES
 */
// Micro ROS functions
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// Private functions
void imu_subscription_callback (const void * msgin);
void cmd_vel_subscription_callback (const void * msgin);
void pressure_subscription_callback (const void * msgin);
void temperature_subscription_callback (const void * msgin);

void set_pwm_idle();
void set_pwms(uint32_t pwms[8]);
void clamp_pwm_output(uint32_t *, int);
void update_pid_constants(arm_pid_instance_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd);

void arm_disarm_service_callback(const void *, void *);
void set_nav_mode_service_callback(const void *, void *);
/*
 * END FUNCTION PROTOTYPES
 */

#ifdef __cplusplus
}
#endif
#endif /* INC_FREERTOS_H_ */
