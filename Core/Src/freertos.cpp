/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FC_app.h"
#include "safety/thruster_safe_state.h"
#include "arbiter/safety_arbiter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile RovArmModes rov_arm_mode = ROV_DISARMED;
volatile NavigationModes navigation_mode = NAVIGATION_MODE_MANUAL;
volatile bool thruster_test_mode = false;
volatile uint32_t thruster_test_pwms[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
//ControlSystem controllers[3];

// micro-ROS entities shared between create_entities / destroy_entities / task
static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;

static rcl_publisher_t thruster_status_publisher;
static rcl_publisher_t arm_state_publisher;

static nereo_interfaces__msg__ThrusterStatuses thruster_status_msg;
static std_msgs__msg__Bool arm_state_msg;

static rcl_subscription_t cmd_vel_subscriber;
static rcl_subscription_t imu_subscriber;
static rcl_subscription_t thruster_test_subscriber;
static rcl_subscription_t arm_mode_subscriber;
static rcl_subscription_t nav_mode_subscriber;

static nereo_interfaces__msg__CommandVelocity cmd_vel_msg;
static sensor_msgs__msg__Imu imu_data_msg;
static sensor_msgs__msg__FluidPressure fluid_pressure;
static sensor_msgs__msg__Temperature water_temperature;
static std_msgs__msg__Int32MultiArray thruster_test_msg;
static std_msgs__msg__Bool arm_mode_msg;
static std_msgs__msg__Int32 nav_mode_msg;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 5000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern "C" void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
extern "C" void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
static rcl_ret_t create_entities(void)
{
	rcl_ret_t rc;
	static micro_ros_utilities_memory_conf_t default_conf = {0};
	micro_ros_utilities_memory_conf_t test_conf = {0};
	test_conf.max_basic_type_sequence_capacity = 8;

	allocator = rcl_get_default_allocator();

	rc = rclc_support_init(&support, 0, NULL, &allocator);
	if (rc != RCL_RET_OK) { printf("Error support init.\n"); return rc; }
	HAL_IWDG_Refresh(&hiwdg);

	rc = rclc_node_init_default(&node, "fc_node", "", &support);
	if (rc != RCL_RET_OK) { printf("Error node init.\n"); return rc; }
	HAL_IWDG_Refresh(&hiwdg);

	executor = rclc_executor_get_zero_initialized_executor();
	rc = rclc_executor_init(&executor, &support.context, NUMBER_SUBS_TIMS_SRVS, &allocator);
	if (rc != RCL_RET_OK) { printf("Error executor init.\n"); return rc; }
	HAL_IWDG_Refresh(&hiwdg);

	rclc_publisher_init_best_effort(&thruster_status_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, ThrusterStatuses), "/thruster_status");
	rclc_publisher_init_best_effort(&arm_state_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/rov_armed");

	rc = rclc_subscription_init_default(&imu_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu_data");
	if (rc != RCL_RET_OK) { printf("Error imu sub init.\n"); return rc; }
	micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), &imu_data_msg, default_conf);
	rc = rclc_executor_add_subscription(&executor, &imu_subscriber, &imu_data_msg, &imu_subscription_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) { printf("Error executor add imu sub.\n"); return rc; }

	rc = rclc_subscription_init_default(&cmd_vel_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity), "/nereo_cmd_vel");
	if (rc != RCL_RET_OK) { printf("Error cmdvel sub init.\n"); return rc; }
	micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity), &cmd_vel_msg, default_conf);
	rc = rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) { printf("Error executor add cmdvel sub.\n"); return rc; }

	rc = rclc_subscription_init_default(&thruster_test_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "/thruster_pwm_test");
	if (rc != RCL_RET_OK) { printf("Error thruster test sub init.\n"); return rc; }
	micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), &thruster_test_msg, test_conf);
	rc = rclc_executor_add_subscription(&executor, &thruster_test_subscriber, &thruster_test_msg, &thruster_pwm_test_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) { printf("Error executor add thruster test sub.\n"); return rc; }

	arm_mode_msg = {0};
	rc = rclc_subscription_init_default(&arm_mode_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/set_arm_mode");
	if (rc != RCL_RET_OK) { printf("Error arm mode sub init.\n"); return rc; }
	rc = rclc_executor_add_subscription(&executor, &arm_mode_subscriber, &arm_mode_msg, &set_arm_mode_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) { printf("Error executor add arm mode sub.\n"); return rc; }

	nav_mode_msg = {0};
	rc = rclc_subscription_init_default(&nav_mode_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/set_nav_mode");
	if (rc != RCL_RET_OK) { printf("Error nav mode sub init.\n"); return rc; }
	rc = rclc_executor_add_subscription(&executor, &nav_mode_subscriber, &nav_mode_msg, &set_nav_mode_callback, ON_NEW_DATA);
	if (rc != RCL_RET_OK) { printf("Error executor add nav mode sub.\n"); return rc; }

	printf("Micro ROS initialization done.\n");
	return RCL_RET_OK;
}

static void destroy_entities(void)
{
	rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
	(void) uxr_close_custom_transport((uxrCustomTransport *)rmw_ctx->impl);

	rclc_executor_fini(&executor);
	rcl_publisher_fini(&thruster_status_publisher, &node);
	rcl_publisher_fini(&arm_state_publisher, &node);
	rcl_subscription_fini(&imu_subscriber, &node);
	rcl_subscription_fini(&cmd_vel_subscriber, &node);
	rcl_subscription_fini(&thruster_test_subscriber, &node);
	rcl_subscription_fini(&arm_mode_subscriber, &node);
	rcl_subscription_fini(&nav_mode_subscriber, &node);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

	// micro-ROS transport setup (once, before the state machine)
	HAL_IWDG_Refresh(&hiwdg);
	rmw_uros_set_custom_transport(
		true,
		(void *) &huart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);
	HAL_IWDG_Refresh(&hiwdg);

	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;
	if (!rcutils_set_default_allocator(&freeRTOS_allocator))
		printf("Error on default allocators (line %d)\n", __LINE__);
	else HAL_IWDG_Refresh(&hiwdg);

	uint32_t pwm_output[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
	arm_status pwm_computation_error = ARM_MATH_SUCCESS;

	typedef enum { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } AgentState;
	AgentState agent_state = WAITING_AGENT;

	while (1)
	{
		switch (agent_state)
		{
		case WAITING_AGENT:
			if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK)
				agent_state = AGENT_AVAILABLE;
			else {
				osDelay(500);
				HAL_IWDG_Refresh(&hiwdg);
			}
			break;

		case AGENT_AVAILABLE:
			if (create_entities() == RCL_RET_OK) {
				agent_state = AGENT_CONNECTED;
			} else {
				destroy_entities();
				agent_state = WAITING_AGENT;
			}
			break;

		case AGENT_CONNECTED:
		{
			if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
				agent_state = AGENT_DISCONNECTED;
				break;
			}

			uint32_t time_ms = HAL_GetTick();

			rclc_executor_spin_some(&executor, 10000000);

			ArbiterDecision arbiter_decision = arbiter_decide(
					rov_arm_mode == ROV_ARMED, thruster_test_mode,
					(int)navigation_mode);

			switch (arbiter_decision) {
			case ARBITER_IDLE:
				set_pwm_idle();
				break;
			case ARBITER_THRUSTER_TEST:
				for (uint8_t i = 0; i < 8; i++) pwm_output[i] = thruster_test_pwms[i];
				clamp_pwm_output(pwm_output, 8);
				set_pwms(pwm_output);
				break;
			case ARBITER_MANUAL:
				pwm_computation_error = calculate_pwm(cmd_vel_msg.cmd_vel, pwm_output);
				clamp_pwm_output(pwm_output, 8);
				set_pwms(pwm_output);
				break;
			case ARBITER_STABILIZE_FULL:
				pwm_computation_error = calculate_pwm_with_pid(cmd_vel_msg.cmd_vel, pwm_output,
						(Quaternion *)&imu_data_msg.orientation,
						(float *)&fluid_pressure.fluid_pressure);
				clamp_pwm_output(pwm_output, 8);
				set_pwms(pwm_output);
				break;
			case ARBITER_UNKNOWN_MODE:
			default:
				for (uint8_t i = 0; i < 8; i++) pwm_output[i] = 1500;
				clamp_pwm_output(pwm_output, 8);
				set_pwms(pwm_output);
				break;
			}

			for (uint8_t i = 0; i < 8; i++) thruster_status_msg.thruster_pwms[i] = pwm_output[i];
			rcl_ret_t rc = rcl_publish(&thruster_status_publisher, &thruster_status_msg, NULL);
			if (rc != RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
			else HAL_IWDG_Refresh(&hiwdg);

			arm_state_msg.data = (rov_arm_mode == ROV_ARMED);
			rcl_publish(&arm_state_publisher, &arm_state_msg, NULL);

			uint32_t elapsed_time = HAL_GetTick() - time_ms;
			if (elapsed_time < TS_DEFAULT_TASK_MS) osDelay(TS_DEFAULT_TASK_MS - elapsed_time);
			break;
		}

		case AGENT_DISCONNECTED:
			destroy_entities();
			set_pwm_idle();
			rov_arm_mode = ROV_DISARMED;
			thruster_test_mode = false;
			printf("Agent disconnected, waiting for reconnection...\n");
			agent_state = WAITING_AGENT;
			break;
		}
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void inline set_pwms(uint32_t pwms[8])
{
	// HERE THE PWM Channel - Thruster relation is defined
	// TIMx CCRy = pwms[i] + offset   → Motor N (position)
	TIM2 -> CCR1 = pwms[4] - 25;  // Motor 2  - Front SX orizz
	TIM2 -> CCR2 = pwms[3] + 46;  // Motor 6  - Front SX vert
	TIM2 -> CCR3 = pwms[1] + 46;  // Motor 5  - Front DX vert
	TIM2 -> CCR4 = pwms[0] + 46;  // Motor 1  - Front DX orizz
	// VERTICAL THRUSTERS
	TIM3 -> CCR1 = pwms[7] + 44;  // Motor 4  - Rear SX orizz
	TIM3 -> CCR2 = pwms[5] + 47;  // Motor 8  - Rear SX vert
	TIM3 -> CCR3 = pwms[2] + 47;  // Motor 7  - Rear DX vert
	TIM3 -> CCR4 = pwms[6] + 42;  // Motor 3  - Rear DX orizz
}
void inline set_pwm_idle()
{
	thruster_force_neutral();
}
void clamp_pwm_output(uint32_t pwms[], int N) {
	for(uint16_t i = 0; i < N; i++) {
		if (pwms[i] < PWM_MIN)
			pwms[i] = PWM_MIN;
		else if (pwms[i] > PWM_MAX)
			pwms[i] = PWM_MAX;
	}
}

void update_pid_constants(arm_pid_instance_f32 * pid, const float32_t * Kp, const float32_t * Ki, const float32_t * Kd) {
    if(Kp != NULL) pid->Kp = *Kp;
    if(Ki != NULL) pid->Ki = *Ki;
    if(Kd != NULL) pid->Kd = *Kd;

    pid->A0 = pid->Kp + pid->Ki + pid->Kd;
    pid->A1 = -pid->Kp - 2 * pid->Kd;
    pid->A2 = pid->Kd;
}
void imu_subscription_callback(const void * msgin) {
	HAL_IWDG_Refresh(&hiwdg);
}
void cmd_vel_subscription_callback (const void * msgin) {
	thruster_test_mode = false;
	HAL_IWDG_Refresh(&hiwdg);
}
void thruster_pwm_test_callback(const void * msgin) {
	const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
	if (msg->data.size != 8) return;
	for (uint8_t i = 0; i < 8; i++)
		thruster_test_pwms[i] = (uint32_t)msg->data.data[i];
	thruster_test_mode = true;
	HAL_IWDG_Refresh(&hiwdg);
}
void set_arm_mode_callback(const void * msgin) {
	const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
	rov_arm_mode = msg->data ? ROV_ARMED : ROV_DISARMED;
	if (rov_arm_mode == ROV_DISARMED) thruster_test_mode = false;
	HAL_IWDG_Refresh(&hiwdg);
}
void set_nav_mode_callback(const void * msgin) {
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	navigation_mode = (NavigationModes)msg->data;
	HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END Application */

