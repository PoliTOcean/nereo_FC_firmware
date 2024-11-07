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
#include "usart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "micro_ros_utilities/type_utilities.h"

#include <std_msgs/msg/Int32.h>
#include <sensor_msgs/msg/Imu.h>
#include <sensor_msgs/msg/Joy.h>
#include <sensor_msgs/msg/Fluid_Pressure.h>
#include <sensor_msgs/msg/Temperature.h>
#include <nereo_interfaces/msg/thruster_statuses.h>
#include <std_srvs/srv/set_bool.h>
#include <nereo_interfaces/srv/set_navigation_mode.h>

#include "navigation/stabilize_mode.h"
#include "navigation/navigation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ROS2_OK,
	ROS2_WARNING,
	ROS2_ERROR,
	ROS2_STALE
} RosErrors;

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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// combined number of timers and subscriptions: needed for initializing the executor: make sure to udpate this if adding any timer or subscription
#define NUMBER_SUBS_TIMS 4
#define DEFAULT_TASK_FREQUENCY_HZ 100
#define TS_DEFAULT_TASK_MS (1000/DEFAULT_TASK_FREQUENCY_HZ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile RovArmModes rov_arm_mode = ROV_DISARMED;
volatile NavigationModes navigation_mode = NAVIGATION_MODE_MANUAL;
char empty_string[] = "";
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
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void imu_subscription_callback (const void * msgin);
void joystick_subscription_callback (const void * msgin);
void pressure_subscription_callback (const void * msgin);
void temperature_subscription_callback (const void * msgin);

void set_pwm_idle();
void set_pwms(uint32_t pwms[8]);
void constrain_pwm_output(uint32_t *, int);

void joystick_msg_to_cmd_vel_array(const sensor_msgs__msg__Joy *, float[6]);

void arm_disarm_service_callback(const void *, void *);
void set_nav_mode_service_callback(const void *, void *);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	// micro-ROS configuration
	  rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart1,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;


	  // PUBLISHERS
	  rcl_publisher_t thruster_status_publisher;

	  // messages
	  nereo_interfaces__msg__ThrusterStatuses thruster_status_msg;

	  // SUBSCRIBERS
	  rcl_subscription_t joystick_subscriber;
	  rcl_subscription_t imu_subscriber;
	  rcl_subscription_t pressure_subscriber;
	  rcl_subscription_t temperature_subscriber;
	  // messages
	  sensor_msgs__msg__Joy joystick_input_msg;
	  sensor_msgs__msg__Imu imu_data_msg;
	  sensor_msgs__msg__FluidPressure fluid_pressure;
	  sensor_msgs__msg__Temperature water_temperature;

	  // SERVICES

	  // PARAM SERVER

	  rcl_ret_t rc;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rc = rclc_support_init(&support, 0, NULL, &allocator);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // create node
	  rc = rclc_node_init_default(&node, "cubemx_node", "", &support);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  executor = rclc_executor_get_zero_initialized_executor();
	  rc = rclc_executor_init(&executor, &support.context, NUMBER_SUBS_TIMS, &allocator);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // PUBLISHERS
	  rclc_publisher_init_default(
	    &thruster_status_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, ThrusterStatuses),
	    "/thruster_status");
	  thruster_status_msg.thrusters_pwms[0] = 0;

	  // SUBSCRIBERS
	  static micro_ros_utilities_memory_conf_t default_conf = {0};

	  // IMU sub
	  rc = rclc_subscription_init_default(
			  &imu_subscriber,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			  "/imu_data");
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);
	  // initialize message memory
	  rc = !micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), &imu_data_msg, default_conf);

	  rc = rclc_executor_add_subscription(
			  &executor, &imu_subscriber,
			  &imu_data_msg, &imu_subscription_callback, ON_NEW_DATA);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // JOY sub
	  rc = rclc_subscription_init_default(
			  &joystick_subscriber,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
			  "/joy");
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);
	  // initialize message memory
	  rc = !micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy), &joystick_input_msg, default_conf);

	  rc = rclc_executor_add_subscription(
			  &executor, &joystick_subscriber,
			  &joystick_input_msg, &joystick_subscription_callback, ON_NEW_DATA);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  uint32_t pwm_output[8] = {1500};
	  arm_status pwm_computation_error = ARM_MATH_SUCCESS;
	  float joy_input[6] = {0};

	  while(1)
	  {
		uint32_t time_ms = HAL_GetTick();
		printf("Free heap: %d.\n", xPortGetFreeHeapSize());
		// Spin executor once to receive requests and update messages
		rclc_executor_spin_some(&executor, 1000000);

	    if (rov_arm_mode == ROV_ARMED)
	    {
	    	joystick_msg_to_cmd_vel_array(&joystick_input_msg, joy_input);
	    	switch (navigation_mode) {
	    		case NAVIGATION_MODE_MANUAL:
	    			pwm_computation_error = calculate_pwm(joy_input, pwm_output);
	    			break;
	    		case NAVIGATION_MODE_STABILIZE_FULL:
	    			pwm_computation_error = calculate_pwm_with_pid(joy_input, pwm_output,
	    					(Quaternion *)&imu_data_msg.orientation,
							(float *)&fluid_pressure.fluid_pressure);
	    			break;
	    		default:
	    			for(uint8_t i = 0; i < 8; i++) pwm_output[i] = 1500;
	    			break;
	    	}
	    	constrain_pwm_output(pwm_output, 8);
	    	set_pwms(pwm_output);
	    } else set_pwm_idle();

	    for(uint8_t i = 0; i < 8; i++) thruster_status_msg.thrusters_pwms[i] = pwm_output[i];
	    rc = rcl_publish(&thruster_status_publisher, &thruster_status_msg, NULL);
	    if(rc!=RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);

	    uint32_t elapsed_time = HAL_GetTick() - time_ms;
	    if (elapsed_time < TS_DEFAULT_TASK_MS) osDelay(TS_DEFAULT_TASK_MS - elapsed_time);
	  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void inline set_pwms(uint32_t pwms[8])
{
	TIM2 -> CCR1 = pwms[0];
	TIM2 -> CCR2 = pwms[1];
	TIM2 -> CCR3 = pwms[2];
	TIM2 -> CCR4 = pwms[3];

	// vertical thrusters
	TIM3 -> CCR1 = pwms[4];
	TIM3 -> CCR2 = pwms[5];
	TIM3 -> CCR3 = pwms[6];
	TIM3 -> CCR4 = pwms[7];
}
void inline set_pwm_idle()
{
	TIM2 -> CCR1 = PWM_IDLE;
	TIM2 -> CCR2 = PWM_IDLE;
	TIM2 -> CCR3 = PWM_IDLE;
	TIM2 -> CCR4 = PWM_IDLE;

	// vertical thrusters
	TIM3 -> CCR1 = PWM_IDLE;
	TIM3 -> CCR2 = PWM_IDLE;
	TIM3 -> CCR3 = PWM_IDLE;
	TIM3 -> CCR4 = PWM_IDLE;
}
void constrain_pwm_output(uint32_t pwms[], int N) {
	for(uint16_t i = 0; i < N; i++) {
		if (pwms[i] < PWM_MIN)
			pwms[i] = PWM_MIN;
		else if (pwms[i] > PWM_MAX)
			pwms[i] = PWM_MAX;
	}
}
void joystick_msg_to_cmd_vel_array(const sensor_msgs__msg__Joy * joystick_input_msg, float joy_input_array[6]) {
	joy_input_array[0] = joystick_input_msg->axes.data[0]; // sway
	joy_input_array[1] =joystick_input_msg->axes.data[1]; // forward
	joy_input_array[2] =joystick_input_msg->axes.data[3]; // heave
	joy_input_array[6] =joystick_input_msg->axes.data[2]; // yaw
}
void imu_subscription_callback(const void * msgin) {
	const sensor_msgs__msg__Imu * msg = (const sensor_msgs__msg__Imu *)msgin;
}
void joystick_subscription_callback (const void * msgin) {
	const sensor_msgs__msg__Joy * msg = (const sensor_msgs__msg__Joy *)msgin;
}
void arm_disarm_service_callback(const void * request_msg, void * response_msg) {
	std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) request_msg;
	std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) response_msg;

	// Handle request message and set the response message values
	rov_arm_mode = req_in->data ? ROV_ARMED : ROV_DISARMED;
	res_in->success = true;
	res_in->message.capacity = 2;
	res_in->message.size = strlen(empty_string);
	res_in->message.data = empty_string;
}
void set_nav_mode_service_callback(const void * request_msg, void * response_msg) {
	nereo_interfaces__srv__SetNavigationMode_Request * req_in = (nereo_interfaces__srv__SetNavigationMode_Request *) request_msg;
	nereo_interfaces__srv__SetNavigationMode_Response * res_in = (nereo_interfaces__srv__SetNavigationMode_Response *) response_msg;

	navigation_mode = req_in->navigation_mode;

	res_in->mode_after_set = navigation_mode;
	res_in->success = true;
	res_in->message.capacity = 2;
	res_in->message.size = strlen(empty_string);
	res_in->message.data = empty_string;
}
/* USER CODE END Application */

