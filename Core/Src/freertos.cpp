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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	// micro-ROS configuration
	  rcl_ret_t rc;
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
	  rcl_subscription_t cmd_vel_subscriber;
	  rcl_subscription_t imu_subscriber;
	  rcl_subscription_t pressure_subscriber;
	  rcl_subscription_t temperature_subscriber;
	  // messages
	  nereo_interfaces__msg__CommandVelocity cmd_vel_msg;
	  sensor_msgs__msg__Imu imu_data_msg;
	  sensor_msgs__msg__FluidPressure fluid_pressure;
	  sensor_msgs__msg__Temperature water_temperature;

	  // PARAM SERVER


	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rc = rclc_support_init(&support, 0, NULL, &allocator);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // create node
	  rc = rclc_node_init_default(&node, "cubemx_node", "", &support);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  executor = rclc_executor_get_zero_initialized_executor();
	  rc = rclc_executor_init(&executor, &support.context, NUMBER_SUBS_TIMS_SRVS + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // PUBLISHERS
	  rclc_publisher_init_default(
	    &thruster_status_publisher,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, ThrusterStatuses),
	    "/thruster_status");

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

	  // CMD Vel sub
	  rc = rclc_subscription_init_default(
			  &cmd_vel_subscriber,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity),
			  "/nereo_cmd_vel");
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);
	  // initialize message memory
	  rc = !micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity), &cmd_vel_msg, default_conf);

	  rc = rclc_executor_add_subscription(
			  &executor, &cmd_vel_subscriber,
			  &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // SERVICES
	  rcl_service_t arm_disarm_srv_server;
	  std_srvs__srv__SetBool_Request set_arm_mode_reqin;
	  std_srvs__srv__SetBool_Response set_arm_mode_resout;
	  rc = rclc_service_init_default(
			  &arm_disarm_srv_server, &node,
			  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/set_rov_arm_mode");
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);
	  rc = rclc_executor_add_service(
			  &executor, &arm_disarm_srv_server, &set_arm_mode_reqin,
			  &set_arm_mode_resout, &arm_disarm_service_callback);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  rcl_service_t nav_mode_srv_server;
	  nereo_interfaces__srv__SetNavigationMode_Request set_navigation_mode_reqin;
	  nereo_interfaces__srv__SetNavigationMode_Response set_navigation_mode_resout;
	  rc = rclc_service_init_default(
			  &nav_mode_srv_server, &node,
			  ROSIDL_GET_SRV_TYPE_SUPPORT(nereo_interfaces, srv, SetNavigationMode), "/set_rov_navigation_mode");
	  rc = rclc_executor_add_service(
			  &executor, &nav_mode_srv_server, &set_navigation_mode_reqin,
			  &set_navigation_mode_resout, &set_nav_mode_service_callback);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  uint32_t pwm_output[8] = {1500};
	  arm_status pwm_computation_error = ARM_MATH_SUCCESS;
	  printf("Micro ROS initialization done without errors.\n");
	  while(1)
	  {
		uint32_t time_ms = HAL_GetTick();
		//printf("Free heap: %d.\n", xPortGetFreeHeapSize());
		// Spin executor once to receive requests and update messages
		rclc_executor_spin_some(&executor, 1000000);

	    if (rov_arm_mode == ROV_ARMED)
	    {
	    	switch (navigation_mode) {
	    		case NAVIGATION_MODE_MANUAL:
	    			pwm_computation_error = calculate_pwm(cmd_vel_msg.cmd_vel, pwm_output);
	    			break;
	    		case NAVIGATION_MODE_STABILIZE_FULL:
	    			pwm_computation_error = calculate_pwm_with_pid(cmd_vel_msg.cmd_vel, pwm_output,
	    					(Quaternion *)&imu_data_msg.orientation,
							(float *)&fluid_pressure.fluid_pressure);
	    			break;
	    		default:
	    			for(uint8_t i = 0; i < 8; i++) pwm_output[i] = 1500;
	    			break;
	    	}
	    	clamp_pwm_output(pwm_output, 8);
	    	// offset to compensate for the optocoupler offset
	    	for(int8_t i = 0; i < 8; i++) pwm_output[i] += OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	    	set_pwms(pwm_output);
	    } else set_pwm_idle();

	    // do not publish the offset by subtracting 50
	    for(uint8_t i = 0; i < 8; i++) thruster_status_msg.thruster_pwms[i] = pwm_output[i] - OPTOCOUPLER_INTRODUCED_OFFSET_uS;
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
void clamp_pwm_output(uint32_t pwms[], int N) {
	for(uint16_t i = 0; i < N; i++) {
		if (pwms[i] < PWM_MIN)
			pwms[i] = PWM_MIN;
		else if (pwms[i] > PWM_MAX)
			pwms[i] = PWM_MAX;
	}
}
void update_pid_constants(arm_pid_instance_f32 *pid, float32_t Kp, float32_t Ki, float32_t Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->A0 = Kp + Ki + Kd;
    pid->A1 = -Kp - 2 * Kd;
    pid->A2 = Kd;
}
void imu_subscription_callback(const void * msgin) {
	const sensor_msgs__msg__Imu * msg = (const sensor_msgs__msg__Imu *)msgin;
}
void cmd_vel_subscription_callback (const void * msgin) {

}
void arm_disarm_service_callback(const void * request_msg, void * response_msg) {
	std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) request_msg;
	std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) response_msg;

	// Handle request message and set the response message values
	rov_arm_mode = req_in->data ? ROV_ARMED : ROV_DISARMED;
	printf("%d: arm mode.\n", (int)rov_arm_mode);
	res_in->success = true;
	res_in->message.capacity = 2;
	res_in->message.size = strlen(empty_string);
	res_in->message.data = empty_string;
}
void set_nav_mode_service_callback(const void * request_msg, void * response_msg) {
	nereo_interfaces__srv__SetNavigationMode_Request * req_in = (nereo_interfaces__srv__SetNavigationMode_Request *) request_msg;
	nereo_interfaces__srv__SetNavigationMode_Response * res_in = (nereo_interfaces__srv__SetNavigationMode_Response *) response_msg;

	navigation_mode = (NavigationModes)req_in->navigation_mode;

	res_in->mode_after_set = navigation_mode;
	res_in->success = true;
}
/* USER CODE END Application */

