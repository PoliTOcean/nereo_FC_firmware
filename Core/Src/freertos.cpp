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
//ControlSystem controllers[3];
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
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  } else HAL_IWDG_Refresh(&hiwdg);

	  // micro-ROS app

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;

	  // PID PARAM SERVER
	  rclc_parameter_server_t pid_param_server;

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

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rc = rclc_support_init(&support, 0, NULL, &allocator);
	  if (rc != RCL_RET_OK) printf("Error support init.\n");
	  else HAL_IWDG_Refresh(&hiwdg);

	  // create node
	  rc = rclc_node_init_default(&node, "fc_node", "", &support);
	  if (rc != RCL_RET_OK) printf("Error node init\n");
	  else HAL_IWDG_Refresh(&hiwdg);

	  executor = rclc_executor_get_zero_initialized_executor();
	  rc = rclc_executor_init(&executor, &support.context, NUMBER_SUBS_TIMS_SRVS + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);
	  if (rc != RCL_RET_OK) printf("Error executor init.\n");
	  else HAL_IWDG_Refresh(&hiwdg);

	  // PUBLISHERS
	  rclc_publisher_init_best_effort(
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
	  if (rc != RCL_RET_OK) printf("Error imu sub init.\n");
	  // initialize message memory
	  rc = !micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), &imu_data_msg, default_conf);

	  rc = rclc_executor_add_subscription(
			  &executor, &imu_subscriber,
			  &imu_data_msg, &imu_subscription_callback, ON_NEW_DATA);
	  if (rc != RCL_RET_OK) printf("Error executor add imu sub.\n");

	  // CMD Vel sub
	  rc = rclc_subscription_init_default(
			  &cmd_vel_subscriber,
			  &node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity),
			  "/nereo_cmd_vel");
	  if (rc != RCL_RET_OK) printf("Error cmdvel sub init.\n");
	  // initialize message memory
	  rc = !micro_ros_utilities_create_message_memory(ROSIDL_GET_MSG_TYPE_SUPPORT(nereo_interfaces, msg, CommandVelocity), &cmd_vel_msg, default_conf);

	  rc = rclc_executor_add_subscription(
			  &executor, &cmd_vel_subscriber,
			  &cmd_vel_msg, &cmd_vel_subscription_callback, ON_NEW_DATA);
	  if (rc != RCL_RET_OK) printf("Error executor add cmdvel sub.\n");

	  // SERVICES
	  rcl_service_t arm_disarm_srv_server;
	  std_srvs__srv__SetBool_Request set_arm_mode_reqin;
	  std_srvs__srv__SetBool_Response set_arm_mode_resout;
	  rc = rclc_service_init_default(
			  &arm_disarm_srv_server, &node,
			  ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/set_rov_arm_mode");
	  if (rc != RCL_RET_OK) printf("Error armmode srv init.\n");
	  rc = rclc_executor_add_service(
			  &executor, &arm_disarm_srv_server, &set_arm_mode_reqin,
			  &set_arm_mode_resout, &arm_disarm_service_callback);
	  if (rc != RCL_RET_OK) printf("Error exec add armmode srv.\n");

	  rcl_service_t nav_mode_srv_server;
	  nereo_interfaces__srv__SetNavigationMode_Request set_navigation_mode_reqin;
	  nereo_interfaces__srv__SetNavigationMode_Response set_navigation_mode_resout;
	  rc = rclc_service_init_default(
			  &nav_mode_srv_server, &node,
			  ROSIDL_GET_SRV_TYPE_SUPPORT(nereo_interfaces, srv, SetNavigationMode), "/set_rov_navigation_mode");
	  rc = rclc_executor_add_service(
			  &executor, &nav_mode_srv_server, &set_navigation_mode_reqin,
			  &set_navigation_mode_resout, &set_nav_mode_service_callback);
	  if (rc != RCL_RET_OK) printf("Error exec addset_nav_mode srv.\n");

	  /* PID PARAM SERVER INIT AND CONFIG
	  const rclc_parameter_options_t pid_param_server_options = {
			  .notify_changed_over_dds = false,
			  .max_params = 18,
			  .allow_undeclared_parameters = false,
			  .low_mem_mode = true };
	  //rc = rclc_parameter_server_init_with_option(&pid_param_server, &node, &pid_param_server_options);
	  rc = rclc_parameter_server_init_default(&pid_param_server, &node);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);

	  // parameters
	  rc = rclc_add_parameter(&pid_param_server, "pid0_K0", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid0_K1", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid0_K2", RCLC_PARAMETER_DOUBLE);

	  rc = rclc_add_parameter(&pid_param_server, "pid1_K0", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid1_K1", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid1_K2", RCLC_PARAMETER_DOUBLE);

	  rc = rclc_add_parameter(&pid_param_server, "pid2_K0", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid2_K1", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid2_K2", RCLC_PARAMETER_DOUBLE);

	  rc = rclc_add_parameter(&pid_param_server, "pid3_K0", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid3_K1", RCLC_PARAMETER_DOUBLE);
	  rc = rclc_add_parameter(&pid_param_server, "pid3_K2", RCLC_PARAMETER_DOUBLE);

	  rc = rclc_executor_add_parameter_server(&executor, &pid_param_server, on_parameter_changed);
	  if (rc != RCL_RET_OK) printf("Error (line %d)\n", __LINE__);*/

	  // END MICRO ROS INIT
	  printf("Micro ROS initialization done.\n");

	  uint32_t pwm_output[8] = {1500};
	  arm_status pwm_computation_error = ARM_MATH_SUCCESS;

	  // PID INIT

	  float kps[PID_NUMBER] = {0, 0, 0, 0};
	  float kis[PID_NUMBER] = {0, 0, 0, 0};
	  float kds[PID_NUMBER] = {0, 0, 0, 0};
	  //init_pids(kps, kis, kds); // sw pids
	  //controllers_init(); // cs controllers

	  while(1)
	  {
		uint32_t time_ms = HAL_GetTick();
		//printf("Free heap: %d.\n", xPortGetFreeHeapSize());
		// Spin executor once to receive requests and update messages
		rc = rclc_executor_spin_some(&executor, 10000000);

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
	    		case NAVIGATION_MODE_STABILIZE_CS:
	    			//pwm_computation_error = calculate_pwm_cs_controller(cmd_vel_msg.cmd_vel, pwm_output,
	    			//		(Quaternion *)&imu_data_msg.orientation,
					//		(float *)&fluid_pressure.fluid_pressure);
	    			//break;
	    		default:
	    			for(uint8_t i = 0; i < 8; i++) pwm_output[i] = 1500;
	    			break;
	    	}
	    	clamp_pwm_output(pwm_output, 8);
	    	set_pwms(pwm_output);
	    } else set_pwm_idle();

	    for(uint8_t i = 0; i < 8; i++) thruster_status_msg.thruster_pwms[i] = pwm_output[i];
	    rc = rcl_publish(&thruster_status_publisher, &thruster_status_msg, NULL);
	    if(rc!=RCL_RET_OK) printf("Error publishing (line %d)\n", __LINE__);
	    else HAL_IWDG_Refresh(&hiwdg);

	    uint32_t elapsed_time = HAL_GetTick() - time_ms;
	    if (elapsed_time < TS_DEFAULT_TASK_MS) osDelay(TS_DEFAULT_TASK_MS - elapsed_time);
	  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void inline set_pwms(uint32_t pwms[8])
{
	// HERE THE PWM Channel - Thruster relation is defined
	TIM3 -> CCR3 = pwms[0] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR1 = pwms[1] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR4 = pwms[2] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR2 = pwms[3] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	// VERTICAL THRUSTERS
	TIM2 -> CCR2 = pwms[4] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR3 = pwms[5] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR4 = pwms[6] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR1 = pwms[7] + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
}
void inline set_pwm_idle()
{
	TIM2 -> CCR1 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR2 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR3 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM2 -> CCR4 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR1 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR2 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR3 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
	TIM3 -> CCR4 = PWM_IDLE + OPTOCOUPLER_INTRODUCED_OFFSET_uS;
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
	HAL_IWDG_Refresh(&hiwdg);
}
void arm_disarm_service_callback(const void * request_msg, void * response_msg) {
	HAL_IWDG_Refresh(&hiwdg);
	std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) request_msg;
	std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) response_msg;
	rov_arm_mode = req_in->data ? ROV_ARMED : ROV_DISARMED;
	printf("%d: arm mode.\n", (int)rov_arm_mode);
	res_in->success = true;
	res_in->message.capacity = 2;
	res_in->message.size = strlen(empty_string);
	res_in->message.data = empty_string;
}
void set_nav_mode_service_callback(const void * request_msg, void * response_msg) {
	HAL_IWDG_Refresh(&hiwdg);
	nereo_interfaces__srv__SetNavigationMode_Request * req_in = (nereo_interfaces__srv__SetNavigationMode_Request *) request_msg;
	nereo_interfaces__srv__SetNavigationMode_Response * res_in = (nereo_interfaces__srv__SetNavigationMode_Response *) response_msg;
	navigation_mode = (NavigationModes)req_in->navigation_mode;
	res_in->mode_after_set = navigation_mode;
	res_in->success = true;
}
bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void) context;
  HAL_IWDG_Refresh(&hiwdg);
  if (old_param == NULL && new_param == NULL) {
    printf("Callback error, both parameters are NULL\n");
    return false;
  }

  if (old_param == NULL) {
	  return false;
  } else if (new_param == NULL) {
	  return false;
  } else {
    printf("Parameter %s modified.", old_param->name.data);
    if(old_param->name.data[3] != '0' || old_param->name.data[3] != '1' || old_param->name.data[3] != '2' || old_param->name.data[3] != '3')
    	return false;
    if(old_param->name.data[6] != '0' || old_param->name.data[6] != '1' || old_param->name.data[6] != '2' || old_param->name.data[6] != '3')
        	return false;
    int n_k = old_param->name.data[6] - '0';
    int n_p = old_param->name.data[3] - '0';
    float32_t new_param_f = new_param->value.double_value;
    switch(n_k) {
    case 0: update_pid_constants(&pids[n_p], &new_param_f, NULL, NULL); break;
    case 1: update_pid_constants(&pids[n_p], NULL, &new_param_f, NULL); break;
    case 2: update_pid_constants(&pids[n_p], NULL, NULL, &new_param_f); break;
    }
  }

  return true;
}
/* USER CODE END Application */

