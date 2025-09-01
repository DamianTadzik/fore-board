/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can-not/can_not.h"

#include "task_can_rx.h"
#include "task_adc.h"
#include "task_servo_control.h"
#include "task_servo_power_monitor.h"
#include "task_range_meas.h"
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
osThreadId_t task_can_rx_handle;
const osThreadAttr_t task_can_rx_attributes = {
  .name = "task_can_rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_adc_handle;
const osThreadAttr_t task_adc_attributes = {
  .name = "task_adc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_servo_control_handle;
const osThreadAttr_t task_servo_control_attributes = {
  .name = "task_servo_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_servo_power_monitor_handle;
const osThreadAttr_t task_servo_power_monitor_attributes = {
  .name = "task_servo_power_monitor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t task_range_meas_handle;
const osThreadAttr_t task_range_meas_attributes = {
  .name = "task_range_meas",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	size_t res = xPortGetFreeHeapSize();
	UNUSED(res);

	cant_freertos_init();
	res = xPortGetFreeHeapSize();
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
  res = xPortGetFreeHeapSize();

  task_can_rx_handle = osThreadNew(task_can_rx, NULL, &task_can_rx_attributes);
  res = xPortGetFreeHeapSize();

  task_adc_handle = osThreadNew(task_adc, NULL, &task_adc_attributes);
  res = xPortGetFreeHeapSize();

  task_servo_control_handle = osThreadNew(task_servo_control, NULL, &task_servo_control_attributes);
  res = xPortGetFreeHeapSize();

  task_servo_power_monitor_handle = osThreadNew(task_servo_power_monitor, NULL, &task_servo_power_monitor_attributes);
  res = xPortGetFreeHeapSize();

  task_range_meas_handle = osThreadNew(task_range_meas, NULL, &task_range_meas_attributes);
  res = xPortGetFreeHeapSize();

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


volatile uint32_t task_can_rx_alive;
volatile uint32_t task_adc_alive;
volatile uint32_t task_servo_control_alive;
volatile uint32_t task_servo_power_monitor_alive;
volatile uint32_t task_range_meas_alive;

volatile UBaseType_t task_can_rx_high_watermark;
volatile UBaseType_t task_adc_high_watermark;
volatile UBaseType_t task_servo_control_high_watermark;
volatile UBaseType_t task_servo_power_monitor_high_watermark;
volatile UBaseType_t task_range_meas_high_watermark;

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIO_LED_GPIO_Port, GPIO_LED_Pin);

	  task_can_rx_high_watermark           = uxTaskGetStackHighWaterMark((TaskHandle_t)task_can_rx_handle);
	  task_adc_high_watermark              = uxTaskGetStackHighWaterMark((TaskHandle_t)task_adc_handle);
	  task_servo_control_high_watermark    = uxTaskGetStackHighWaterMark((TaskHandle_t)task_servo_control_handle);
	  task_servo_power_monitor_high_watermark = uxTaskGetStackHighWaterMark((TaskHandle_t)task_servo_power_monitor_handle);
	  task_range_meas_high_watermark       = uxTaskGetStackHighWaterMark((TaskHandle_t)task_range_meas_handle);

	  osDelay(200);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

