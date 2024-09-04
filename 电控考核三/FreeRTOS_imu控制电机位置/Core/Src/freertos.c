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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

osThreadId LEDBlinkTaskHandle;
osThreadId LEDBreathTaskHandle;
osThreadId IMUTaskHandle;
osThreadId MotorTaskHandle;


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, Start_Task, osPriorityRealtime, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_Task */
#include "tim.h"
#include "pid.h"
#include "bsp_imu.h"

void LED_Blink_Task(void const * argument);
void LED_Breath_Task(void const * argument);
void IMU_Task(void const * argument);
void Motor_Task(void const * argument);

/**
* @brief Function implementing the StartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task */
void Start_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Task */
  /* Infinite loop */
  taskENTER_CRITICAL();
	
  osThreadDef(LEDBlinkTask, LED_Blink_Task, osPriorityNormal, 0, 128);
  LEDBlinkTaskHandle = osThreadCreate(osThread(LEDBlinkTask), NULL);

//  osThreadDef(LEDBreathTask, LED_Breath_Task, osPriorityNormal, 0, 128);
//  LEDBreathTaskHandle = osThreadCreate(osThread(LEDBreathTask), NULL);
	
  osThreadDef(IMUTask, IMU_Task, osPriorityNormal, 0, 128);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

	osThreadDef(MotorTask, Motor_Task, osPriorityNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

	osThreadTerminate(StartTaskHandle);
	
	taskEXIT_CRITICAL();
	
  /* USER CODE END Start_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


void LED_Blink_Task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t prevTime = osKernelSysTick();
  /* Infinite loop */
  while(1)
	{
		osDelayUntil(&prevTime,500);
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
	}
  /* USER CODE END StartDefaultTask */
}

//void LED_Breath_Task(void const * argument)
//{
//  /* USER CODE BEGIN StartDefaultTask */
//	uint32_t prevTime = osKernelSysTick();
//  /* Infinite loop */
//	int light =0;
//	int flag =1;
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//  while(1)
//	{
//		osDelayUntil(&prevTime,2);
//		if(flag)
//		{
//			light+=2;
//		}
//		else
//		{
//			light-=2;
//		}
//		if(light>=1000)
//		{
//			light =999;
//			flag=0;
//		}
//		else if(light<=0)
//		{
//			light =0;
//			flag=1;
//		}
//		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,light);
//	}
//  /* USER CODE END StartDefaultTask */
//}

void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t prevTime = osKernelSysTick();
	static float temp1=0;
	static float temp2=0;
  /* Infinite loop */
  while(1)
	{
		osDelayUntil(&prevTime,10);
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
		
		temp2=temp1;
		
		temp1=imu.pit;
		if(temp1-temp2>1) //把小于1的变化值当作干扰滤掉
		{
			imu.pit=temp1;
		}
		else
		{
			imu.pit=temp2;
		}
	}
  /* USER CODE END StartDefaultTask */
}


#include "bsp_can.h"

pid_struct_t motor_pid[7];
moto_info_t motor_info[7];

float angle;
void Motor_Task(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	uint32_t prevTime = osKernelSysTick();
  /* Infinite loop */
	pid_init(&motor_pid[0], 10, 0, 0.5, 9000, 9000); 
	
//			
  while(1)
	{
		osDelayUntil(&prevTime,50);
		
		angle=(imu.pit+90)/180*8192; //将imupit值（-90~90）转为（0~180），等比对应于电机的0~8191
    motor_info[0].set_voltage = pid_calc(&motor_pid[0],angle , motor_info[0].rotor_angle);
    set_motor_voltage(0, 
                      motor_info[0].set_voltage, 
                      0, 
                      0, 
                      0);
		
//		
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE END Application */
