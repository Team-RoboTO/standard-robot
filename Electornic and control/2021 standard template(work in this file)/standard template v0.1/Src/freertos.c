/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "CAN_receive.h"
#include "tim.h"
#include <stdio.h>
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

extern rc_info_t rc;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId anothertaskHandle;
osThreadId shooterTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TstTask(void const * argument);
void shooter(void const * argument);

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

  /* definition and creation of anothertask */
  osThreadDef(anothertask, TstTask, osPriorityNormal, 0, 128);
  anothertaskHandle = osThreadCreate(osThread(anothertask), NULL);

  /* definition and creation of shooterTask */
  osThreadDef(shooterTask, shooter, osPriorityNormal, 0, 128);
  shooterTaskHandle = osThreadCreate(osThread(shooterTask), NULL);

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
		vTaskDelay(1);
	 }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TstTask */
/**
* @brief Function implementing the anothertask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TstTask */
void TstTask(void const * argument)
{
  /* USER CODE BEGIN TstTask */
	int16_t x=0;
	int16_t y=0;
	int16_t raw=0;
	uint16_t shooter;  // default pwm pulse width:1080~1920
	float set_spd[4];
	
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 20000,
									1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
  /* Infinite loop */
  for(;;)
  {
		x= (int16_t)(1000*rc.ch3/660);//0~1000
		y= (int16_t)(1000*rc.ch1/660);
		raw= (int16_t)(1000*rc.ch4/660);
		
		shooter=100*rc.ch2/660+1300;
		//µ×ÅÌ
		set_spd[0]=y-x+raw;
		set_spd[1]=y+x-raw;
		set_spd[2]=y-x-raw;
		set_spd[3]=y+x+raw;
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], motor_chassis[i].speed_rpm, set_spd[i]);
		}
		CAN_cmd_chassis(pid_spd[0].pos_out,pid_spd[1].pos_out,pid_spd[2].pos_out,pid_spd[3].pos_out);
		

		//ÔÆÌ¨
//		
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, shooter);//PA8 up and down
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1450);//PA9
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, shooter);//PA10
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, shooter);//PA11
		osDelay(1);

				//		

  }
  /* USER CODE END TstTask */
}

/* USER CODE BEGIN Header_shooter */
/**
* @brief Function implementing the shooterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shooter */
void shooter(void const * argument)
{
  /* USER CODE BEGIN shooter */
	float set_spd[3];
	int k=0;
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd_shooter[i], POSITION_PID, 20000, 20000,
									2.0f,	0.0f,	0.1f	);  //4 motos angular rate closeloop.
	}
  /* Infinite loop */
  for(;;)
  {
				//·¢Éä
		set_spd[0]=3000;
		set_spd[1]=-3000;
		set_spd[2]=50;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
		if (rc.sw1==1)
		{
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd_shooter[i], motor_chassis[i+4].speed_rpm, set_spd[i]);
			}
			if (k==0)
			{
				CAN_cmd_gimbal(pid_spd_shooter[0].pos_out,pid_spd_shooter[1].pos_out,0,0);
				if(motor_chassis[4].speed_rpm>=100 && motor_chassis[5].speed_rpm<=-100)
				{
					k=1;
				}
			}
			else
			{
//				if(motor_chassis[6].speed_rpm<=100)
//				{
//					CAN_cmd_gimbal(pid_spd_shooter[0].pos_out,pid_spd_shooter[1].pos_out,-pid_spd_shooter[2].pos_out,0);
//				}
				CAN_cmd_gimbal(pid_spd_shooter[0].pos_out,pid_spd_shooter[1].pos_out,pid_spd_shooter[2].pos_out,0);
			}
		}
		else if (rc.sw1!=1)
		{
			k=0;
			CAN_cmd_gimbal(0,0,0,0);
		}
    osDelay(50);
  }
  /* USER CODE END shooter */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
