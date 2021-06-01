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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int16_t power=0;
extern motor_measure_t motor_chassis[7];
extern rc_info_t rc;//remote controller
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
osThreadId_t shooterTaskHandle;
osThreadId_t chasisTaskHandle;
osThreadId_t tripodTaskHandle;
osThreadId_t LEDTaskHandle;
osThreadId_t powerdispTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void shooter_task(void *argument);
void chassis_task(void *argument);
void tripod_task(void *argument);
void LED_task(void *argument);
void Power_display_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */
osKernelInitialize();

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
  /* definition and creation of shooterTask */
  const osThreadAttr_t shooterTask_attributes = {
    .name = "shooterTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  shooterTaskHandle = osThreadNew(shooter_task, NULL, &shooterTask_attributes);

  /* definition and creation of chasisTask */
  const osThreadAttr_t chasisTask_attributes = {
    .name = "chasisTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  chasisTaskHandle = osThreadNew(chassis_task, NULL, &chasisTask_attributes);

  /* definition and creation of tripodTask */
  const osThreadAttr_t tripodTask_attributes = {
    .name = "tripodTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256
  };
  tripodTaskHandle = osThreadNew(tripod_task, NULL, &tripodTask_attributes);

  /* definition and creation of LEDTask */
  const osThreadAttr_t LEDTask_attributes = {
    .name = "LEDTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256
  };
  LEDTaskHandle = osThreadNew(LED_task, NULL, &LEDTask_attributes);

  /* definition and creation of powerdispTask */
  const osThreadAttr_t powerdispTask_attributes = {
    .name = "powerdispTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 256
  };
  powerdispTaskHandle = osThreadNew(Power_display_task, NULL, &powerdispTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_shooter_task */
/**
  * @brief  Function implementing the shooterTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_shooter_task */
void shooter_task(void *argument)
{
  /* USER CODE BEGIN shooter_task */
	PID_TypeDef motor_2006;
	PID_TypeDef motor_3508_left;
	PID_TypeDef motor_3508_right;
	
	u16 init_shoot_delay_count=0;
	
	/*pid initialization*/
	pid_init(&motor_2006);
	pid_init(&motor_3508_left);
	pid_init(&motor_3508_right);
	
	motor_2006.f_param_init(&motor_2006,PID_Speed,16384,5000,10,0,8000,0,1.5,0.1,0.0);
	motor_3508_left.f_param_init(&motor_3508_left,PID_Speed,20000,20000,10,0,8000,0,4.0,0.0,0.0);
	motor_3508_right.f_param_init(&motor_3508_right,PID_Speed,20000,20000,10,0,8000,0,4.0,0.0,0.0);
  /* Infinite loop */
  for(;;)
  {
		/*设置发射和旋转的速度*/
		switch(rc.sw1)
		{
			case 1: 
				motor_2006.target=0;
				motor_3508_left.target=0;
				motor_3508_right.target=0;break;
			case 3: 
				motor_2006.target=-3000;
				motor_3508_left.target=-3000;
				motor_3508_right.target=3000;break;
			case 2: 
				motor_2006.target=-5000;
				motor_3508_left.target=-3000;
				motor_3508_right.target=3000;break;
		}
		
//		motor_3508_right.target=-100;
		
		/*计算PID输出值*/
		motor_2006.f_cal_pid(&motor_2006,motor_chassis[4].speed_rpm);
		motor_3508_left.f_cal_pid(&motor_3508_left,motor_chassis[5].speed_rpm);
		motor_3508_right.f_cal_pid(&motor_3508_right,motor_chassis[6].speed_rpm);
		
//		if(init_shoot_delay_count>=1000)
//		{
//			init_shoot_delay_count=1000;
//			CAN_cmd_gimbal(motor_2006.output,motor_3508_left.output,motor_3508_right.output,0);
//		}
//		else
//		{
//			CAN_cmd_gimbal(0,motor_3508_left.output,motor_3508_right.output,0);
//		}
//经过测试是可以用的
//		CAN_cmd_gimbal(motor_2006.output,motor_3508_left.output,motor_3508_right.output,0);
			if (init_shoot_delay_count<=5000)
			{
				CAN_cmd_gimbal(0,motor_3508_left.output,motor_3508_right.output,0);
				init_shoot_delay_count++;
			}
			else
			{
				CAN_cmd_gimbal(motor_2006.output,motor_3508_left.output,motor_3508_right.output,0);
				if (rc.sw1==1)
				{
					init_shoot_delay_count=0;
				}
			}
    osDelay(1);
  }
  /* USER CODE END shooter_task */
}

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chasisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */

	PID_TypeDef chassis_id1;
	PID_TypeDef chassis_id2;
	PID_TypeDef chassis_id3;
	PID_TypeDef chassis_id4;
	
	int16_t x=0;
	int16_t y=0;
	int16_t raw=0;
	
	//底盘pid初始化
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5,GPIO_PIN_SET);
	
	pid_init(&chassis_id1);
	pid_init(&chassis_id2);
	pid_init(&chassis_id3);
	pid_init(&chassis_id4);
	
	chassis_id1.f_param_init(&chassis_id1,PID_Speed,20000,20000,10,0,8000,0,2.0,0.0,0.1);
	chassis_id2.f_param_init(&chassis_id2,PID_Speed,20000,20000,10,0,8000,0,2.0,0.0,0.1);
	chassis_id3.f_param_init(&chassis_id3,PID_Speed,20000,20000,10,0,8000,0,2.0,0.0,0.1);
	chassis_id4.f_param_init(&chassis_id4,PID_Speed,20000,20000,10,0,8000,0,2.0,0.0,0.1);
	
  /* Infinite loop */
  for(;;)
  {

		x= (int16_t)(1000*rc.ch3/660);//0~1000
		y= (int16_t)(1000*rc.ch1/660);
		raw= (int16_t)(1000*rc.ch4/660);
		
		//底盘麦轮运动解算
		chassis_id1.target=y-x+raw;
		chassis_id2.target=y+x-raw;
		chassis_id3.target=y-x-raw;
		chassis_id4.target=y+x+raw;
		
		chassis_id1.f_cal_pid(&chassis_id1,motor_chassis[0].speed_rpm);
		chassis_id2.f_cal_pid(&chassis_id2,motor_chassis[1].speed_rpm);
		chassis_id3.f_cal_pid(&chassis_id3,motor_chassis[2].speed_rpm);
		chassis_id4.f_cal_pid(&chassis_id4,motor_chassis[3].speed_rpm);
		
		CAN_cmd_chassis(chassis_id1.output, chassis_id2.output, chassis_id3.output, chassis_id4.output);
	
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_tripod_task */
/**
* @brief Function implementing the tripodTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tripod_task */
void tripod_task(void *argument)
{
  /* USER CODE BEGIN tripod_task */
  /* Infinite loop */
  for(;;)
  {
			/**todo:
		 *right switch==0 absolute
     *right switch==1 relative
     *right switvh==2 static
		 *速度环+PWM舵机控制
     */
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1450);//PA8 up and down
//    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1450);//PA9
		//CAN 2 12 速度环
//		switch(rc.sw2){
//			case 1: 
//				//ist1080
//				break;
//			case 2: 
//				//mpu6500
//				break;
//			case 3: 
//				
//				break;
//		}
    osDelay(1);
  }
  /* USER CODE END tripod_task */
}

/* USER CODE BEGIN Header_LED_task */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_task */
void LED_task(void *argument)
{
  /* USER CODE BEGIN LED_task */
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
    osDelay(100);
  }
  /* USER CODE END LED_task */
}

/* USER CODE BEGIN Header_Power_display_task */
/**
* @brief Function implementing the powerdispTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Power_display_task */
void Power_display_task(void *argument)
{
  /* USER CODE BEGIN Power_display_task */
	/**
	 * 用ADC采样电源电压，显示电源电量百分比（做不到）
	 * C板可以直接采样电源电压用于计算但是A板需要加一块板子用来测量电源电压
	 */
	int16_t voltage=24;
	u16 motor_ID=0;
	int16_t sum_current;

  /* Infinite loop */
  for(;;)
  {
		sum_current=0;
		motor_ID=0;
		for(;motor_ID<=7;motor_ID++){
			if(motor_chassis[motor_ID].given_current>=0)
				sum_current=sum_current+motor_chassis[motor_ID].given_current;
			else
				sum_current=sum_current-motor_chassis[motor_ID].given_current;
		}
		power=voltage*sum_current;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_2);
    osDelay(50);
  }
  /* USER CODE END Power_display_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
