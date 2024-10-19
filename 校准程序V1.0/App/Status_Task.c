/** 
  * @file     Status_Task.c
  * @version  v1.0
  * @date     2020.1.3
	*
  * @brief    状态检测
	*
  *	@author   YY
  *
  */

#include "Status_Task.h"
#include "tim.h"
#include "BMI088_Read.h"
#include "IST8310_Read.h"
#include "IMU_AHRSupdate.h"
#include "Comm_Task.h"
#include "bsp_imu.h"
uint8_t Data_Ready_Flag=0;	// 0:数据未就绪	 1：数据已就绪
uint8_t led_dir=0;	//LED呼吸方向
uint16_t led_pwm=0;

/**
  * @brief status_task
  * @param     
  * @attention  
	* @note  
  */
void status_task(void)
{
    err = fabs(Temperature-calibration_temperature);
	if(err > 0.2f)
        imu_mode=temperature_error;
	else 
        imu_mode=normal;

    if(imu_mode==temperature_error)
    {	
        if(led_dir==0)
		{
			led_pwm+=10/(err-0.2f);
			if(led_pwm>=1000)
				led_dir=1;
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);   
		}
		else
		{
			led_pwm-=10/(err-0.2f);
			if(led_pwm<=0)
                led_dir=0;
            HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
		}
	}
	Status_LED_PWM_Set(led_pwm);	//数据就绪后变为呼吸灯
    HAL_Delay(1);
}

void finish_LED(void)
{
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
    if(led_dir==0)
    {
        led_pwm+=10;
        if(led_pwm>=1000)
            led_dir=1;
    }
    else
    {
        led_pwm-=10;
        if(led_pwm<=0)
            led_dir=0;
    }
	Status_LED_PWM_Set(led_pwm);	//数据就绪后变为呼吸灯
    HAL_Delay(1); 
}

void Data_Not_Ready(void)
{
	Data_Ready_Flag=0;
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
}

/**
  * @brief Data_Ready
  * @param     
  * @attention  
	* @note  
  */
void Data_Ready(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
	HAL_Delay(200);
	Data_Ready_Flag=1;
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}
