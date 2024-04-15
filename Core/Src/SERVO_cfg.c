/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations
	{
	    GPIOA,
		GPIO_PIN_0,
		TIM5,
		&TIM5->CCR1,
		TIM_CHANNEL_1,
		168000000,
		0.5,
		2.5
	},
	// Servo Motor 2 Configurations
	{
	    GPIOA,
		GPIO_PIN_1,
		TIM5,
		&TIM5->CCR2,
		TIM_CHANNEL_2,
		168000000,
		0.5,
		2.5
	},
    // Servo Motor 3 Configurations
   {
	    GPIOB,
		GPIO_PIN_14,
		TIM12,
		&TIM12->CCR1,
		TIM_CHANNEL_1,
		168000000,
		0.5,
		2.5
	},
	// Servo Motor 4 Configurations
    {
	    GPIOB,
		GPIO_PIN_15,
		TIM12,
		&TIM12->CCR2,
		TIM_CHANNEL_2,
		168000000,
		0.5,
		2.5
	}
};
