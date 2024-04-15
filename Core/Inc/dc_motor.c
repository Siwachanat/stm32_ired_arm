/*
 * dc_motor.c
 *
 *  Created on: Feb 27, 2024
 *      Author: watch
 */


#include "dc_motor.h"
#include "main.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[2] =
{
	// DC MOTOR 1 Configurations
    {
	    GPIOA,
		GPIOA,
		GPIO_PIN_10,//CH3
		GPIO_PIN_11,//CH4
		TIM1,
		TIM_CHANNEL_3,
		TIM_CHANNEL_4,
		84,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	},
	// DC MOTOR 2 Configurations
	{
		GPIOC,
		GPIOC,
		GPIO_PIN_8,
		GPIO_PIN_9,
		TIM8,
		TIM_CHANNEL_3,
		TIM_CHANNEL_4,
		84,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	}
};



void DC_MOTOR_Init(uint8_t au8_MOTOR_Instance)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    TIM_HandleTypeDef htim;

    uint32_t PSC_Value = 0;
    uint32_t ARR_Value = 0;
    uint8_t i = 0;

	/*--------[ Configure The 2 Direction Control GPIO Pins ]-------*/

    if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO == GPIOA || DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO == GPIOA)
    {
    	__HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO == GPIOB || DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO == GPIOB)
    {
    	__HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO == GPIOC || DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO == GPIOD || DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO == GPIOE || DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    /*
	GPIO_InitStruct.Pin = DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO, &GPIO_InitStruct);
	*/
	GPIO_InitStruct.Pin = DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, &GPIO_InitStruct);
	//HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_PIN, 0);
	HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 0);


	/*--------[ Calculate The PSC & ARR Values To Set PWM Resolution And Approx. The F_pwm ]-------*/

	/* Those Equations Sets The PWM Resolution & Approximates The F_pwm */

	ARR_Value = 1;
	for(i=0; i<DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_RES_BITS; i++)
	{
		ARR_Value *= 2;
	}
	PSC_Value = (uint32_t) ((DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_CLK_MHz*1000000) / (ARR_Value*DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_FREQ_Hz));
	PSC_Value--;
	ARR_Value -= 2;

	/*--------[ Configure The DC Motor PWM Timer Channel ]-------*/
	PSC_Value= 84-1;
	ARR_Value = 100-1;

	   htim.Instance = DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance;

	/*if (au8_MOTOR_Instance == 0){
		htim.Instance = TIM1;

	}else if(au8_MOTOR_Instance == 1){
		htim.Instance = TIM8;

	}*/

		htim.Init.Prescaler = PSC_Value;
		htim.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim.Init.Period = ARR_Value;
		htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim.Init.RepetitionCounter = 0;
		htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(&htim) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	  if (HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.DeadTime = 0;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  if (HAL_TIMEx_ConfigBreakDeadTime(&htim, &sBreakDeadTimeConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_TIM_CH4);
	//HAL_TIM_MspPostInit(&htim);

	/*--------[ Start The PWM Channel ]-------*/
	//DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance->CCR3 = 50;
	HAL_TIM_PWM_Start(&htim, DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_TIM_CH3);
	//HAL_TIM_PWM_Start(&htim,TIM_CHANNEL_3);

	//HAL_TIM_PWM_Start(&htim, DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_TIM_CH4);
}

void DC_MOTOR_Run(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint16_t au16_SPEED)
{
	/* Write To The 2 Direction Control Pins */
	if(au8_DIR == DIR_CW)
	{
//		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_PIN, 1);
//		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 0);
		(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance)->CCR3 = au16_SPEED;

		//if (au8_MOTOR_Instance == 0){
		//	TIM1->CCR3 = au16_SPEED;
		//}else if(au8_MOTOR_Instance == 1){
		//	TIM8->CCR3 = au16_SPEED;
		//}

		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 0);
		//DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance->CCR4 = au16_SPEED;
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);

	}
	else if(au8_DIR == DIR_CCW)
	{
//		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_PIN, 0);
//		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 1);
		(DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance)->CCR3 = au16_SPEED;
		//if (au8_MOTOR_Instance == 0){
		//	TIM1->CCR3 = au16_SPEED;
		//}else if(au8_MOTOR_Instance == 1){
		//	TIM8->CCR3 = au16_SPEED;
		//}
		HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 1);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);

		//DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance->CCR4 = 0;
	}
}

void DC_MOTOR_Stop(uint8_t au8_MOTOR_Instance)
{
	/* Write To The 2 Direction Control Pins */
	//HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR1_PIN, 0);
	//HAL_GPIO_WritePin(DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_GPIO, DC_MOTOR_CfgParam[au8_MOTOR_Instance].DIR2_PIN, 0);

	/* Write ZERO To The PWM Ch DutyCycle Register */

	DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance->CCR3 = 0;
	//DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_Instance->CCR4 = 0;

}

uint32_t DC_MOTOR_Get_MaxFreq(uint8_t au8_MOTOR_Instance)
{
	uint32_t ARR_Value = 1;
    uint8_t i = 0;

	for(i=0; i<DC_MOTOR_CfgParam[au8_MOTOR_Instance].PWM_RES_BITS; i++)
	{
		ARR_Value *= 2;
	}
	return ((DC_MOTOR_CfgParam[au8_MOTOR_Instance].TIM_CLK_MHz*1000000)/ARR_Value);
}
