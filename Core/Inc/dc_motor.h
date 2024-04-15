/*
 * dc_motor.h
 *
 *  Created on: Feb 27, 2024
 *      Author: watch
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_TIM_MODULE_ENABLED

#include "stm32f4xx_hal.h"

// DC Motor Rotation Directions
#define DIR_CW    0
#define DIR_CCW   1

// DC Motor PWM Properties
#define DC_MOTOR_PWM_RES  10
#define DC_MOTOR_F_PWM    500

// The Number OF DC MOTORs To Be Used In The Project
#define DC_MOTOR_UNITS  1

typedef struct
{
    GPIO_TypeDef * DIR1_GPIO;
    GPIO_TypeDef * DIR2_GPIO;
    uint16_t       DIR1_PIN;
    uint16_t       DIR2_PIN;
    TIM_TypeDef *  TIM_Instance;
    uint32_t       PWM_TIM_CH3;
    uint32_t       PWM_TIM_CH4;
    uint16_t       TIM_CLK_MHz;
    uint32_t       PWM_FREQ_Hz;
    uint8_t        PWM_RES_BITS;
}DC_MOTOR_CfgType;

/*-----[ Prototypes For All Functions ]-----*/

void DC_MOTOR_Init(uint8_t au8_MOTOR_Instance);
void DC_MOTOR_Run(uint8_t au8_MOTOR_Instance, uint8_t au8_DIR, uint16_t au16_SPEED);
void DC_MOTOR_Stop(uint8_t au8_MOTOR_Instance);
uint32_t DC_MOTOR_Get_MaxFreq(uint8_t au8_MOTOR_Instance);

#ifdef __cplusplus
}
#endif
#endif /* INC_DC_MOTOR_H_ */
