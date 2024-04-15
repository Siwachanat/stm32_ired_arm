/*
 * File: SERVO.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "SERVO.h"
#include "SERVO_cfg.h"
//#include "../../util/DWT_Delay.h"

typedef struct
{
	uint16_t  Period_Min;
	uint16_t  Period_Max;
}SERVO_info;

static SERVO_info gs_SERVO_info[SERVO_NUM] = {0};



void SERVO_Init(uint16_t au16_SERVO_Instance)
{


	/*--------[ Calculate & Save The Servo Pulse Information ]-------*/

	gs_SERVO_info[au16_SERVO_Instance].Period_Min = (uint16_t) (10000 * (SERVO_CfgParam[au16_SERVO_Instance].MinPulse/20.0));
	gs_SERVO_info[au16_SERVO_Instance].Period_Max = (uint16_t) (10000 * (SERVO_CfgParam[au16_SERVO_Instance].MaxPulse/20.0));


}

/* Moves A Specific Motor To A Specific Degree That Can Be Float Number */
void SERVO_MoveTo(uint16_t au16_SERVO_Instance, float af_Angle)
{
	uint16_t au16_Pulse = 0;

	au16_Pulse = ((af_Angle*(gs_SERVO_info[au16_SERVO_Instance].Period_Max - gs_SERVO_info[au16_SERVO_Instance].Period_Min))/180.0)
			+ gs_SERVO_info[au16_SERVO_Instance].Period_Min;

	*(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
}

/* Moves A Specific Motor With A Raw Pulse Width Value */
void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t au16_Pulse)
{
	//if(au16_Pulse <= gs_SERVO_info[au16_SERVO_Instance].Period_Max && au16_Pulse >= gs_SERVO_info[au16_SERVO_Instance].Period_Min)
	//{
		*(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
	//}
}

/* Gets The Maximum Pulse Width Value For A Specific Motor */
uint16_t SERVO_Get_MaxPulse(uint16_t au16_SERVO_Instance)
{
	return (gs_SERVO_info[au16_SERVO_Instance].Period_Max);
}


/* Gets The Minimum Pulse Width Value For A Specific Motor */
uint16_t SERVO_Get_MinPulse(uint16_t au16_SERVO_Instance)
{
	return (gs_SERVO_info[au16_SERVO_Instance].Period_Min);
}


/* Move A Motor From 0 deg to 180 And Back to 0 again */
void SERVO_Sweep(uint16_t au16_SERVO_Instance)
{
	uint8_t au8_Angle = 0;
/*
	SERVO_MoveTo(au16_SERVO_Instance, 0);

	DWT_Delay_ms(250);
	while(au8_Angle < 180)
	{
		SERVO_MoveTo(au16_SERVO_Instance, au8_Angle++);
		DWT_Delay_ms(5);
	}
	DWT_Delay_ms(250);
	while(au8_Angle > 0)
	{
		SERVO_MoveTo(au16_SERVO_Instance, au8_Angle--);
		DWT_Delay_ms(5);
	}
	*/
}


#ifdef __cplusplus
}
#endif
