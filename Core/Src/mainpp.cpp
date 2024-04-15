/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <stdlib.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt16.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <PID_v1.h>
#include "main.h"
#include "ssd1306.h"
#include "fonts.h"
#include "dc_motor.h"
#include <stdio.h>
#include "SERVO.h"


#define SERVO_Motor1  0
#define SERVO_Motor2  1
#define SERVO_Motor3  2
#define SERVO_Motor4  3

#define GPIO_SET_PIN(port, pin)     ((port)->BSRR = (pin))
#define GPIO_CLEAR_PIN(port, pin)     ((port)->BSRR = (pin << 16u))

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

#define DELAY_MS(ms) \
    do { \
        for (uint32_t i = 0; i < ms; ++i) { \
            DELAY_US(1000); \
        } \
    } while (0)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
float periods=0.0;

float grip_close_ms = 1.0, grip_open_ms = 2.0;
float grip_up_ms = 2.5, grip_down_ms = 0.5;
float L_rot_ms = 1.5, R_rot_ms = 2.5;
float slide_out_ms = 2.0, slide_in_ms = 2.5;
float slide_out_min_ms = 2.4, slide_out_medium_ms = 2.2,slide_out_max_ms=2.0;

bool checkBoxState=false;

uint16_t gripper_command,action_State=0 ,gripper_last_command=0;

uint8_t degree = 0,pos=0;


//ROS PUBLISH VARIABLE SETUP
ros::NodeHandle nh;

std_msgs::UInt16 grip_state_val;
ros::Publisher grip_state("grip_state", &grip_state_val);


//std_msgs::UInt32 rc_ch2_val;
//ros::Publisher rc_ch2("rc_ch2", &rc_ch2_val);



/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}
*/


//ROS SUBSCRIBE VARIABLE SETUP AND CALL BACK FUNCTION

/*
void cmd_vel_cb( const geometry_msgs::Twist& twist){
  float demandx = twist.linear.x;
  float demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );

*/


void grip_cmd_cb( const std_msgs::UInt16 &msg){
	gripper_command =uint16_t(msg.data);
}

ros::Subscriber<std_msgs::UInt16> sub("grip_cmd", grip_cmd_cb );




uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax, uint32_t au32_OUTmin, uint32_t au32_OUTmax)
{
    return ((((au32_IN - au32_INmin)*(au32_OUTmax - au32_OUTmin))/(au32_INmax - au32_INmin)) + au32_OUTmin);
}

uint32_t Constrain(uint32_t au32_IN, uint32_t au32_MIN, uint32_t au32_MAX)
{
    if(au32_IN < au32_MIN)
    {
    return au32_MIN;
    }
    else if (au32_IN > au32_MAX)
    {
    return au32_MAX;
    }
    else
    {
    return au32_IN;
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
	  __NOP();
  } else {
      __NOP();
  }
}
// ACTION //


void gripper_close(float msperiod){
    uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor1,rotatewidth);


}

void gripper_open(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor1,rotatewidth);

}

void gripper_up(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor2,rotatewidth);


}

void gripper_down(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor2,rotatewidth);


}

void gripper_slideout(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor3,rotatewidth);


}

void gripper_slidein(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor3,rotatewidth);


}

void gripper_rotate_ms(float msperiod){
	uint16_t rotatewidth;
	rotatewidth=msperiod*500;
	SERVO_RawMove(SERVO_Motor4,rotatewidth);
}




void setupcpp(void)
{


  nh.initNode();
  nh.subscribe(sub);

  nh.advertise(grip_state);
  //nh.advertise(rc_ch2);

  SSD1306_Init();  // initialise



  SERVO_Init(SERVO_Motor1);
  SERVO_Init(SERVO_Motor2);
  SERVO_Init(SERVO_Motor3);
  SERVO_Init(SERVO_Motor4);

 // SERVO_MoveTo(SERVO_Motor1, 0);
  //SERVO_MoveTo(SERVO_Motor2, 0);
  //SERVO_MoveTo(SERVO_Motor3, 0);
  //SERVO_MoveTo(SERVO_Motor4, 0);

 // TIM5->CCR1 = 5000;
  //TIM5->CCR2 = 5000;


}



void loop(void)
{
  uint32_t time_ms,time_lcdrefresh=0;

  uint16_t rotatewidth ;
  uint16_t periodpp;

	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	  nh.spinOnce();

	  time_ms = HAL_GetTick() ;

	  //CHECK COMMAND FROM ROS
	  if(gripper_last_command !=gripper_command){
		  action_State=0;
		  if((gripper_command==1 )&&(action_State!=1)){//Grip

			  //Off LED
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			  //Rotate Aligment
			  /*
			  if((rangelunaL<60) || (rangelunaR<60)){
				  if(rangelunaL-rangelunaR > 20){
					  //ROTATE TO R (L to far )
					  gripper_rotate_ms(0.7);
				  }else if(rangelunaR-rangelunaL > 20){
					  //ROTATE TO L (R to far )
					  gripper_rotate_ms(2.3);
				  }else {
					  //Center
					  gripper_rotate_ms(1.5);
				  }
			  }
*/
			  HAL_Delay(1000);
			  //Aligment
			  if((rangelunaL<60) || (rangelunaR<60)){
				  if(rangelunaL-rangelunaR > 20){
					  //ROTATE TO R (L to far )

	  				  //CHECK IF SENSOR FOUND BOX IN CENTER
	  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  			      {

	  			          HAL_Delay(10);
	  			          //RE-CHECK IF SENSOR FOUND BOX
	  				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  				      {
	  				    	  gripper_rotate_ms(1.1);

	  				      }else{

	  				    	 gripper_rotate_ms(0.7);
	  				      }
	  			      }else{

	  			    	  gripper_rotate_ms(0.7);
	  			      }

				  }else if(rangelunaR-rangelunaL > 20){
					  //ROTATE TO L (R to far )
					  //gripper_rotate_ms(2.3);
	  				  //CHECK IF SENSOR FOUND BOX IN CENTER
	  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  			      {

	  			          HAL_Delay(10);
	  			          //RE-CHECK IF SENSOR FOUND BOX
	  				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  				      {
	  				    	  gripper_rotate_ms(1.9);

	  				      }else{

	  				    	 gripper_rotate_ms(2.3);
	  				      }
	  			      }else{

	  			    	gripper_rotate_ms(2.3);

	  			      }
				  }else {
					  //Center
					  gripper_rotate_ms(1.5);
				  }
			  }

			  HAL_Delay(1000);

              checkBoxState=false;

              //Slide to Min State 1
			  gripper_slideout(slide_out_min_ms);
			  HAL_Delay(1000);
			  //CHECK IF SENSOR FOUND BOX
		      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
		      {
		          HAL_Delay(10);
		          //RE-CHECK IF SENSOR FOUND BOX
			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			      {
			    	  //IF FOUND BOX SET checkBoxState flag
			    	  checkBoxState=true;
			          // Set The LED ON!
			          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			      }
		      }
		      //Slide Medium State 2
		      if(checkBoxState==false){
	              //Slide Min
				  gripper_slideout(slide_out_medium_ms);
				  HAL_Delay(1000);
				  //CHECK IF SENSOR FOUND BOX
			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			      {

			          HAL_Delay(10);
			          //RE-CHECK IF SENSOR FOUND BOX
				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
				      {
				    	  //IF FOUND BOX SET checkBoxState flag
				    	  checkBoxState=true;
				          // Set The LED ON!
				          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				      }
			      }

		      }
		      //Slide to MAX State 3
		      if(checkBoxState==false){
	              //Slide MAX
				  gripper_slideout(slide_out_max_ms);
				  HAL_Delay(1000);
				  //CHECK IF SENSOR FOUND BOX
				  HAL_Delay(10);
			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			      {
			    	  //IF FOUND BOX SET checkBoxState flag
			    	  checkBoxState=true;
			          // Set The LED ON!
			          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

			      }
		      }

//IF NOT FOUND BOX XXX HOLD IT

		      //if(checkBoxState){
				  //gripper_slideout(slide_out_ms);
				  //HAL_Delay(2000);
				  gripper_close(grip_close_ms);
				  HAL_Delay(2000);
		     // }

			  gripper_up(grip_up_ms);
			  HAL_Delay(2000);

			  gripper_slidein(slide_in_ms);
			  HAL_Delay(2000);

			  gripper_rotate_ms(1.5);
			  HAL_Delay(2000);
			  action_State=1;

		  }else if ((gripper_command==2 )&&(action_State!=1)){//Grip

			  HAL_Delay(1000);
			  //Aligment
			  if((rangelunaL<60) || (rangelunaR<60)){
				  if(rangelunaL-rangelunaR > 20){
					  //ROTATE TO R (L to far )

	  				  //CHECK IF SENSOR FOUND BOX IN CENTER
	  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  			      {

	  			          HAL_Delay(10);
	  			          //RE-CHECK IF SENSOR FOUND BOX
	  				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  				      {
	  				    	  gripper_rotate_ms(1.1);

	  				      }else{

	  				    	 gripper_rotate_ms(0.7);
	  				      }
	  			      }else{

	  			    	  gripper_rotate_ms(0.7);
	  			      }

				  }else if(rangelunaR-rangelunaL > 20){
					  //ROTATE TO L (R to far )
					  //gripper_rotate_ms(2.3);
	  				  //CHECK IF SENSOR FOUND BOX IN CENTER
	  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  			      {

	  			          HAL_Delay(10);
	  			          //RE-CHECK IF SENSOR FOUND BOX
	  				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1))
	  				      {
	  				    	  gripper_rotate_ms(1.9);

	  				      }else{

	  				    	 gripper_rotate_ms(2.3);
	  				      }
	  			      }else{

	  			    	gripper_rotate_ms(2.3);

	  			      }
				  }else {
					  //Center
					  gripper_rotate_ms(1.5);
				  }
			  }

			  HAL_Delay(1000);

              //Slide Gripper Out with check
			  checkBoxState=false;

			                //Slide to Min State 1
			  			  gripper_slideout(slide_out_min_ms);
			  			  HAL_Delay(1000);
			  			  //CHECK IF SENSOR FOUND BOX
			  		      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			  		      {
			  		          HAL_Delay(10);
			  		          //RE-CHECK IF SENSOR FOUND BOX
			  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			  			      {
			  			    	  //IF FOUND BOX SET checkBoxState flag
			  			    	  checkBoxState=true;
			  			          // Set The LED ON!
			  			          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			  			      }
			  		      }
			  		      //Slide Medium State 2
			  		      if(checkBoxState==false){
			  	              //Slide Min
			  				  gripper_slideout(slide_out_medium_ms);
			  				  HAL_Delay(1000);
			  				  //CHECK IF SENSOR FOUND BOX
			  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			  			      {

			  			          HAL_Delay(10);
			  			          //RE-CHECK IF SENSOR FOUND BOX
			  				      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			  				      {
			  				    	  //IF FOUND BOX SET checkBoxState flag
			  				    	  checkBoxState=true;
			  				          // Set The LED ON!
			  				          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			  				      }
			  			      }

			  		      }
			  		      //Slide to MAX State 3
			  		      if(checkBoxState==false){
			  	              //Slide MAX
			  				  gripper_slideout(slide_out_max_ms);
			  				  HAL_Delay(2000);
			  				  //CHECK IF SENSOR FOUND BOX
			  				  HAL_Delay(10);
			  			      if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_0))
			  			      {
			  			    	  //IF FOUND BOX SET checkBoxState flag
			  			    	  checkBoxState=true;
			  			          // Set The LED ON!
			  			          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

			  			      }
			  		      }

			  //IF NOT FOUND BOX XXX HOLD IT

			  //gripper_slideout(slide_out_ms);
			  //HAL_Delay(2000);


		 //if(checkBoxState){
			 //if((!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_2)) || checkBoxState){
			  HAL_Delay(5000);
			  gripper_down(grip_down_ms);
			  HAL_Delay(2000);
			  gripper_open(grip_open_ms);
			  HAL_Delay(2000);
		    // }

			  gripper_slidein(slide_in_ms);
			  HAL_Delay(2000);
			  //ROTATE TO CENTER
			  gripper_rotate_ms(1.5);
			  HAL_Delay(2000);
			  action_State=1;


			  action_State=1;
		  }
		  gripper_last_command=gripper_command;
	  }


  if(time_ms -time_lcdrefresh > 500){
	  time_lcdrefresh=time_ms;
/*
	  if(pos ==0){

		  if( periods < 3){
			  periods=periods+0.01;
			  rotatewidth=periods*500;
				  SERVO_RawMove(SERVO_Motor1,rotatewidth);
				  SERVO_RawMove(SERVO_Motor2,rotatewidth);
				  SERVO_RawMove(SERVO_Motor3,rotatewidth);
				  SERVO_RawMove(SERVO_Motor4,rotatewidth);

		  }else{
			  pos=1;

		  }
			  //SERVO_MoveTo(SERVO_Motor1, 180);
			  //SERVO_MoveTo(SERVO_Motor2, 0);
			  //SERVO_MoveTo(SERVO_Motor3, 180);
			  //SERVO_MoveTo(SERVO_Motor4, 0);
	  }else if(pos ==1){

		  if( periods > 0  ){
			  periods=periods-0.01;
			  rotatewidth=periods*500;
				  SERVO_RawMove(SERVO_Motor1,rotatewidth);
				  SERVO_RawMove(SERVO_Motor2,rotatewidth);
				  SERVO_RawMove(SERVO_Motor3,rotatewidth);
				  SERVO_RawMove(SERVO_Motor4,rotatewidth);

		  }else{
			  pos=0;

		  }

	  }


*/



		char sbuff[20];

	  /// lets print some string
		SSD1306_Fill(SSD1306_COLOR_BLACK);

		//SSD1306_GotoXY (1,1);
		//SSD1306_Puts (sbuff, &Font_7x10, SSD1306_COLOR_WHITE);
		//sprintf(sbuff,"ad1= %d",rawadc1);
		SSD1306_GotoXY (1,1);
		sprintf(sbuff,"SERVO EXPAND");
		SSD1306_Puts (sbuff, &Font_7x10, SSD1306_COLOR_WHITE);

		SSD1306_GotoXY (1,17);

		sprintf(sbuff,"cmd= %d",gripper_command);
		SSD1306_Puts (sbuff, &Font_7x10, SSD1306_COLOR_WHITE);

		SSD1306_GotoXY (1,33);
		sprintf(sbuff,"RaL= %d",rangelunaL);
		SSD1306_Puts (sbuff, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY (1,41);
		sprintf(sbuff,"RaR= %d",rangelunaR);
		SSD1306_Puts (sbuff, &Font_7x10, SSD1306_COLOR_WHITE);

		SSD1306_UpdateScreen(); //display

  }

}

