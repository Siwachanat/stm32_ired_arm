#ifndef _ROS_ired_msgs_Motor_h
#define _ROS_ired_msgs_Motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ired_msgs
{

  class Motor : public ros::Msg
  {
    public:
      float speed_sp[4];
      float speed_fb[4];
      float pid_motor_front_left[3];
      float pid_motor_front_right[3];
      float pid_motor_rear_left[3];
      float pid_motor_rear_right[3];

    Motor():
      speed_sp(),
      speed_fb(),
      pid_motor_front_left(),
      pid_motor_front_right(),
      pid_motor_rear_left(),
      pid_motor_rear_right()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->speed_sp[i]);
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->speed_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pid_motor_front_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pid_motor_front_right[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pid_motor_rear_left[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->pid_motor_rear_right[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed_sp[i]));
      }
      for( uint32_t i = 0; i < 4; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed_fb[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pid_motor_front_left[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pid_motor_front_right[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pid_motor_rear_left[i]));
      }
      for( uint32_t i = 0; i < 3; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->pid_motor_rear_right[i]));
      }
     return offset;
    }

    virtual const char * getType() override { return "ired_msgs/Motor"; };
    virtual const char * getMD5() override { return "ec2526e9a146e07c0941072e0ef16c7b"; };

  };

}
#endif
