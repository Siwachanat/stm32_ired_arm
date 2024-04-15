#ifndef _ROS_ired_msgs_Mode_h
#define _ROS_ired_msgs_Mode_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ired_msgs
{

  class Mode : public ros::Msg
  {
    public:
      typedef bool _differential_wheel_type;
      _differential_wheel_type differential_wheel;

    Mode():
      differential_wheel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_differential_wheel;
      u_differential_wheel.real = this->differential_wheel;
      *(outbuffer + offset + 0) = (u_differential_wheel.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->differential_wheel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_differential_wheel;
      u_differential_wheel.base = 0;
      u_differential_wheel.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->differential_wheel = u_differential_wheel.real;
      offset += sizeof(this->differential_wheel);
     return offset;
    }

    virtual const char * getType() override { return "ired_msgs/Mode"; };
    virtual const char * getMD5() override { return "e0f5377ca955b8c348e5de93be73c07e"; };

  };

}
#endif
