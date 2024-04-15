#ifndef _ROS_ired_msgs_PID_h
#define _ROS_ired_msgs_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ired_msgs
{

  class PID : public ros::Msg
  {
    public:
      typedef const char* _motor_type;
      _motor_type motor;
      typedef float _kp_type;
      _kp_type kp;
      typedef float _ki_type;
      _ki_type ki;
      typedef float _kd_type;
      _kd_type kd;

    PID():
      motor(""),
      kp(0),
      ki(0),
      kd(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_motor = strlen(this->motor);
      varToArr(outbuffer + offset, length_motor);
      offset += 4;
      memcpy(outbuffer + offset, this->motor, length_motor);
      offset += length_motor;
      offset += serializeAvrFloat64(outbuffer + offset, this->kp);
      offset += serializeAvrFloat64(outbuffer + offset, this->ki);
      offset += serializeAvrFloat64(outbuffer + offset, this->kd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_motor;
      arrToVar(length_motor, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motor; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motor-1]=0;
      this->motor = (char *)(inbuffer + offset-1);
      offset += length_motor;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kp));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ki));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->kd));
     return offset;
    }

    virtual const char * getType() override { return "ired_msgs/PID"; };
    virtual const char * getMD5() override { return "2169d9d8246848be6270c046ce9df384"; };

  };

}
#endif
