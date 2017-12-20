#ifndef _ROS_grudsby_lowlevel_ArduinoResponse_h
#define _ROS_grudsby_lowlevel_ArduinoResponse_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace grudsby_lowlevel
{

  class ArduinoResponse : public ros::Msg
  {
    public:
      typedef float _linearX_type;
      _linearX_type linearX;
      typedef float _angularZ_type;
      _angularZ_type angularZ;
      typedef uint32_t _encoderLeft_type;
      _encoderLeft_type encoderLeft;
      typedef uint32_t _encoderRight_type;
      _encoderRight_type encoderRight;
      typedef float _velLeft_type;
      _velLeft_type velLeft;
      typedef float _velRight_type;
      _velRight_type velRight;

    ArduinoResponse():
      linearX(0),
      angularZ(0),
      encoderLeft(0),
      encoderRight(0),
      velLeft(0),
      velRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linearX;
      u_linearX.real = this->linearX;
      *(outbuffer + offset + 0) = (u_linearX.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linearX.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linearX.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linearX.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linearX);
      union {
        float real;
        uint32_t base;
      } u_angularZ;
      u_angularZ.real = this->angularZ;
      *(outbuffer + offset + 0) = (u_angularZ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angularZ.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angularZ.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angularZ.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angularZ);
      *(outbuffer + offset + 0) = (this->encoderLeft >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->encoderLeft >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->encoderLeft >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->encoderLeft >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoderLeft);
      *(outbuffer + offset + 0) = (this->encoderRight >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->encoderRight >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->encoderRight >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->encoderRight >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoderRight);
      union {
        float real;
        uint32_t base;
      } u_velLeft;
      u_velLeft.real = this->velLeft;
      *(outbuffer + offset + 0) = (u_velLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velLeft);
      union {
        float real;
        uint32_t base;
      } u_velRight;
      u_velRight.real = this->velRight;
      *(outbuffer + offset + 0) = (u_velRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linearX;
      u_linearX.base = 0;
      u_linearX.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linearX.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linearX.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linearX.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linearX = u_linearX.real;
      offset += sizeof(this->linearX);
      union {
        float real;
        uint32_t base;
      } u_angularZ;
      u_angularZ.base = 0;
      u_angularZ.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angularZ.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angularZ.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angularZ.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angularZ = u_angularZ.real;
      offset += sizeof(this->angularZ);
      this->encoderLeft =  ((uint32_t) (*(inbuffer + offset)));
      this->encoderLeft |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderLeft |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->encoderLeft |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->encoderLeft);
      this->encoderRight =  ((uint32_t) (*(inbuffer + offset)));
      this->encoderRight |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderRight |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->encoderRight |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->encoderRight);
      union {
        float real;
        uint32_t base;
      } u_velLeft;
      u_velLeft.base = 0;
      u_velLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velLeft = u_velLeft.real;
      offset += sizeof(this->velLeft);
      union {
        float real;
        uint32_t base;
      } u_velRight;
      u_velRight.base = 0;
      u_velRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velRight = u_velRight.real;
      offset += sizeof(this->velRight);
     return offset;
    }

    const char * getType(){ return "grudsby_lowlevel/ArduinoResponse"; };
    const char * getMD5(){ return "3d4ac294fa6ba254aec0d9d6416fb8b5"; };

  };

}
#endif