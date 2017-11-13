#ifndef _ROS_grudsby_lowlevel_ArduinoVel_h
#define _ROS_grudsby_lowlevel_ArduinoVel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace grudsby_lowlevel
{

  class ArduinoVel : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _leftvel_type;
      _leftvel_type leftvel;
      typedef int32_t _rightvel_type;
      _rightvel_type rightvel;

    ArduinoVel():
      header(),
      leftvel(0),
      rightvel(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_leftvel;
      u_leftvel.real = this->leftvel;
      *(outbuffer + offset + 0) = (u_leftvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftvel);
      union {
        int32_t real;
        uint32_t base;
      } u_rightvel;
      u_rightvel.real = this->rightvel;
      *(outbuffer + offset + 0) = (u_rightvel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightvel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightvel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightvel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightvel);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_leftvel;
      u_leftvel.base = 0;
      u_leftvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftvel = u_leftvel.real;
      offset += sizeof(this->leftvel);
      union {
        int32_t real;
        uint32_t base;
      } u_rightvel;
      u_rightvel.base = 0;
      u_rightvel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightvel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightvel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightvel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightvel = u_rightvel.real;
      offset += sizeof(this->rightvel);
     return offset;
    }

    const char * getType(){ return "grudsby_lowlevel/ArduinoVel"; };
    const char * getMD5(){ return "22ffe82a7dd0e86dcfa1b09fb0c80554"; };

  };

}
#endif