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
      typedef int32_t _leftvel_type;
      _leftvel_type leftvel;
      typedef int32_t _rightvel_type;
      _rightvel_type rightvel;
      typedef int32_t _leftpos_type;
      _leftpos_type leftpos;
      typedef int32_t _rightpos_type;
      _rightpos_type rightpos;
      typedef bool _autonomous_type;
      _autonomous_type autonomous;
      typedef bool _kill_type;
      _kill_type kill;

    ArduinoResponse():
      leftvel(0),
      rightvel(0),
      leftpos(0),
      rightpos(0),
      autonomous(0),
      kill(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      union {
        int32_t real;
        uint32_t base;
      } u_leftpos;
      u_leftpos.real = this->leftpos;
      *(outbuffer + offset + 0) = (u_leftpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftpos);
      union {
        int32_t real;
        uint32_t base;
      } u_rightpos;
      u_rightpos.real = this->rightpos;
      *(outbuffer + offset + 0) = (u_rightpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightpos);
      union {
        bool real;
        uint8_t base;
      } u_autonomous;
      u_autonomous.real = this->autonomous;
      *(outbuffer + offset + 0) = (u_autonomous.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->autonomous);
      union {
        bool real;
        uint8_t base;
      } u_kill;
      u_kill.real = this->kill;
      *(outbuffer + offset + 0) = (u_kill.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->kill);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      union {
        int32_t real;
        uint32_t base;
      } u_leftpos;
      u_leftpos.base = 0;
      u_leftpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftpos = u_leftpos.real;
      offset += sizeof(this->leftpos);
      union {
        int32_t real;
        uint32_t base;
      } u_rightpos;
      u_rightpos.base = 0;
      u_rightpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightpos = u_rightpos.real;
      offset += sizeof(this->rightpos);
      union {
        bool real;
        uint8_t base;
      } u_autonomous;
      u_autonomous.base = 0;
      u_autonomous.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->autonomous = u_autonomous.real;
      offset += sizeof(this->autonomous);
      union {
        bool real;
        uint8_t base;
      } u_kill;
      u_kill.base = 0;
      u_kill.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->kill = u_kill.real;
      offset += sizeof(this->kill);
     return offset;
    }

    const char * getType(){ return "grudsby_lowlevel/ArduinoResponse"; };
    const char * getMD5(){ return "3b37d13e3d0c6dfe64b43bd0871fb240"; };

  };

}
#endif