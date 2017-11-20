#ifndef GrudsbyImu_H
#define GrudsbyImu_H

#include "GrudsbyImuReg.h"

#include <cstddef>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint-gcc.h>

class GrudsbyImuBase {
  public:
    GrudsbyImuBase(const unsigned int slaveAddr);
    ~GrudsbyImuBase();
    bool init();
    
  protected: 
    int x, y, z;
    int readyReg;
    int statusReg;

    int error() {
      return myError;
    }

    int addr() {
      return myAddr;
    }

    int readRegister(int regToRead);

    int writeRegister(int regtoWrite, int dataToWrite);
      
    bool readInternal(int highReg, int lowReg, int *value);
  

  private: 
    unsigned char myI2CBus ;
    int myI2CFileDescriptor ;
    int myAddr;
    int myError;
};

class GrudsbyImuAccel : GrudsbyImuBase {
  public:
    GrudsbyImuAccel();
    ~GrudsbyImuAccel();
    bool activate();
    bool deactivate();
    bool begin();
    double readX();
    double readY();
    double readZ();
};

class GrudsbyImuMag : GrudsbyImuBase{
  public:
    GrudsbyImuMag();
    ~GrudsbyImuMag();
    bool activate();
    bool deactivate();
    bool begin();
    double readX();
    double readY();
    double readZ();
};

class GrudsbyImuGyro : GrudsbyImuBase{
  public:
    GrudsbyImuGyro();
    ~GrudsbyImuGyro();
    bool activate();
    bool deactivate();
    bool begin();
    double readX();
    double readY();
    double readZ();
};
#endif //GrudsbyImu_H
