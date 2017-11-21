// NOTE: Need to install i2c tools:
// sudo apt-get install libi2c-dev i2c-tools
#include "GrudsbyImu.h"
#include "ros/ros.h"

GrudsbyImuBase::GrudsbyImuBase(const unsigned int address) {
  myI2CBus = 1; // sets the selected I2C bus
  myError = 0;
  myAddr = address;  
  x = 0;
  y = 0;
  z = 0;
  init();
}

GrudsbyImuBase::~GrudsbyImuBase() {
  if (myI2CFileDescriptor > 0) {
    close(myI2CFileDescriptor);
    // WARNING - This is not quite right, need to check for error first
    myI2CFileDescriptor = -1 ;
  }
}

bool GrudsbyImuBase::init() {
  char fileNameBuffer[32];
  sprintf(fileNameBuffer,"/dev/i2c-%d", myI2CBus);
  myI2CFileDescriptor = open(fileNameBuffer, O_RDWR);
  if (myI2CFileDescriptor < 0) {
      // Could not open the file
      myError = errno ;
      return false ;
  }
  if (ioctl(myI2CFileDescriptor, I2C_SLAVE, myAddr) < 0) {
      // Could not open the devices on the bus
      myError = errno ;
      return false ;
  }
  return true ;
}


int GrudsbyImuBase::readRegister(int regToRead) { 
  int toReturn ;
  toReturn = i2c_smbus_write_byte(myI2CFileDescriptor, myAddr) ;
  toReturn = i2c_smbus_write_byte(myI2CFileDescriptor, regToRead) ;
  if (toReturn < 0) {
      myError = errno ;
      toReturn = -1 ;
  }
  toReturn = i2c_smbus_read_byte(myI2CFileDescriptor) ;
  if (toReturn < 0) {
      myError = errno ;
      toReturn = -1 ;
  }
  return toReturn ;
}

int GrudsbyImuBase::writeRegister(int regtoWrite, int dataToWrite) {
  ioctl(myI2CFileDescriptor, I2C_SLAVE, myAddr);
  i2c_smbus_write_byte(myI2CFileDescriptor, myAddr) ;
  int toReturn = i2c_smbus_write_byte_data(myI2CFileDescriptor, regtoWrite,  dataToWrite);
  if (toReturn < 0) {
      myError = errno ;
      toReturn = -1 ;
  }
  return toReturn ;
}

bool GrudsbyImuBase::readInternal(int highReg, int lowReg, int *value) {
  uint16_t data = 0;
  unsigned char  read = 0;
  int16_t signed_data = 0;

  for (;;) {
      read = readRegister(statusReg);
      if (read & readyReg) {
          read = readRegister(lowReg);
          data = read;      // LSB

          read = readRegister(highReg);
          data |= read << 8; // MSB

          signed_data = data;
          *value = signed_data;
          return true;
      } else {
          usleep(400); // data wasn't ready
      }
  }
  return false;
}


GrudsbyImuAccel::GrudsbyImuAccel() : GrudsbyImuBase(LSM9DS1_AG_ADDRESS) {
  readyReg = AG_ACCELEROMETER_READY;
  statusReg = AG_STATUS_REG;
}

GrudsbyImuAccel::~GrudsbyImuAccel() {

}

bool GrudsbyImuAccel::activate() {
  uint8_t data;
  data = readRegister(AG_CTRL_REG1_G);
  //data |= POWER_UP;
  data |= AG_ODR_SET;
  writeRegister(AG_CTRL_REG1_G, data);

  return true;
}

bool GrudsbyImuAccel::deactivate() {
  uint8_t data;
  data = readRegister(AG_CTRL_REG1_G);
  data &= ~AG_ODR_SET;
  writeRegister(AG_CTRL_REG1_G, data);
  return true;
}

bool GrudsbyImuAccel::begin() {
  uint8_t data;
  data = readRegister(AG_WHO_AM_I);
  if (data == AG_WHO_AM_I_RETURN) {
    if (activate()) {
      return true;
    }
  }
  return false;
}

double GrudsbyImuAccel::readX() {
  int data = 0;
  if (readInternal(AG_ACC_X_H, AG_ACC_X_L, &data)) {
      x = data;
  }
  // Decode Accel x-axis  [mdps measurement unit]
  return (9.81 * A_LSB_SENSIT_2MG * x /  1000.0 ) ;
}

double GrudsbyImuAccel::readY() {
  int data = 0;
  if (readInternal(AG_ACC_Y_H, AG_ACC_Y_L, &data)) {
      y = data;
  }
  // Decode Accel x-axis  [mdps measurement unit]
  return (9.81 * A_LSB_SENSIT_2MG * y / -1000.0 );
}

double GrudsbyImuAccel::readZ() {
  int data = 0;
  if (readInternal(AG_ACC_Z_H, AG_ACC_Z_L, &data)) {
      z = data;
  }
  // Decode Accel x-axis  [mdps measurement unit]
  return (9.81 * A_LSB_SENSIT_2MG * z / 1000.0);
}


GrudsbyImuMag::GrudsbyImuMag() : GrudsbyImuBase(LSM9DS1_M_ADDRESS) {
  readyReg  = M_ZYX_AXIS_READY;
  statusReg = M_STATUS_REG;
}

GrudsbyImuMag::~GrudsbyImuMag() {

}

bool GrudsbyImuMag::activate() {
  uint8_t data;
  data = readRegister(M_CTRL_REG3_G);
  //data |= POWER_UP;
  //data |= 0x21;
  data &= ~M_OPER_MODE_DIS;
  //data &= (0xFC);
  writeRegister(M_CTRL_REG3_G, data);
  return true;
}

bool GrudsbyImuMag::deactivate() {
  uint8_t data;
  data = readRegister(M_CTRL_REG3_G);
  //data |= POWER_UP;
  //data |= 0x21;
  data |= M_OPER_MODE_DIS;
  writeRegister(M_CTRL_REG3_G, data);

  return true;
}

bool GrudsbyImuMag::begin() {
  uint8_t data;
  data = readRegister(M_WHO_AM_I);
  if (data == M_WHO_AM_I_RETURN){
      if (activate()){
          return true;
      }
  }
  return false;
}

double GrudsbyImuMag::readX() {
  int data = 0;
  if (readInternal(M_X_H, M_X_L, &data)) {
      x = data;
  }
  // Decode magnetic x-axis  [mgauss measurement unit]
  return (M_LSB_SENSIT_4MG * x / 1000.0);
}

double GrudsbyImuMag::readY() {
  int data = 0;
  if (readInternal(M_Y_H, M_Y_L, &data)) {
      y = data;
  }
  // Decode magnetic x-axis  [mgauss measurement unit]
  return (M_LSB_SENSIT_4MG * y / -1000.0);
}

double GrudsbyImuMag::readZ() {
  int data = 0;
  if (readInternal(M_Z_H, M_Z_L, &data)) {
      z = data;
  }
  // Decode magnetic x-axis  [mgauss measurement unit]
  return (M_LSB_SENSIT_4MG * z / 1000.0);
}


GrudsbyImuGyro::GrudsbyImuGyro() : GrudsbyImuBase(LSM9DS1_AG_ADDRESS){
  readyReg  = AG_GYROSCOPE_READY;
  statusReg = AG_STATUS_REG;
}

GrudsbyImuGyro::~GrudsbyImuGyro() {

}

bool GrudsbyImuGyro::activate() {
  uint8_t data;
  data = readRegister(AG_CTRL_REG1_G);
  //data |= POWER_UP;
  data |= AG_ODR_SET;
  writeRegister(AG_CTRL_REG1_G, data);
  return true;
}

bool GrudsbyImuGyro::deactivate() {
  uint8_t data;
  data = readRegister(AG_CTRL_REG1_G);
  data &= ~AG_ODR_SET;
  writeRegister(AG_CTRL_REG1_G, data);
  return true;
}

bool GrudsbyImuGyro::begin() {
  uint8_t data;
  data = readRegister(AG_WHO_AM_I);
  if (data == AG_WHO_AM_I_RETURN){
      if (activate()){
          return true;
      }
  }
  return false;
}

double GrudsbyImuGyro::readX() {
  int data = 0;
  if (readInternal(AG_GYR_X_H, AG_GYR_X_L, &data)) {
      x = data;
  }
  // Decode Gyroscope x-axis  [mdps measurement unit]
  return (G_LSB_SENSIT_245MDPS * x * 3.141592653 / (1000.0 * 180));
}

double GrudsbyImuGyro::readY() {
  int data = 0;
  if (readInternal(AG_GYR_Y_H, AG_GYR_Y_L, &data)) {
      y = data;
  }
  // Decode Gyroscope x-axis  [mdps measurement unit]
  return (G_LSB_SENSIT_245MDPS * y * 3.141592653 / (-1000.0 * 180));
}

double GrudsbyImuGyro::readZ() {
  int data = 0;
  if (readInternal(AG_GYR_Z_H, AG_GYR_Z_L, &data)) {
      z = data;
  }
  // Decode Gyroscope x-axis  [mdps measurement unit]
  return (G_LSB_SENSIT_245MDPS * z * 3.141592653 / (1000.0 * 180));
}
