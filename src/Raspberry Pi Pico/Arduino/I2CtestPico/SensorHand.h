#ifndef _SENSOR_HAND_H_
#define _SENSOR_HAND_H_

#include <MPU6050_6Axis_MotionApps612.h>
#include <MadgwickAHRS.h>
#include <I2Cdev.h>

#include <Keyboard.h>

class SensorHand
{
private:
  MPU6050 mpu;
  Madgwick madgwickFilter;

  bool isImuInitialized = false;

  /* ===== Sensor & System settings =====*/
  float accelScale, gyroScale;
  int fullScaleGyroRange;
  int fullScaleAccelRange;
  uint16_t sensorFullScaleGyroValue;
  uint8_t sensorFullScaleAccelValue;
  uint16_t sensorFullScaleValue = 32768.0;
  uint8_t calibrateCount = 6;

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  float scaledAx, scaledAy, scaledAz, scaledGx, scaledGy, scaledGz;
  
  int convertGyroRangeValue(uint8_t fullScaleGyroRange);
  int convertAccelRangeValue(uint8_t fullScaleAccelRange);

public:
  SensorHand();
  
  // SensorSettings & Processes
  void setOffset(int16_t gyroOffset[3], int16_t accelOffset[3]);
  bool imuInit(uint8_t address, uint8_t fullScaleGyroRange, uint8_t fullScaleAccelRange, int16_t gyroOffset[3], int16_t accelOffset[3], uint8_t calibrateCount = 6);
  void updateImuData();
  void printImuScaledData();

};




#endif /* _SENSOR_HAND_H_ */