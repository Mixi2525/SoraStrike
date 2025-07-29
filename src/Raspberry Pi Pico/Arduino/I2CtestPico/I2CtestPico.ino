#include "SensorHand.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include <MadgwickAHRS.h>

SensorHand right, left;

// 校正結果: 8g, 2000deg/s [-2797,-2796], [-449,-448], [947,947], [49,50], [19,20], [-14,-13] 
int16_t gyroOffsetR[3] = { -2796, -449, 947 };
int16_t accelOffsetR[3] = { 49, 19, -14 };

// 校正結果: 8g, 2000deg/s [-5113,-5113] [-69,-68] [937,938] [138,139] [7,8] [-64,-63] 
int16_t gyroOffsetL[3] = { -5113, -69, 937 };
int16_t accelOffsetL[3] = { 138, 7, -64 };


void setup()
{
  Serial.begin(115200);
  Serial.println("Init MPU");
  bool rightReady = right.imuInit(MPU6050_ADDRESS_AD0_HIGH, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_8, gyroOffsetR, accelOffsetR, 6);
  bool leftReady = left.imuInit(MPU6050_ADDRESS_AD0_LOW, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_8, gyroOffsetL, accelOffsetL, 6);

  if (!rightReady || !leftReady)
  {
    while (1)
    {
      Serial.println("MPU6050 connection failed");
      delay(1000);
    }
  }
  Serial.println("MPU6050 connection success");
}

void loop()
{
  right.updateImuData();
  left.updateImuData();
  Serial.println("right");
  right.printImuScaledData();
  Serial.println("left");
  left.printImuScaledData();
  delay(100);

}