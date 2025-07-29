#include "SensorHand.h"

SensorHand::SensorHand()
{

}

int SensorHand::convertGyroRangeValue(uint8_t fullScaleGyroRange)
{
  switch (fullScaleGyroRange)
  {
    case MPU6050_GYRO_FS_250:
      return 250;
    case MPU6050_GYRO_FS_500:
      return 500;
    case MPU6050_GYRO_FS_1000:
      return 1000;
    case MPU6050_GYRO_FS_2000:
      return 2000;
    default:
      return 250;
  }
}

int SensorHand::convertAccelRangeValue(uint8_t fullScaleAccelRange)
{
  switch (fullScaleAccelRange)
  {
    case MPU6050_ACCEL_FS_2:
      return 2;
    case MPU6050_ACCEL_FS_4:
      return 4;
    case MPU6050_ACCEL_FS_8:
      return 8;
    case MPU6050_ACCEL_FS_16:
      return 16;
    default:
      return 2;
  }
}

void SensorHand::setOffset(int16_t gyroOffset[3], int16_t accelOffset[3])
{
  mpu.setXGyroOffset(gyroOffset[0]);
  mpu.setYGyroOffset(gyroOffset[1]);
  mpu.setZGyroOffset(gyroOffset[2]);
  mpu.setXAccelOffset(accelOffset[0]);
  mpu.setYAccelOffset(accelOffset[1]);
  mpu.setZAccelOffset(accelOffset[2]);
}

bool SensorHand::imuInit(uint8_t address, uint8_t fullScaleGyroRange, uint8_t fullScaleAccelRange, int16_t gyroOffset[3], int16_t accelOffset[3], uint8_t calibrateCount)
{
  mpu = MPU6050(address);

  this->fullScaleGyroRange = fullScaleGyroRange;
  this->fullScaleAccelRange = fullScaleAccelRange;
  this->calibrateCount = calibrateCount;

  this->sensorFullScaleGyroValue = convertGyroRangeValue(fullScaleGyroRange);
  this->sensorFullScaleAccelValue = convertAccelRangeValue(fullScaleAccelRange);

  Wire.begin();
  mpu.initialize();

  // センサーの動作確認
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    isImuInitialized = false;
    return false;
  }

  mpu.setFullScaleAccelRange(fullScaleAccelRange);
  accelScale = sensorFullScaleAccelValue / (float)sensorFullScaleValue;

  mpu.setFullScaleGyroRange(fullScaleGyroRange);
  gyroScale = sensorFullScaleGyroValue / (float)sensorFullScaleValue;

  setOffset(gyroOffset, accelOffset);

  // 6回ずつ校正
  mpu.CalibrateAccel(calibrateCount);
  mpu.CalibrateGyro(calibrateCount);

  isImuInitialized = true;

  return true;
}

void SensorHand::updateImuData()
{
  if (!isImuInitialized)
    return;
  
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  // 単位をG、deg/sに変換する
  scaledAx = rawAx * accelScale;
  scaledAy = rawAy * accelScale;
  scaledAz = rawAz * accelScale;
  scaledGx = rawGx * gyroScale;
  scaledGy = rawGy * gyroScale;
  scaledGz = rawGz * gyroScale;
}

void SensorHand::printImuScaledData()
{
  if (!isImuInitialized)
    return;

  Serial.print(scaledAx); Serial.print(",");
  Serial.print(scaledAy); Serial.print(",");
  Serial.print(scaledAz); Serial.print(",");
  Serial.print(scaledGx); Serial.print(",");
  Serial.print(scaledGy); Serial.print(",");
  Serial.println(scaledGz);
}
