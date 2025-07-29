#include "SensorHand.h"
#include <Keyboard.h>

SensorHand::SensorHand()
{

  filterInit();
}

SensorHand::SensorHand(uint8_t keyBinding[NUMBER_OF_INPUT])
{
  filterInit();
  setKeyBinding(keyBinding);
}

bool SensorHand::isInitialized()
{
  if (isImuInitialized && isFilterInitialized && isKeyBindingInitialized)
    return true;

  Serial.print("NOT INIT >> ");
  if (!isImuInitialized) Serial.print("I");
  if (!isFilterInitialized) Serial.print("F");
  if (!isKeyBindingInitialized) Serial.print("K");
  Serial.println();
  return false; 
}

/* ===== Sensor Settings & Processes ===== */
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
  
  mpu.setFullScaleGyroRange(fullScaleGyroRange);
  gyroScale = sensorFullScaleGyroValue / (float)sensorFullScaleValue;

  mpu.setFullScaleAccelRange(fullScaleAccelRange);
  accelScale = sensorFullScaleAccelValue / (float)sensorFullScaleValue;

  setOffset(gyroOffset, accelOffset);

  // 6回ずつ校正
  mpu.CalibrateGyro(calibrateCount);
  mpu.CalibrateAccel(calibrateCount);

  isImuInitialized = true;

  return true;
}

void SensorHand::updateImuData()
{
  if (!isInitialized())
    return;
  
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

  // 単位をG、deg/sに変換する
  scaledAx = rawAx * accelScale;
  scaledAy = rawAy * accelScale;
  scaledAz = rawAz * accelScale;
  scaledGx = rawGx * gyroScale;
  scaledGy = rawGy * gyroScale;
  scaledGz = rawGz * gyroScale;
  
  // ジャイロバッファ更新
  gyroBuffer[bufferIndex][0] = scaledGx;
  gyroBuffer[bufferIndex][1] = scaledGy;
  gyroBuffer[bufferIndex][2] = scaledGz;
  
  // 加速度バッファ更新
  accelBuffer[bufferIndex][0] = scaledAx;
  accelBuffer[bufferIndex][1] = scaledAy;
  accelBuffer[bufferIndex][2] = scaledAz;
}

void SensorHand::calcLinerAccel()
{
  // ワールド座標系での重力ベクトルを、姿勢角をもとにセンサ座標系へ変換（回転）
  gravity[0] = 2 * (q.x * q.z - q.w * q.y); 
  gravity[1] = 2 * (q.w * q.x + q.y * q.z);
  gravity[2] = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
  
  // 動加速度を計算
  linerAccel[bufferIndex][0] = scaledAx - gravity[0];
  linerAccel[bufferIndex][1] = scaledAy - gravity[1];
  linerAccel[bufferIndex][2] = scaledAz - gravity[2];
  
  // 動加速度フィルタ処理
  #if defined(USE_AVERAGE) || defined(USE_MEDIAN)
  filteredLinerAccel[0] = filter(linerAccel, 0, filterFunc);
  filteredLinerAccel[1] = filter(linerAccel, 1, filterFunc);
  filteredLinerAccel[2] = filter(linerAccel, 2, filterFunc);
  #endif
}

/* ===== Filter Functions ===== */
bool SensorHand::madgwickFilterInit(float sampleFrequency)
{
  madgwickFilter.begin(sampleFrequency);
  isFilterInitialized = true;
  return true;
}

void SensorHand::filterInit()
{
#ifdef USE_AVERAGE // 平均フィルタを適用
  filterFunc = &SensorHand::averageFilter;
#endif
#ifdef USE_MEDIAN // 中央値フィルタを適用
  filterFunc = &SensorHand::medianFilter;
#endif
}

float SensorHand::filter(float buffer[][3], int axis, FilterFunctionPtr filterFunc)
{
  return filterFunc(buffer, axis);
}

#ifdef USE_AVERAGE
float SensorHand::averageFilter(float buffer[][3], int axis)
{
  float sum = 0;

  for (int i = 0; i < windowSize; i++)
  {
    sum += buffer[i][axis];
  }
  return sum / windowSize;
}
#endif

#ifdef USE_MEDIAN
float SensorHand::medianFilter(float buffer[][3], int axis)
{
  // 配列の中身をコピー
  float sort_elems[windowSize];
  for(int i = 0; i < windowSize; i++)
  {
    sort_elems[i] = buffer[i][axis];
  }

  // ソート
  SensorHand::bubbleSort(sort_elems, windowSize);

  float result;
  if (windowSize % 2 == 1)
  { // 配列長が奇数
    result = sort_elems[windowSize / 2];
  }
  else
  { // 配列長が偶数
    result = (sort_elems[windowSize / 2 - 1] + sort_elems[windowSize / 2]) / 2.0f;
  }

  return result;
}

void SensorHand::bubbleSort(float arr[], int size)
{
  for (int i = 0; i < size - 1; ++i) {
    for (int j = 0; j < size - 1 - i; ++j) {
      if (arr[j] > arr[j + 1]) {
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}
#endif

void SensorHand::applyFilter()
{
  if (!isInitialized())
    return;

  preFilteredAccel[0] = filteredAccel[0];
  preFilteredAccel[1] = filteredAccel[1];
  preFilteredAccel[2] = filteredAccel[2];
  preFilteredGyro[0] = filteredGyro[0];
  preFilteredGyro[1] = filteredGyro[1];
  preFilteredGyro[2] = filteredGyro[2];

#if defined(USE_AVERAGE) || defined(USE_MEDIAN)
  // フィルタを適用
  filteredAccel[0] = filter(accelBuffer, 0, filterFunc);
  filteredAccel[1] = filter(accelBuffer, 1, filterFunc);
  filteredAccel[2] = filter(accelBuffer, 2, filterFunc);
  filteredGyro[0] = filter(gyroBuffer, 0, filterFunc);
  filteredGyro[1] = filter(gyroBuffer, 1, filterFunc);
  filteredGyro[2] = filter(gyroBuffer, 2, filterFunc);
#endif

}

/* ===== Attitude ===== */
void SensorHand::updateAttitude()
{
  if (!isInitialized())
    return;

  #ifdef USE_MEGNETOMETER
  // 加速度、ジャイロ、地磁気の9軸使用時はこっち
  madgwickFilter.update(scaledGx, scaledGy, scaledGz, 
                        scaledAx, scaledAy, scaledAz, 
                        rawMx, rawMy, rawMz);
  #else
  // 加速度、ジャイロのみ使用時はこっち
  madgwickFilter.updateIMU(scaledGx, scaledGy, scaledGz, 
                           scaledAx, scaledAy, scaledAz);
  #endif
  
  // Madgwickフィルタからクォータニオンを取得
  q = Quaternion(madgwickFilter.q0, madgwickFilter.q1, madgwickFilter.q2, madgwickFilter.q3);
  
  // 姿勢角のクォータニオンから動加速度を計算
  calcLinerAccel();

  // 各軸の回転角度を取得＆表示
  ypr[0] = madgwickFilter.getYaw();
  ypr[1] = madgwickFilter.getPitch();
  ypr[2] = madgwickFilter.getRoll();

  quat[0] = q.w;
  quat[1] = q.x;
  quat[2] = q.y;
  quat[3] = q.z;

  // バッファインデックス更新
  preBufferIndex = bufferIndex;
  bufferIndex = (bufferIndex + 1) % windowSize;  
}

/* ===== System Control ===== */
void SensorHand::setKeyBinding(uint8_t keyBinding[NUMBER_OF_INPUT])
{
  for (int i = 0; i < NUMBER_OF_INPUT; i++)
  {
    this->keyBinding[i] = keyBinding[i];
  }

  Keyboard.begin();

  isKeyBindingInitialized = true;
}


void SensorHand::strikeDitection()
{
  if (!isInitialized())
    return;

  // 打撃検知機構　（状態遷移）
  switch (state) {
    case State::LowSwing:
      // 十分な角速度を維持してなかったらIdleへ移行
      if (filteredGyro[0] >= failureLowSwingGyroXThreshold)
      {
        Serial.println("gyroX: " + String(filteredGyro[0]) + " | failure swing process. return to Idle State");
        state = State::Idle;
        stateText = "Idle";
      }
      
      // 急停止だったら
      if (filteredLinerAccel[2] > ImpactLinerAccelThreshold)
      {
        state = State::Stopping;
        stateText = "Stopping";
        endLowSwingTime = micros();

        float currentPitch = madgwickFilter.getPitch();
        float swingTime = (endLowSwingTime - startLowSwingTime) / 1000.0;

        // ピッチ値に基づいて入力番号を決定
        int inputNumber = 0;
        for (int i = 0; i < 4; i++) {
            if (currentPitch >= inputPitchThresholds[i]) {
                inputNumber = i + 1;
            }
        }
        
        // キー入力
        Keyboard.press(keyBinding[inputNumber]);

        Serial.println("Input" + String(inputNumber) + "(" + String((char)keyBinding[inputNumber]) + "):" + String(swingTime, 3) + "ms");
      }
      break;

    case State::Stopping:
      // 叩いた後、加速度が閾値以下になるまで急停止状態を維持
      // これにより、連続で検出されてしまうのを防ぐ
      if (filteredLinerAccel[2] < ImpactLinerAccelThreshold * StoppingAccelThresholdFactor)
      {
        Keyboard.releaseAll();
        state = State::Idle;
        stateText = "Idle";
      }
      break;

    case State::Idle:
      if (filteredLinerAccel[2] < beginLowSwingLinerAccelZThreshold && 
          filteredGyro[0] < beginLowSwingGyroXThreshold)
      {
        state = State::LowSwing;
        stateText = "LowSwing";
        startLowSwingTime = micros();
      }
      break;
  }
}

void SensorHand::printState()
{
  if (!isInitialized())
    return;

  Serial.println(stateText);
}

/* ===== Debug ===== */
#ifdef ENABLE_DEBUG
void SensorHand::printDebug()
{
  if (!isInitialized())
    return;

  Serial.print("filterdLinerAccelZ:");
  Serial.print(filteredLinerAccel[2]);
  Serial.print(", beginLowSwingLinerAccelZ:");
  Serial.print(beginLowSwingLinerAccelZThreshold);
  Serial.print(", ImpactLinerAccelThreshold:");
  Serial.print(ImpactLinerAccelThreshold);

  Serial.print(", filteredGyroX:");
  Serial.print(filteredGyro[0]);
  Serial.print(", beginLowSwingGyroX:");
  Serial.print(beginLowSwingGyroXThreshold);

  Serial.println("");
}

void SensorHand::printImuScaledData()
{
  if (!isInitialized())
    return;

  Serial.print(scaledAx); Serial.print(",");
  Serial.print(scaledAy); Serial.print(",");
  Serial.print(scaledAz); Serial.print(",");
  Serial.print(scaledGx); Serial.print(",");
  Serial.print(scaledGy); Serial.print(",");
  Serial.println(scaledGz);
}

void SensorHand::printImuFilteredData()
{
  if (!isInitialized())
    return;
  
  Serial.print(filteredAccel[0]); Serial.print(", ");
  Serial.print(filteredAccel[1]); Serial.print(", ");
  Serial.print(filteredAccel[2]); Serial.print(", ");
  Serial.print(filteredGyro[0]); Serial.print(", ");
  Serial.print(filteredGyro[1]); Serial.print(", ");
  Serial.println(filteredGyro[2]);
}

void SensorHand::printYPR()
{
  if (!isInitialized())
    return;

  Serial.print(ypr[0]); Serial.print(",");
  Serial.print(ypr[1]); Serial.print(",");
  Serial.println(ypr[2]);
}

#endif /* ENABLE_DEBUG */