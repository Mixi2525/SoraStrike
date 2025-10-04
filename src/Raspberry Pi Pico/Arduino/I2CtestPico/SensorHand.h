#ifndef _SENSOR_HAND_H_
#define _SENSOR_HAND_H_

#define ENABLE_DEBUG

// 使用するフィルタの指定
// #define USE_NONE
// #define USE_AVERAGE
#define USE_MEDIAN

#define NUMBER_OF_INPUT 5

#include <MPU6050_6Axis_MotionApps612.h>
#include <MadgwickAHRS.h>
#include <I2Cdev.h>



class SensorHand
{
private:
  MPU6050 mpu;
  Madgwick madgwickFilter;

  bool isImuInitialized = false;
  bool isFilterInitialized = false;
  bool isKeyBindingInitialized = false;

  /* ===== Sensor Settings =====*/
  float accelScale, gyroScale;
  int fullScaleGyroRange;
  int fullScaleAccelRange;
  uint16_t sensorFullScaleGyroValue;
  uint8_t sensorFullScaleAccelValue;
  uint16_t sensorFullScaleValue = 32768.0;
  uint8_t calibrateCount = 6;

  /* ===== Sensor Data ===== */
  typedef float (*FilterFunctionPtr)(float buffer[][3], int axis); // フィルタ用関数をまとめて扱うための関数ポインタ
  FilterFunctionPtr filterFunc; // フィルタ関数を格納する関数ポインタ

  const static int windowSize = 4;

  int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  float scaledAx, scaledAy, scaledAz, scaledGx, scaledGy, scaledGz;

  float accelBuffer[windowSize][3] = { 0 };
  float gyroBuffer[windowSize][3] = { 0 };

  float preFilteredAccel[3] = { 0 };
  float preFilteredGyro[3] = { 0 };
  float filteredAccel[3] = { 0 };
  float filteredGyro[3] = { 0 };

  float gravity[3] = { 0 };
  float linerAccel[windowSize][3] = { 0 };
  float filteredLinerAccel[3] = { 0 };

  int preBufferIndex = windowSize - 1;
  int bufferIndex = 0;

  /* ===== Attitude ===== */
  float ypr[3];
  Quaternion q;
  float quat[4];

  /* ===== System Control ===== */
  float impactLinerAccelThreshold = 1.9f; // 打撃したと判定される加速度
  float beginLowSwingLinerAccelZThreshold = -1.0f; // 振り下ろし始めと判定されるZ軸動加速度
  float beginLowSwingGyroXThreshold = -300.0f; // 振り下ろし始めと判定されるX軸角速度
  float failureLowSwingGyroXThreshold = -200.0f; // 振り下ろし中に打撃をやめたとみなすX軸角速度
  float StoppingAccelThresholdFactor = 0.9f; // 状態StoppingからIdleへ移行するときの加速度の係数　ベースは打撃したと判定される加速度

  unsigned long startLowSwingTime = 0;
  unsigned long endLowSwingTime = 0;

  enum class State
  {
    Idle,
    LowSwing,
    Stopping,
  };

  State state = State::Idle;
  String stateText = "Idle";

  uint8_t keyBinding[NUMBER_OF_INPUT];

  /* ====== Input Thresholds ====== */
  float input0Pitch = -55.0f;
  float input1Pitch = -27.5f;
  float input2Pitch =   0.0f;
  float input3Pitch =  27.5f;
  float input4Pitch =  55.0f;

  float inputPitchThresholds[4] = { (input0Pitch + input1Pitch) / 2.0f,
                                    (input1Pitch + input2Pitch) / 2.0f,
                                    (input2Pitch + input3Pitch) / 2.0f,
                                    (input3Pitch + input4Pitch) / 2.0f  };

  /* ===== General =====*/
  bool isInitialized();
  
  /* ===== Sensor Settings & Processes ===== */
  int convertGyroRangeValue(uint8_t fullScaleGyroRange);
  int convertAccelRangeValue(uint8_t fullScaleAccelRange);
  void setOffset(int16_t gyroOffset[3], int16_t accelOffset[3]);
  void calcLinerAccel();

  /* ===== Filter Functions ===== */
  void filterInit();
  float filter(float buffer[][3], int axis, FilterFunctionPtr filterFunc);
#ifdef USE_AVERAGE
  static float averageFilter(float buffer[][3], int axis);
#endif
#ifdef USE_MEDIAN
  static void bubbleSort(float arr[], int size);
  static float medianFilter(float buffer[][3], int axis);

#endif

public:
  /* ===== Constructor ===== */
  SensorHand();
  SensorHand(uint8_t keyBinding[NUMBER_OF_INPUT]);
  
  /* ===== Sensor Settings & Processes ===== */
  bool imuInit(uint8_t address, uint8_t fullScaleGyroRange, uint8_t fullScaleAccelRange, int16_t gyroOffset[3], int16_t accelOffset[3], uint8_t calibrateCount = 6);
  void updateImuData();
  
  /* ===== Filter Functions ===== */
  bool madgwickFilterInit(float sampleFrequency);
  void applyFilter();
  
  /* ===== Attitude ===== */
  void updateAttitude();
  
  /* ===== System Control & Settings ===== */
  void setKeyBinding(uint8_t keyBinding[NUMBER_OF_INPUT]);
  void strikeDitection();
  void printState();
  
  /* ===== Debug ===== */
#ifdef ENABLE_DEBUG
  void printDebug();
  void printImuScaledData();
  void printImuFilteredData();
  void printYPR();
#endif /* ENABLE_DEBUG */
  
};

#endif /* _SENSOR_HAND_H_ */