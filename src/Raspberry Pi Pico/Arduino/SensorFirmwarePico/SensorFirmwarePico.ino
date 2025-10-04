#include "SensorHand.h"
#include <MPU6050_6Axis_MotionApps612.h>
#include <MadgwickAHRS.h>

SensorHand left, right;

// 校正結果: 8g, 2000deg/s [-5113,-5113] [-69,-68] [937,938] [138,139] [7,8] [-64,-63] 
int16_t gyroOffsetL[3] = { -5113, -69, 937 };
int16_t accelOffsetL[3] = { 138, 7, -64 };

// 校正結果: 8g, 2000deg/s [-2797,-2796], [-449,-448], [947,947], [49,50], [19,20], [-14,-13] 
int16_t gyroOffsetR[3] = { -2796, -449, 947 };
int16_t accelOffsetR[3] = { 49, 19, -14 };

uint8_t keyBindingL[NUMBER_OF_INPUT] = {'d', 'd', 'f', 'f', 'f'};
uint8_t keyBindingR[NUMBER_OF_INPUT] = {'j', 'j', 'j', 'k', 'k'};

unsigned long lastExecutedTime = 0;
unsigned long dt;
unsigned long interval = 10 * (int)pow(10, 3); // intervalに設定したμsごとにループが実行

const uint8_t STATE_LED_PIN = LED_BUILTIN;
const uint8_t OK_LED_PIN = 26;
const uint8_t ERROR_LED_PIN = 27;
bool isLighting = false;

void setup()
{
  pinMode(STATE_LED_PIN, OUTPUT);
  digitalWrite(STATE_LED_PIN, HIGH);

  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(OK_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, HIGH);
  
  Serial.begin(115200);
  Serial.println("Init MPU");
  bool leftReady = left.imuInit(MPU6050_ADDRESS_AD0_HIGH, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_8, gyroOffsetL, accelOffsetL, 6);
  bool rightReady = right.imuInit(MPU6050_ADDRESS_AD0_LOW, MPU6050_GYRO_FS_2000, MPU6050_ACCEL_FS_8, gyroOffsetR, accelOffsetR, 6);
  Serial.println("Init MadgwickFilter");
  bool filterReadyL = left.madgwickFilterInit( 1.0f / (interval * pow(10, -6)) );
  bool filterReadyR = right.madgwickFilterInit( 1.0f / (interval * pow(10, -6)) );
  Serial.println("Set KeyBinding");
  left.setKeyBinding(keyBindingL);
  right.setKeyBinding(keyBindingR);

  if (!rightReady || !leftReady)
  {
    while (true)
    { // システムが動作不可のため、ループしてエラーを示す
      Serial.print("MPU6050");
      if (!leftReady) Serial.print("L");
      if (!rightReady) Serial.print("R");
      Serial.println(" connection failed");
      digitalWrite(ERROR_LED_PIN, isLighting);
      isLighting = !isLighting;
      delay(1000);
    }
  }
  Serial.println("MPU6050 connection success");

  if (!filterReadyR || !filterReadyL)
  {
    while (true)
    { // システムが動作不可のため、ループしてエラーを示す
      Serial.println("Madgwick filter init failed");
      digitalWrite(ERROR_LED_PIN, isLighting);
      isLighting = !isLighting;
      delay(1000);
    }
  }

  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, LOW);
}


void loop()
{
  /* ========== time measurement  ========== */
  // 経過時間
  dt = micros() - lastExecutedTime;

  // 最後に実行したときから{interval} μs経っていなかったときはループ内の処理を実行しない
  if (dt < interval)
    return;

  // 最後にループ内の処理が実行された時間を記憶
  lastExecutedTime = micros();

  right.updateImuData();
  left.updateImuData();
  right.applyFilter();
  left.applyFilter();
  right.updateAttitude();
  left.updateAttitude();

  right.strikeDitection();
  left.strikeDitection();
}