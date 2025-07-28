// #define OUTPUT_YPR // Roll Pitch Yaw
// #define OUTPUT_QUATERNION // 四元数

// #define OUTPUT_LINERACCEL // 動加速度
// #define OUTPUT_GYRO // 角速度
// #define OUTPUT_LINERACCEL_WORLD // ワールド座標の動加速度

// #define OUTPUT_STATEINFO
#define OUTPUT_DT

// 可視化のための包括的な出力設定
#define OUTPUT_VISUALIZATION_DATA

// 使用するフィルタの指定
// #define USE_NONE
// #define USE_AVERAGE
#define USE_MEDIAN

// コマンド入力を許可するかの指定
// #define ARROW_COMMAND

// #define USE_MEGNETOMETER

#include <MPU6050_6Axis_MotionApps612.h>
#include <MadgwickAHRS.h>
#include <I2Cdev.h>

#include <Keyboard.h>

MPU6050 mpu;
Madgwick madgwickFilter;

/* ===== センサ・システムの設定 =====*/
unsigned long transmissionSpeed = 115200;

float accelScale, gyroScale;
int fullScaleAccelRange = MPU6050_ACCEL_FS_8;
int fullScaleGyroRange = MPU6050_GYRO_FS_2000;
uint8_t sensorFullScaleAccel = 8;
uint16_t sensorFullScaleGyro = 2000;
uint16_t sensorFullScaleValue = 32768.0;
uint8_t calibrateCount = 6;

// 校正結果: 8g, 2000deg/s [-2797,-2796], [-449,-448], [947,947], [49,50], [19,20], [-14,-13] 
int16_t gyroOffset[3] = { -2796, -449, 947 };
int16_t accelOffset[3] = { 49, 19, -14 };

// [-5113,-5113] 	[-69,-68] 	[937,938] 	[138,139] 	[7,8] 	[-64,-63] 

const uint8_t STATE_LED_PIN = LED_BUILTIN;
const uint8_t OK_LED_PIN = 26;
const uint8_t ERROR_LED_PIN = 27;

bool isLedBlinking = true;

unsigned long lastExecutedTime = 0;
unsigned long dt;
unsigned long interval = 10 * (int)pow(10, 3); // intervalに設定したμsごとにループが実行
unsigned long startLowSwingTime = 0;
unsigned long endLowSwingTime = 0;

/* ===== センサデータ関連 ===== */
typedef float (*FilterFunctionPtr)(float buffer[][3], int axis); // フィルタ用関数をまとめて扱うための関数ポインタ
FilterFunctionPtr filterFunc; // フィルタ関数を格納する関数ポインタ

const int windowSize = 4;

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

float sort_elems[windowSize] = { 0 };
int preBufferIndex = windowSize - 1;
int bufferIndex = 0;

/* ===== 制御情報 ===== */
float ypr[3];
Quaternion q;
float quat[4];

float ImpactLinerAccelThreshold = 1.9f; // 打撃したと判定される加速度
float beginLowSwingLinerAccelZThreshold = -1.0f; // 振り下ろし始めと判定されるZ軸動加速度
float beginLowSwingGyroXThreshold = -300.0f; // 振り下ろし始めと判定されるX軸角速度
float failureLowSwingGyroXThreshold = -200.0f; // 振り下ろし中に打撃をやめたとみなすX軸角速度
float StoppingAccelThresholdFactor = 0.9f; // 状態StoppingからIdleへ移行するときの加速度の係数　ベースは打撃したと判定される加速度

enum class State
{
  Idle,
  LowSwing,
  Stopping,
};

State state = State::Idle;
String stateText = "Idle";

/* ====== 入力閾値設定 ====== */
float input0Pitch = -55.0f;
float input1Pitch = -27.5f;
float input2Pitch =   0.0f;
float input3Pitch =  27.5f;
float input4Pitch =  55.0f;

float inputPitchThresholds[] = {(input0Pitch + input1Pitch) / 2.0f,
                                (input1Pitch + input2Pitch) / 2.0f,
                                (input2Pitch + input3Pitch) / 2.0f,
                                (input3Pitch + input4Pitch) / 2.0f };


#ifdef USE_MEGNETOMETER
#include <QMC5883LCompass.h>

QMC5883LCompass compass;
int rawMx, rawMy, rawMz;

// ハードアイアンの補正値
float hard_iron_offset[3] = { -87.5000f, 38.5000f, -143.0000f };

// ソフトアイアンの補正行列
float soft_iron_matrix[3][3] = {
  { 1.1070f, 0.0000f, 0.0000f },
  { 0.0000f, 1.0028f, 0.0000f },
  { 0.0000f, 0.0000f, 0.9096f },
};

float initialYawOffset = 0.0f;
int calibrationSampleCount = 0;
const int CALIBRATION_SAMPLES = 100;
float yawSum = 0.0f;

enum class InitializationState {
  WaitingForMadgwickStabilization,
  CollectingYawOffsetSamples,
  CalibrationComplete
};

InitializationState initializationState = InitializationState::WaitingForMadgwickStabilization;
unsigned long madgwickStabilizationStartTime = 0;
const unsigned long MADGWICK_STABILIZATION_DELAY_MS = 20000;
#endif

/* Filter Functions */
float filter(float buffer[][3], int axis, FilterFunctionPtr filterFunc)
{
  return filterFunc(buffer, axis);
}

#ifdef USE_AVERAGE
float averageFilter(float buffer[][3], int axis)
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
void bubbleSort(float arr[], int size) {
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

float medianFilter(float buffer[][3], int axis)
{
  // 配列の中身をコピー
  for(int i = 0; i < windowSize; i++)
  {
    sort_elems[i] = buffer[i][axis];
  }

  // ソート
  bubbleSort(sort_elems, windowSize);

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
#endif

void applyFilter()
{
#ifdef USE_AVERAGE // 平均フィルタを適用
  filterFunc = averageFilter;
#endif
#ifdef USE_MEDIAN // 中央値フィルタを適用
  filterFunc = medianFilter;
#endif

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

void getSensorData()
{
  // センサの生の出力値を取得
  mpu.getMotion6(&rawAx, &rawAy, &rawAz, &rawGx, &rawGy, &rawGz);

#ifdef USE_MEGNETOMETER
  compass.read();
  rawMx = compass.getX();
  rawMy = compass.getY();
  rawMz = compass.getZ();

  // float corrected_mx = ((float)raw_mx - hard_iron_offset[0]) * soft_iron_matrix[0][0];
  // float corrected_my = ((float)raw_my - hard_iron_offset[1]) * soft_iron_matrix[1][1];
  // float corrected_mz = ((float)raw_mz - hard_iron_offset[2]) * soft_iron_matrix[2][2];

  float mx = -rawMx;
  float my =  rawMy;
  float mz =  rawMz;
#endif

  // 単位をG、deg/sに変換する
  scaledAx = rawAx * accelScale;
  scaledAy = rawAy * accelScale;
  scaledAz = rawAz * accelScale;
  scaledGx = rawGx * gyroScale;
  scaledGy = rawGy * gyroScale;
  scaledGz = rawGz * gyroScale;

  // 加速度バッファ更新
  accelBuffer[bufferIndex][0] = scaledAx;
  accelBuffer[bufferIndex][1] = scaledAy;
  accelBuffer[bufferIndex][2] = scaledAz;

  // ジャイロバッファ更新
  gyroBuffer[bufferIndex][0] = scaledGx;
  gyroBuffer[bufferIndex][1] = scaledGy;
  gyroBuffer[bufferIndex][2] = scaledGz;
}

void calcLinerAccel()
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

// 地磁気センサYaw校正
#ifdef USE_MEGNETOMETER
void megnetometerOffsetCalibration()
{
// 初期オフセットの校正処理
 switch (initializationState) {
    case InitializationState::WaitingForMadgwickStabilization:
      if (millis() - madgwickStabilizationStartTime >= MADGWICK_STABILIZATION_DELAY_MS) {
        initializationState = InitializationState::CollectingYawOffsetSamples;
        Serial.println("Madgwick stabilized. Starting yaw offset calibration...");
        // LEDを点滅させて、安定化完了とキャリブレーション開始を視覚的に通知するのも良いでしょう
        digitalWrite(ERROR_LED_PIN, LOW); // エラーLEDを消す
        digitalWrite(OK_LED_PIN, LOW); // OK LEDも一旦消す
      } else {
        // 安定化待機中の進捗を表示
        if (millis() / 1000 % 2 == 0) { // 2秒ごとに表示
          Serial.println("Waiting for Madgwick stabilization... " + String((millis() - madgwickStabilizationStartTime) / 1000.0f) + "s / " + String(MADGWICK_STABILIZATION_DELAY_MS / 1000.0f) + "s");
        }
        digitalWrite(ERROR_LED_PIN, isLedBlinking); // エラーLEDを点滅させて待機中であることを示す
        isLedBlinking = !isLedBlinking;
      }
      break;

    case InitializationState::CollectingYawOffsetSamples:
      if (calibrationSampleCount < CALIBRATION_SAMPLES) {
        yawSum += ypr[0];
        calibrationSampleCount++;
        
        if (calibrationSampleCount % 20 == 0) {
          Serial.println("Calibrating yaw offset... " + String(calibrationSampleCount) + "/" + String(CALIBRATION_SAMPLES));
        }
      } else {
        // 校正完了
        initialYawOffset = yawSum / CALIBRATION_SAMPLES;
        initializationState = InitializationState::CalibrationComplete;
        madgwickFilter.setBeta(0.1f);
        Serial.println("Yaw offset calibration completed. Offset: " + String(initialYawOffset));
        digitalWrite(OK_LED_PIN, HIGH); // OK LEDを点灯させてキャリブレーション完了を通知
        digitalWrite(ERROR_LED_PIN, LOW); // エラーLEDを消す
      }
      break;

    case InitializationState::CalibrationComplete:
      // 校正済みの場合、オフセットを適用
      ypr[0] = ypr[0] - initialYawOffset;
      
      // 0°〜360°の範囲に正規化
      while (ypr[0] < 0.0f) {
        ypr[0] += 360.0f;
      }
      while (ypr[0] >= 360.0f) {
        ypr[0] -= 360.0f;
      }
      break;
  }
}
#endif

void updateAttitude()
{
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
  
  #ifdef USE_MEGNETOMETER
  megnetometerOffsetCalibration();
  #endif
}

void sendFloatByte(float values[], int size)
{
  // floatとByteを共用！
  union FloatBytes {
    float f;
    byte b[4];
  } floatData;

  for (int i = 0; i < size; i++)
  {
    floatData.f = values[i];

    // Byteで送る
    Serial.write(floatData.b[0]);  // LSB
    Serial.write(floatData.b[1]);
    Serial.write(floatData.b[2]);
    Serial.write(floatData.b[3]);  // MSB
  }
}

/* === Data Transfer === */
#ifdef OUTPUT_DT
void sendDeltaTime()
{
  // dtをミリ秒に変換
  float dt_ms = dt * pow(10, -3);
  Serial.println("DT 1");
  sendFloatByte(&dt_ms, 1);
  Serial.print("\n");
}
#endif

void sendYPR()
{
  Serial.println("YPR 3");
  sendFloatByte(ypr, 3);
  Serial.print("\n");
}

void sendQuaternion()
{
  Serial.println("QUAT 4");
  sendFloatByte(quat, 4);
  Serial.print("\n");
}

void sendLinerAccel()
{
  Serial.println("LINERACCEL 3");
  sendFloatByte(filteredLinerAccel, 3);
  Serial.print("\n");
}

void sendGyro()
{
  Serial.print(filteredGyro[0]); Serial.print(",");
  Serial.print(filteredGyro[1]); Serial.print(",");
  Serial.println(filteredGyro[2]);
}

void sendVisualizationData()
{
  sendYPR();
  sendLinerAccel();
}

void sendOutput()
{
  #ifdef OUTPUT_DT
  sendDeltaTime();
  #endif

  // 可視化用出力を行うときはYPRと動加速度だけ表示する。他は強制的に表示しないようにする。
  #ifdef OUTPUT_VISUALIZATION_DATA
  sendVisualizationData();
  return;
  #endif

  #if defined(OUTPUT_YPR)
  sendYPR();
  #endif
    
  #if defined(OUTPUT_QUATERNION)
  sendQuaternion();
  #endif
  
  #if defined(OUTPUT_LINERACCEL)
  sendLinerAccel();
  #endif
  
  #if defined(OUTPUT_GYRO)
  sendGyro();
  #endif
}

void strikeDitection()
{
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

        if (inputNumber <= 2)
        {
          Keyboard.press('j');
        }
        else 
        {
          Keyboard.press('k');
        }

        Serial.println("Input" + String(inputNumber) + ":" + String(swingTime, 3) + "ms");
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

void setup() {
  pinMode(STATE_LED_PIN, OUTPUT);
  digitalWrite(STATE_LED_PIN, HIGH);

  Serial.begin(transmissionSpeed);

  pinMode(OK_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(OK_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, HIGH);

  Wire.begin();
  mpu.initialize();

  // センサーの動作確認
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while(1) {
      digitalWrite(ERROR_LED_PIN, isLedBlinking);
      isLedBlinking = !isLedBlinking;
      delay(500);
    }
  }

  mpu.setFullScaleAccelRange(fullScaleAccelRange);
  accelScale = sensorFullScaleAccel / (float)sensorFullScaleValue;

  mpu.setFullScaleGyroRange(fullScaleGyroRange);
  gyroScale = sensorFullScaleGyro / (float)sensorFullScaleValue;

  // オフセット設定
  mpu.setXGyroOffset(gyroOffset[0]);
  mpu.setYGyroOffset(gyroOffset[1]);
  mpu.setZGyroOffset(gyroOffset[2]);
  mpu.setXAccelOffset(accelOffset[0]);
  mpu.setYAccelOffset(accelOffset[1]);
  mpu.setZAccelOffset(accelOffset[2]);

  // 6回ずつ校正
  mpu.CalibrateAccel(calibrateCount);
  mpu.CalibrateGyro(calibrateCount);

  madgwickFilter.begin( 1.0f / (interval * pow(10, -6)) );

  Keyboard.begin();

#ifdef USE_MEGNETOMETER
  madgwickFilter.setBeta(0.5f);
  compass.init(); 

  compass.setCalibrationOffsets(-267.00, -116.00, -206.00);
  compass.setCalibrationScales(0.88, 0.95, 1.22);

  delay(1000);
  Serial.println("Yaw offset calibration starting...");

  madgwickStabilizationStartTime = millis();
#else
  digitalWrite(OK_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, LOW);
#endif
}

void loop() {
  /* ========== Serial Command Processing ========== */
#ifdef ARROW_COMMAND
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // 前後の空白を削除

    if (command.startsWith("SETGAIN ")) {
      String valueString = command.substring(8); // "SETGAIN " の後の部分を取得
      float newGain = valueString.toFloat(); // floatに変換

      if (newGain >= 0.0f && newGain <= 1.0f) {
        madgwickFilter.setBeta(newGain);
        Serial.print("Madgwick gain set to: ");
        Serial.println(newGain, 4); // 浮動小数点数を小数点以下4桁で表示
      } else {
        Serial.println("Error: Gain value must be between 0.0 and 1.0.");
      }
    }
  }
#endif

  /* ========== time measurement  ========== */
  // 経過時間
  dt = micros() - lastExecutedTime;

  // 最後に実行したときから{interval} μs経っていなかったときはループ内の処理を実行しない
  if (dt < interval)
    return;

  // 最後にループ内の処理が実行された時間を記憶
  lastExecutedTime = micros();
  
  /* ========== Main system processing ========== */
  // センサデータ取得＆バッファ格納
  getSensorData();

  // センサデータにフィルタ適用
  applyFilter();
  
  // 姿勢角更新
  updateAttitude();

  // Output
  sendOutput();

  // 打撃検知
  strikeDitection();
  
  #if defined(OUTPUT_STATEINFO) || defined(OUTPUT_VISUALIZATION_DATA)
  // 現在の状態を通知
  Serial.print(stateText);
  Serial.println(" ");
  // Serial.println(static_cast<int>(state));
  #endif

  // バッファインデックス更新
  preBufferIndex = bufferIndex;
  bufferIndex = (bufferIndex + 1) % windowSize;
}