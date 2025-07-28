/* Processing Libraries */
import processing.serial.*;

/* Java Libraries */
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.awt.Robot;
import java.awt.AWTException;
import java.awt.event.KeyEvent;
import java.util.Iterator;
import java.awt.Cursor;
import java.util.Arrays;

/* Libraries */
import ddf.minim.*;

Robot robot;
int keyPressTime = 40;
long lastKeyPressTime = 0; // 最後にキーが押された時間
int pressedKeyCode = 0; // 現在押されているキーのコード
int keyToPressNext = 0; // serialEventからdrawに渡す、次に押すキーのコード

boolean isEnableSound = false; // 音を鳴らすかどうか
boolean isEnableKeyPress = true; // キーボード入力を行うかどうか
boolean isPrintLog = false;

Serial myPort;

BinaryFloatDataInfo binaryFloatDataInfo = new BinaryFloatDataInfo(false, 0, 0, "");

float ypr[] = new float[3]; 
float quat[] = new float[4];
float linerAccel[] = new float[3];
float scaleFactor = 100.0; // フェーダーで調整可能にするための初期値
float gain = 0.5f; // MadgwickフィルタのGain調整用 (0.00 - 1.00)
float receivedDeltaTime = 0;

public static final float EPS = 1.1920928955078125E-10f;

String mode = "";

enum State
{
  Idle,
  LowSwing,
  Stopping,
}

public static final ArrayList<String> states = new ArrayList<String>()
{
  {
    add(SingleCommandConstants.STATE_IDLE);
    add(SingleCommandConstants.STATE_LOW_SWING);
    add(SingleCommandConstants.STATE_STOPPING);
  }
};

State state = State.Idle;

Minim minim;
AudioPlayer dongSound;
AudioPlayer kaSound;

// イベント履歴表示用
float baseTime = 0;
float deltaTime = 0;
ArrayList<Note> noteList = new ArrayList<Note>();
float noteVelocity = 0;

// UIオブジェクト
Button soundToggleButton;
Button keyPressToggleButton;
Button setGainButton;

HorizontalSlider scaleSlider;
HorizontalSlider gainSlider;

void printLog(String message)
{
  if (isPrintLog)
    print(message);
} 

void printlnLog(String message)
{
  printLog(message + "\n");
}

void uiInitialize()
{
  // ボタンの初期化
  soundToggleButton = new Button(UIConstants.SOUND_BUTTON_X, UIConstants.SOUND_BUTTON_Y, 
                                 UIConstants.BUTTON_WIDTH, UIConstants.BUTTON_HEIGHT, 
                                 UIConstants.BUTTON_CORNER_RADIUS, "Button");
  soundToggleButton.setOnReleasedAction(() -> {
    isEnableSound = !isEnableSound;
    println("Sound enabled: " + isEnableSound);
  });

  keyPressToggleButton = new Button(UIConstants.KEY_BUTTON_X, UIConstants.KEY_BUTTON_Y, 
                                    UIConstants.BUTTON_WIDTH, UIConstants.BUTTON_HEIGHT, 
                                    UIConstants.BUTTON_CORNER_RADIUS, "Button");
  keyPressToggleButton.setOnReleasedAction(() -> {
    isEnableKeyPress = !isEnableKeyPress;
    println("Key Press enabled: " + isEnableKeyPress);
  });

  setGainButton = new Button(UIConstants.SET_GAIN_BUTTON_X, UIConstants.SET_GAIN_BUTTON_Y, 
                             UIConstants.BUTTON_WIDTH, UIConstants.BUTTON_HEIGHT, 
                             UIConstants.BUTTON_CORNER_RADIUS, "Set Gain");
  setGainButton.setOnReleasedAction(() -> {
    String message = "SETGAIN " + nf(gain, 1, 2); // 例: "SETGAIN 0.50"
    myPort.write(message + '\n'); // シリアル通信で送信
    println("シリアル送信: " + message);
  });
  setGainButton.setColors(color(0, 200, 0), color(0, 220, 0), color(0, 180, 0));

  // スライダーの初期化
  scaleSlider = new HorizontalSlider(UIConstants.SCALE_SLIDER_X, UIConstants.SCALE_SLIDER_Y, 
                                     UIConstants.SCALE_SLIDER_WIDTH, UIConstants.SCALE_SLIDER_HEIGHT, 
                                     UIConstants.SCALE_FACTOR_MIN, UIConstants.SCALE_FACTOR_MAX, scaleFactor, 2);
  scaleSlider.setOnValueChanged(() -> {
    scaleFactor = scaleSlider.getValue();
    println("SCALE " + nf(scaleFactor, 1, 2));
  });

  gainSlider = new HorizontalSlider(UIConstants.GAIN_SLIDER_X, UIConstants.GAIN_SLIDER_Y,
                                    UIConstants.GAIN_SLIDER_WIDTH, UIConstants.GAIN_SLIDER_HEIGHT,
                                    UIConstants.GAIN_MIN, UIConstants.GAIN_MAX, gain, 2);
  gainSlider.setOnValueChanged(() -> {
    gain = gainSlider.getValue();
    println("GAIN " + nf(gain, 1, 2));
  });
}

void keyExecute()
{
  // 新しいキー押下要求があれば実行
  if (keyToPressNext != 0) {
    if (isEnableKeyPress) { // キー入力が有効な場合のみ実行
      robot.keyPress(keyToPressNext);
      lastKeyPressTime = millis(); // キーが押された時間を記録
    }
    pressedKeyCode = keyToPressNext; // 押されたキーコードを記録
    keyToPressNext = 0; // 処理したのでリセット
  }

  // 時間経過でキーを離す処理
  if (pressedKeyCode != 0 && millis() - lastKeyPressTime > keyPressTime) {
    if (isEnableKeyPress) { // キー入力が有効な場合のみ実行
      robot.keyRelease(pressedKeyCode);
    }
    pressedKeyCode = 0; // キーが離されたのでリセット
  }
}

void changeCursor()
{
  // OpenGLはOSで設定したマウス情報にアクセスできないためJavaデフォルトのカーソルが使用される
  if (soundToggleButton.isHovered() || 
      keyPressToggleButton.isHovered() || 
      setGainButton.isHovered() ||
      scaleSlider.isHovered() ||
      gainSlider.isHovered()) {
    cursor(HAND); 
  } else {
    cursor(ARROW);
  }
}

void applyRotate()
{
  // modeによって異なる回転の適用をする
  switch (mode)
  {
    case "YPR":
      // RPYに従って回転（順序：Yaw → Pitch → Roll）
      // ProcessingはYが高さ方向を表す左手座標系であることに注意！！Z to Y
       rotateY(radians( ypr[0]));   // Yaw（Z軸回転）
       rotateZ(radians(-ypr[1])); // Pitch（Y軸回転）
       rotateX(radians( ypr[2]));  // Roll（X軸回転）
    break;
    
    case "QUAT":
      float angle = 2.0f * acos(quat[0]);
      float s = (float) sqrt(1.0f - quat[0] * quat[0]);
      
      if (s < EPS)
      {
        s = 1.0f;
      }
      else
      {
        s = 1.0f / s;
      }
    
      float ax = quat[1] * s;
      float ay = quat[3] * s;
      float az = -quat[2] * s;
    
      rotate(angle, ax, ay, az);
    break;
  }
}

void setStateColor()
{
  // 状態によって色を変える
  switch (state)
  {
    case Idle:
      fill(StateColors.IDLE_R, StateColors.IDLE_G, StateColors.IDLE_B);
      break;
    case Stopping:
      fill(StateColors.STOPPING_R, StateColors.STOPPING_G, StateColors.STOPPING_B);
      break;
    case LowSwing:
      fill(StateColors.LOW_SWING_R, StateColors.LOW_SWING_G, StateColors.LOW_SWING_B);
      break;
  }
}

void displayLinerAccelLine()
{
  // 加速度ベクトルの可視化
  pushMatrix();  
  stroke(255, 255, 0);
  strokeWeight(3);

  // linerAccel[0]はボックスのローカル座標のX、linerAccel[1]はボックスのローカル座標のY、linerAccel[2]はボックスのローカル座標のZ
  // 加速度の強さを可視化するためにスケーリング
  line(0, 0, 0, linerAccel[0] * scaleFactor, linerAccel[1] * scaleFactor, linerAccel[2] * scaleFactor);
  noStroke();
  popMatrix();
}

void uiUpdate()
{
  fill(255);
  textSize(UIConstants.TEXT_SIZE_LARGE);

  // 現在の状態名を表示
  text("Current State: " + state.name(), UIConstants.TEXT_MARGIN_X, UIConstants.TEXT_MARGIN_Y_STATE);

  // スケール調整スライダー
  scaleSlider.update();
  scaleSlider.draw(); 

  fill(255);
  text("Scale Factor: " + nf(scaleFactor, 1, 2), UIConstants.TEXT_MARGIN_X, UIConstants.TEXT_MARGIN_Y_SCALE_FACTOR); // Scale Factorの値を表示


  // サウンドトグルボタン
  soundToggleButton.label = "Sound: " + (isEnableSound ? "ON" : "OFF");

  if (isEnableSound) {
    soundToggleButton.setColors(color(0, 200, 0), color(0, 220, 0), color(0, 180, 0));
  } else {
    soundToggleButton.setColors(color(200, 0, 0), color(220, 0, 0), color(180, 0, 0));
  }

  soundToggleButton.update();
  soundToggleButton.draw();


  // キー入力ボタン
  keyPressToggleButton.label = "Key Press: " + (isEnableKeyPress ? "ON" : "OFF");

  if (isEnableKeyPress) {
    keyPressToggleButton.setColors(color(0, 200, 0), color(0, 220, 0), color(0, 180, 0));
  } else {
    keyPressToggleButton.setColors(color(200, 0, 0), color(220, 0, 0), color(180, 0, 0));
  }

  keyPressToggleButton.update();
  keyPressToggleButton.draw();

  // Madgwick ゲイン調整スライダー
  gainSlider.update();
  gainSlider.draw();

  fill(255);
  textSize(UIConstants.TEXT_SIZE_MEDIUM);
  text("Madgwick Gain: " + nf(gain, 1, 2), UIConstants.TEXT_MARGIN_X, UIConstants.GAIN_SLIDER_Y + UIConstants.TEXT_MARGIN_Y_GAIN); // Gainの値を表示

  // ゲインボタン
  setGainButton.update();
  setGainButton.draw();

  textAlign(LEFT, BASELINE); // デフォルトに戻す

  noStroke();
}

public void settings()
{
  size(UIConstants.WINDOW_WIDTH, UIConstants.WINDOW_HEIGHT, P3D);
}

void setup() {
  print(Serial.list());

  try { // シリアルポートのインスタンス生成 ＆ シリアルポート開く
    myPort = new Serial(this, SerialConstants.PORT_NAME, SerialConstants.BAUD_RATE);
    myPort.bufferUntil('\n');
  } catch (java.lang.RuntimeException e) {
    e.printStackTrace();
    exit(); // 終了
  }

  try { // Robotインスタンス作成
    robot = new Robot();
  } catch (AWTException e) {
    e.printStackTrace();
    exit(); // 終了
  }

  noStroke();

  // 音源ファイル初期設定
  minim = new Minim(this);
  dongSound = minim.loadFile("Audio/dong.wav");
  kaSound = minim.loadFile("Audio/ka.wav");

  dongSound.setGain(AudioConstants.DEFAULT_GAIN);
  kaSound.setGain(AudioConstants.DEFAULT_GAIN);

  // ウィンドウの端から端までを1000 msで移動するときの速度px/msを計算
  // 1 msでどのくらいのpx分移動するか
  noteVelocity = width / NoteConstants.TRAVEL_TIME_MS;

  // UIコンポーネントの初期設定
  uiInitialize();

  baseTime = millis();
}

void draw() {
  background(75);
  
  keyExecute(); // バッファに溜まっているキー入力の実行
  changeCursor();  // UI要素のホバー状態に応じてカーソルを変更

  /* ===== 3Dオブジェクト描画 ===== */
  lights();
  translate(width / 2, height / 2, 0);
  camera(0, -50, 500, 0, 0, 0, 0, 1, 0); // カメラ位置、注視点、上方向ベクトル

  applyRotate();  // 姿勢の情報を適用
  setStateColor();  // 振り下ろしの状態によって色を反映
  box(30, 30, 150);  // スティック（に見立てたボックス）を表示
  displayLinerAccelLine(); // ボックス基準の動加速度表示

  /* ===== UI描画 ===== */
  hint(DISABLE_DEPTH_TEST);
  camera();
  perspective();

  // UIの更新と描画
  uiUpdate();

  /* ===== イベント履歴表示 ===== */
  deltaTime = millis() - baseTime;  // 経過時間を計算
  baseTime = millis();

  synchronized (noteList) // serialEventでのnoteList.addの処理とぶつからないための同期ブロック
  {
    // イテレータで削除
    Iterator<Note> it = noteList.iterator();

    while (it.hasNext())
    {
      Note note = it.next();
      note.move(deltaTime);
      note.draw();
      if (note.x > width)
        it.remove();
    }
  }
  hint(ENABLE_DEPTH_TEST);
}

void keyPressed() {
  if (keyCode == KeyConstants.KEY_F3)
  {
    isPrintLog = !isPrintLog;
    println("isPrintLog: " + isPrintLog);
  }  
}

/* ======= シリアル通信＆データ処理関連 ======= */
float receiveFloat(Serial p)
{
  // 4B読み取り
  byte[] floatBytes = new byte[4];
  floatBytes[0] = (byte)p.read();  // LSB
  floatBytes[1] = (byte)p.read();
  floatBytes[2] = (byte)p.read();
  floatBytes[3] = (byte)p.read();  // MSB
  
  // バイト配列をfloatに変換
  float receivedFloat = bytesToFloat(floatBytes);
  return receivedFloat;
}

float bytesToFloat(byte[] bytes)
{
  ByteBuffer buffer = ByteBuffer.wrap(bytes);
  buffer.order(ByteOrder.LITTLE_ENDIAN);  // リトルエンディアン指定
  return buffer.getFloat();
}

void assignFloatValue(float receivedValue, float[] assignedArray, Runnable onReceiveCompleteAction)
{
  if (binaryFloatDataInfo.numberOfDataCurrentlyReceived < 0 || assignedArray.length <= binaryFloatDataInfo.numberOfDataCurrentlyReceived)
  {
    printlnLog("!!Invalid numberOfDataCurrentlyReceived: " + binaryFloatDataInfo.numberOfDataCurrentlyReceived + ", assignedArray.length: " + assignedArray.length);
    return;
  }

  assignedArray[binaryFloatDataInfo.numberOfDataCurrentlyReceived] = receivedValue;
  binaryFloatDataInfo.numberOfDataCurrentlyReceived++;

  if ( onReceiveCompleteAction != null && binaryFloatDataInfo.numberOfDataCurrentlyReceived == assignedArray.length)
    onReceiveCompleteAction.run();

}

void receiveBinaryFloatData(Serial p)
{
  // 所定のデータ数だけ繰り返して読み込む
  for (int i = 0; i < binaryFloatDataInfo.numberOfDataToReceive; i++) {
    float receivedValue = receiveFloat(p);

    // 受信したfloat値を対応する変数に割り当てる
    if (binaryFloatDataInfo.expectedDataType.equals(DataConstants.DATA_YPR))
    {
      assignFloatValue(receivedValue, ypr, () -> {
        mode = "YPR";
        printlnLog("YPR: Yaw=" + ypr[0] + ", Pitch=" + ypr[1] + ", Roll=" + ypr[2]);
      });
    } 
    else if (binaryFloatDataInfo.expectedDataType.equals(DataConstants.DATA_QUAT))
    {
      assignFloatValue(receivedValue, quat, () -> {
        mode = "QUAT";
        printlnLog("QUAT: w=" + quat[0] + ", x=" + quat[1] + ", y=" + quat[2] + ", z=" + quat[3]);
      });
    } 
    else if (binaryFloatDataInfo.expectedDataType.equals(DataConstants.DATA_LINERACCEL))
    {
      assignFloatValue(receivedValue, linerAccel, () -> {
        printlnLog("LINERACCEL: X=" + linerAccel[0] + ", Y=" + linerAccel[1] + ", Z=" + linerAccel[2]);
      });
    } 
    else if (binaryFloatDataInfo.expectedDataType.equals(DataConstants.DATA_BETA))
    {
      float tmpGain[] = new float[1];
      assignFloatValue(receivedValue, tmpGain, () -> {
        gain = tmpGain[0];
        printlnLog("BETA: " + gain);
      });
    } 
    else if (binaryFloatDataInfo.expectedDataType.equals(DataConstants.DATA_DT)) 
    {
      float tmpDeltaTime[] = new float[1];
      assignFloatValue(receivedValue, tmpDeltaTime, () -> {
        receivedDeltaTime = tmpDeltaTime[0];
        printlnLog("DT: " + receivedDeltaTime);
      });
    } 
  }
}

void handleSyncData(String parts[], String receivedLine)
{
  String dataType = parts[0];
  try {
    int num = Integer.parseInt(parts[1]);

    switch (dataType) {
      case DataConstants.DATA_YPR: case DataConstants.DATA_QUAT: case DataConstants.DATA_LINERACCEL: case DataConstants.DATA_BETA: case DataConstants.DATA_DT:
        binaryFloatDataInfo.setValue(true, num, 0, dataType);
         myPort.buffer(binaryFloatDataInfo.numberOfDataToReceive * SerialConstants.BYTE_FLOAT);
        break;
      default:
        printlnLog("未知のデータ種別を受信 (同期文字列): " + dataType + " (" + num + "個)");
        myPort.bufferUntil('\n');
        break;
    }
  } catch (NumberFormatException e) {
    printlnLog("個数の解析エラー (同期文字列): " + receivedLine);
    myPort.bufferUntil('\n');
  }
}

void playSound(AudioPlayer audio)
{
  if (isEnableSound) {
    audio.rewind();
    audio.play();
  }
}

void analysisSingleCommand(String parts[], String receivedLine)
{
  if (receivedLine.startsWith(SingleCommandConstants.INPUT))
  {
    // Input{number}: のパターンから数字を抽出
    String pattern = "Input(\\d+):";
    java.util.regex.Pattern r = java.util.regex.Pattern.compile(pattern);
    java.util.regex.Matcher m = r.matcher(receivedLine);
        
    Note newNote = null;
    if (m.find()) {
      String inputNumber = m.group(1); // 数字部分を取得

      if (inputNumber.equals("0") || inputNumber.equals("1") || inputNumber.equals("2"))
      {
        println(String.format(SerialConstants.LOG_FORMAT_EVENT, hour(), minute(), second(), "ドン"));
        playSound(dongSound);
        keyToPressNext = KeyConstants.KEY_DONG;
        newNote = new Note(NoteId.Dong, NoteConstants.NOTE_RADIUS, NoteConstants.NOTE_RADIUS, NoteConstants.NOTE_RADIUS + 5, noteVelocity, 0);
      }
      else if (inputNumber.equals("3") || inputNumber.equals("4"))
      {
        println(String.format(SerialConstants.LOG_FORMAT_EVENT, hour(), minute(), second(), "カッ"));
        playSound(kaSound);
        keyToPressNext = KeyConstants.KEY_KA;
        newNote = new Note(NoteId.Ka, NoteConstants.NOTE_RADIUS, NoteConstants.NOTE_RADIUS, NoteConstants.NOTE_RADIUS + 5, noteVelocity, 0);
      }
      if (newNote != null) {
        synchronized (noteList) { noteList.add(newNote); }
      }
    }
  }
  else if (states.contains(parts[0]))
  { // 送られてきた動作状態に応じてStateを設定
    state = State.valueOf(parts[0]);
    printlnLog(String.format(SerialConstants.LOG_FORMAT_EVENT, hour(), minute(), second(), parts[0]));
  }
  else if (receivedLine.isEmpty())
  {
    // 何もしない
  }
  else
  {
    // 未知の単発コマンド
    printlnLog("未知の単発コマンド: " + receivedLine);
  }
}

void serialEvent(Serial p) {
  if (binaryFloatDataInfo.expectingData) 
  {
    if (p.available() < binaryFloatDataInfo.numberOfDataToReceive * SerialConstants.BYTE_FLOAT)
      return;

    // バイナリデータ（float 4B）受信状態
    receiveBinaryFloatData(p);
    binaryFloatDataInfo.reset();
    myPort.bufferUntil('\n'); // 次の同期文字列または単発コマンドのために改行までバッファリング
    return;
  }

  // 同期文字列または単発コマンドを待っている場合
  String receivedLine = p.readStringUntil('\n');
  if (receivedLine == null) {
    return;
  }
  receivedLine = trim(receivedLine);
  String[] parts = split(receivedLine, ' ');

  if (parts.length == 2) 
  { // {送られるデータ種別} {個数}のフォーマットと仮定
    handleSyncData(parts, receivedLine);
  }
  else
  {
    // 同期文字列の形式ではない場合は、単発コマンドとして処理
    analysisSingleCommand(parts, receivedLine);
    myPort.bufferUntil('\n');
  }
}

