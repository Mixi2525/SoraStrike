class HorizontalSlider
{
  float x, y, w, h;
  float knobX, knobY, knobWidth, knobHeight, knobRadius;
  float minValue, maxValue;
  private float value;
  private float preValue;

  color normalColor, hoverColor;

  private Runnable onValueChanged;

  private Button knob;
  private boolean isMousePressedOnKnob = false;
  private boolean prevMousePressed = false;

  HorizontalSlider(float x, float y, float w, float h, float minValue, float maxValue, float value, float knobRadius)
  {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.minValue = minValue;
    this.maxValue = maxValue;
    this.value = constrain(value, minValue, maxValue);

    knobWidth = 10;
    knobHeight = 20;
    knobX = x;
    knobY = (y + h / 2) - knobHeight / 2;
    this.knobRadius = knobRadius;

    // スライダーのノブをボタンで生成
    knob = new Button(knobX, knobY, knobWidth, knobHeight, knobRadius, "");

    knob.setColors(color(150, 80, 80), color(170, 80, 80), color(130, 60, 60));

    normalColor = color(190, 190, 190);
    hoverColor = color(220, 220, 220);
 }

  void setOnValueChanged(Runnable action)
  {
    this.onValueChanged = action;
  }

  float getValue()
  {
    return value;
  }

  void setValue(float value)
  {
    float preValue = this.value;
    this.value = constrain(value, minValue, maxValue);
    if (onValueChanged != null && preValue != this.value)
    {
      onValueChanged.run();
    }
  }


  void update()
  {
    boolean _mousePressed = mousePressed;

    // 値更新
    if ((isHovered()) && _mousePressed)
    {
      value = map(mouseX, x, x + w, minValue, maxValue);
      value = constrain(value, minValue, maxValue);
    }

    // コールバック処理
    if (onValueChanged != null && preValue != value)
    {
      onValueChanged.run();
    }

    prevMousePressed = _mousePressed;
    preValue = value;
  }

  void draw()
  {
    pushStyle();

    fill(normalColor);
    noStroke();
    rect(x, y, w, h); // スライダー本体

    // ノブ更新・描画
    knob.x = map(value, minValue, maxValue, x, x + w) - knob.w / 2;
    knob.y = (y + h / 2) - knob.h / 2;
    knob.update();
    knob.draw();

    popStyle();
  }


  boolean isHovered()
  {
    return mouseX >= x && mouseX <= x + w && mouseY >= knobY && mouseY <= knobY + knobHeight;
  }
}