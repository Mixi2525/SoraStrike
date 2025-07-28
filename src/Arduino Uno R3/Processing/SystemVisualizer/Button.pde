enum ButtonState {
  Normal,
  Hover,
  Pressed
}

class Button {
  float x, y, w, h, radius;
  String label;
  
  color normalColor, hoverColor, pressedColor;
  
  private ButtonState state = ButtonState.Normal;
  private boolean isMousePressedOnButton = false;

  private Runnable onPressedAction, onReleasedAction;
  private boolean prevMousePressed = false;

  Button(float x, float y, float w, float h, float radius, String label) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
    this.radius = radius;
    this.label = label;
    
    // デフォルト色
    this.normalColor = color(190, 190, 190);
    this.hoverColor = color(220, 220, 220);
    this.pressedColor = color(150, 150, 150);
  }

  void setColors(color normal, color hover, color pressed) {
    this.normalColor = normal;
    this.hoverColor = hover;
    this.pressedColor = pressed;
  }

  void setOnPressedAction(Runnable action) {
    this.onPressedAction = action;
  }

  void setOnReleasedAction(Runnable action) {
    this.onReleasedAction = action;
  }

  boolean isHovered() {
    return mouseX >= x && mouseX <= x + w && mouseY >= y && mouseY <= y + h;
  }

  void update() {
    boolean _mousePressed = mousePressed;
    if (isHovered()) {
      // マウスがボタン上で押されている最中ならPressed状態を維持
      if (_mousePressed && isMousePressedOnButton) {
        state = ButtonState.Pressed;
      } else {
        state = ButtonState.Hover;
      }
    } else {
      state = ButtonState.Normal;
    }

    // マウスイベント処理
    if (!prevMousePressed && _mousePressed) {
      handleMousePressed();
    } else if (prevMousePressed && !_mousePressed) {
      handleMouseReleased();
    } else if (!_mousePressed) {
        isMousePressedOnButton = false;
    }

    prevMousePressed = _mousePressed;
  }

  // 描画処理
  void draw() {
    pushStyle();

    switch (state) {
      case Normal:
        fill(normalColor);
        break;
      case Hover:
        fill(hoverColor);
        break;
      case Pressed:
        fill(pressedColor);
        break;
    }
    rect(x, y, w, h, radius);

    // テキスト描画
    noStroke();
    fill(255);
    textAlign(CENTER, CENTER);
    textSize(16);
    text(label, x + w / 2, y + h / 2);

    popStyle();
  }

  void handleMousePressed() {
    if (isHovered()) {
      isMousePressedOnButton = true;
      state = ButtonState.Pressed;

      if (onPressedAction != null) {
        onPressedAction.run();
      }
    }
  }

  boolean handleMouseReleased() {
    boolean clicked = false;
    if (isHovered() && isMousePressedOnButton) {
      clicked = true;
      if (onReleasedAction != null) {
        onReleasedAction.run();
      }
    }
    isMousePressedOnButton = false;
    return clicked;
  }
}