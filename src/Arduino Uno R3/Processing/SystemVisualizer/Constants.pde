/* Audio Constants */
class AudioConstants {
  static final float DEFAULT_GAIN = -10.0f;
}

/* Event Constants */
class EventConstants {
  static final int EVENT_DONG = 1;
  static final int EVENT_KA = 2;
}

/* Note Constants */
class NoteConstants {
  static final float NOTE_RADIUS = 15.0f;
  static final float TRAVEL_TIME_MS = 1000.0f;
}

enum NoteId
{
    Dong,
    Ka
}

/* Key Constants */
class KeyConstants {
  static final int KEY_DONG = KeyEvent.VK_J;
  static final int KEY_KA = KeyEvent.VK_K;
  static final int KEY_F3 = 99;
}

/* Serial Constants */
class SerialConstants {
  static final String PORT_NAME = "COM7";
  static final int BAUD_RATE = 115200;
  static final int BYTE_MASK = 0xFF;
  static final int SHIFT_24 = 24;
  static final int SHIFT_16 = 16;
  static final int SHIFT_8 = 8;
  static final String LOG_FORMAT_EVENT = "%02d:%02d:%02d > %s";
  static final int BYTE_FLOAT = 4;
}

/* Data Constants */
class DataConstants {
  static final String DATA_YPR = "YPR";
  static final String DATA_QUAT = "QUAT";
  static final String DATA_LINERACCEL = "LINERACCEL";
  static final String DATA_BETA = "BETA";
  static final String DATA_DT = "DT";
}

/* Single Command Constants */
class SingleCommandConstants {
  static final String INPUT = "Input";
  static final String STATE_IDLE = "Idle";
  static final String STATE_LOW_SWING = "LowSwing";
  static final String STATE_STOPPING = "Stopping";
}



/* State Colors */
class StateColors {
  static final int IDLE_R = 100;
  static final int IDLE_G = 255;
  static final int IDLE_B = 0;

  static final int STOPPING_R = 255;
  static final int STOPPING_G = 0;
  static final int STOPPING_B = 0;

  static final int LOW_SWING_R = 255;
  static final int LOW_SWING_G = 0;
  static final int LOW_SWING_B = 255;

  static final int UPWORD_SWING_R = 0; // 仮の色
  static final int UPWORD_SWING_G = 0; // 仮の色
  static final int UPWORD_SWING_B = 255; // 仮の色
}

/* UI Constants */
class UIConstants {
  // Window
  static final int WINDOW_WIDTH = 800;
  static final int WINDOW_HEIGHT = 600;

  // Text
  static final int TEXT_SIZE_LARGE = 24;
  static final int TEXT_SIZE_MEDIUM = 18;
  static final int TEXT_MARGIN_X = 10;
  static final int TEXT_MARGIN_Y_STATE = 30;
  static final int TEXT_MARGIN_Y_SCALE_FACTOR = 90;
  static final int TEXT_MARGIN_Y_GAIN = 30;

  // Scale Factor Slider
  static final int SCALE_SLIDER_X = 10;
  static final int SCALE_SLIDER_Y = 60;
  static final int SCALE_SLIDER_WIDTH = 100;
  static final int SCALE_SLIDER_HEIGHT = 10;
  static final int SCALE_SLIDER_KNOB_WIDTH = 10;
  static final int SCALE_SLIDER_KNOB_HEIGHT = 20;
  static final int SCALE_SLIDER_KNOB_OFFSET_Y = 5;
  static final int SCALE_FACTOR_MIN = 0;
  static final int SCALE_FACTOR_MAX = 1000;

  // Sound Toggle Button
  static final int SOUND_BUTTON_X = 10;
  static final int SOUND_BUTTON_Y = 120;
  static final int BUTTON_WIDTH = 150;
  static final int BUTTON_HEIGHT = 30;
  static final int BUTTON_CORNER_RADIUS = 5;

  // Key Press Toggle Button
  static final int KEY_BUTTON_X = 10;
  static final int KEY_BUTTON_Y = 160;

  // Madgwick Gain Slider
  static final int GAIN_SLIDER_X = 10;
  static final int GAIN_SLIDER_Y = 200;
  static final int GAIN_SLIDER_WIDTH = 100;
  static final int GAIN_SLIDER_HEIGHT = 10;
  static final int GAIN_SLIDER_KNOB_WIDTH = 10;
  static final int GAIN_SLIDER_KNOB_HEIGHT = 20;
  static final int GAIN_SLIDER_KNOB_OFFSET_Y = 5;
  static final float GAIN_MIN = 0.0f;
  static final float GAIN_MAX = 1.0f;
  static final float GAIN_ROUND_FACTOR = 100.0f;

  // SETGAIN Button
  static final int SET_GAIN_BUTTON_X = 10;
  static final int SET_GAIN_BUTTON_Y = 250;
}