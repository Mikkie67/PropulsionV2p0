#ifndef DISLAY_SSD1306
#define DISLAY_SSD1306

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
typedef struct display_ssd1306_data
{
  uint8_t sCurrentCan0State;
  uint8_t CanRxMsg;
  uint8_t CanTxMsg;
  uint16_t LifeSignal;
  int8_t Busvoltage;
  int8_t BusCurrent;
  int8_t PhaseCurrent;
  int16_t SpeedRpm;
  int8_t ControllerTemp;
  int8_t MotorTemp;
  uint8_t ThrottlePos;
}sDisplayData_t;

class display_ssd1306 {
 private:
  Adafruit_SSD1306 *display;
  TwoWire *i2c;

 public:
  display_ssd1306(){}
  ~display_ssd1306(){}
  void begin(int8_t sdaPin, int8_t sclPin);
  void updateDisplay(sDisplayData_t sDisplayData);
};

#endif  // DISLAY_SSD1306
