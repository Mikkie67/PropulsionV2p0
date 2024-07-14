#include "display_ssd1306.hpp"

void display_ssd1306::begin(int8_t sdaPin, int8_t sclPin) {
  // initialize the display
  i2c = new TwoWire(0);
  i2c->begin(sdaPin, sclPin);
  display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, i2c, -1);
  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  delay(100);
  display->setRotation(2);
  display->clearDisplay();
  display->display();
}
void display_ssd1306::updateDisplay(sDisplayData_t sDisplayData) {
  display->clearDisplay();
  display->setTextSize(1);
  display->setCursor(0, 0);
  display->setTextColor(SSD1306_WHITE);
  display->printf("KEROSHEBA MOTOR CNTRL\n");
  display->printf("CAN: %d Uptime: %d\n", sDisplayData.sCurrentCan0State,
                  millis() / 1000);
  display->printf("RX: %d    TX: %02d: %d\n", sDisplayData.CanRxMsg,
                  sDisplayData.LifeSignal, sDisplayData.CanTxMsg);
  // display->printf("123456789012345678901\n");
  display->printf("Bus V  Bus I  Phase I");
  display->printf("%2.2f %#6d %#7d\n", sDisplayData.Busvoltage,
                  sDisplayData.BusCurrent, sDisplayData.PhaseCurrent);
  display->printf("Speed RPM: %#5d\n", sDisplayData.SpeedRpm);
  display->printf("CT:%#2.1f MT:%#2.1f %#2d\n", sDisplayData.ControllerTemp,
                  sDisplayData.MotorTemp, sDisplayData.ThrottlePos);
  display->display();
}
