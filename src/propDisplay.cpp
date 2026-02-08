#include "propDisplay.h"

uint16_t WriteJustifiedString(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, char* string, eJustify_t eJustify, bool newline) 
{
    uint16_t x_pos=0;
    switch (eJustify)
    {
    case CENTER:
        x_pos = x_start + (((x_stop-x_start)/2) - (strnlen(string,80)*PIXEL_WIDTH)/2); break;
    case LEFT:
        x_pos = x_start; break;
    case RIGHT:
        x_pos = (x_stop - (strnlen(string,80)*PIXEL_WIDTH)); break;
    default:
        break;
    }
    tft->setCursor(x_pos,y_pos);
    tft->println(string);
    if (newline)
    {
        y_pos += PIXEL_HEIGHT;
    }
    return y_pos;
}
uint16_t WriteJustifiedValue(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, uint32_t value, eJustify_t eJustify, bool newline) 
{
    uint16_t x_pos=0;
    char str[20] = {0};
    sprintf(str,"%d",value);
    switch (eJustify)
    {
    case CENTER:
        x_pos = x_start + (((x_stop-x_start)/2) - (strnlen(str,80)*PIXEL_WIDTH)/2); break;
    case LEFT:
        x_pos = x_start; break;
    case RIGHT:
        x_pos = (x_stop - (strnlen(str,80)*PIXEL_WIDTH)); break;
    default:
        break;
    }
    tft->setCursor(x_pos,y_pos);
    tft->println(str);
    if (newline)
    {
        y_pos += PIXEL_HEIGHT;
    }
    return y_pos;
}

uint16_t WriteJustifiedBool(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, bool value, eJustify_t eJustify, bool newline) 
{
    uint16_t x_pos=0;
    char str[20] = {0};
    sprintf(str,"%s", value ? " ON" : "OFF");
    switch (eJustify)
    {
    case CENTER:
        x_pos = x_start + (((x_stop-x_start)/2) - (strnlen(str,80)*PIXEL_WIDTH)/2); break;
    case LEFT:
        x_pos = x_start; break;
    case RIGHT:
        x_pos = (x_stop - (strnlen(str,80)*PIXEL_WIDTH)); break;
    default:
        break;
    }
    tft->setCursor(x_pos,y_pos);
    tft->println(str);
    if (newline)
    {
        y_pos += PIXEL_HEIGHT;
    }
    return y_pos;
}

uint16_t WriteJustifiedTemp(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, int32_t value, eJustify_t eJustify, bool newline) 
{
    uint16_t x_pos=0;
    char str[20] = {0};
    if (value < 0) {
        sprintf(str,"--C");
    } else {
        sprintf(str,"%dC", value);
    }
    switch (eJustify)
    {
    case CENTER:
        x_pos = x_start + (((x_stop-x_start)/2) - (strnlen(str,80)*PIXEL_WIDTH)/2); break;
    case LEFT:
        x_pos = x_start; break;
    case RIGHT:
        x_pos = (x_stop - (strnlen(str,80)*PIXEL_WIDTH)); break;
    default:
        break;
    }
    tft->setCursor(x_pos,y_pos);
    tft->println(str);
    if (newline)
    {
        y_pos += PIXEL_HEIGHT;
    }
    return y_pos;
}

void createFixedElements(ILI9488* tft) {
  char string[80] = {};
  uint16_t y_pos = 0;
  tft->setTextColor(ILI9488_WHITE);
  tft->setTextSize(2);
  y_pos = WriteJustifiedString(tft,1,0  ,240,"PORT MOTOR",       eJustify_t::CENTER,false);
  y_pos = WriteJustifiedString(tft,0,240,480,"STARBOARD MOTOR",  eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"MOTOR TEMPERATURE", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"CONTROLLER TEMPERATURE", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"MOTOR RPM", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"PHASE CURRENT", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"COMMUNICATIONS", eJustify_t::CENTER,true);
  
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"FORWARD BATTERY",       eJustify_t::CENTER,false);
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"AFT BATTERY",  eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"VOLTAGE", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"CURRENT", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"STATE OF CHARGE", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"TEMPERATURE", eJustify_t::CENTER,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,480,"COMMUNICATIONS", eJustify_t::CENTER,true);
  uint16_t y_stored_pos = y_pos;
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"COOLANT FAN",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"AMBIENT FAN",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"COOLANT PUMP",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"AMBIENT TEMP",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"COOLANT TEMP",       eJustify_t::LEFT,true);
  y_pos = y_stored_pos;
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"UP TIME",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"SSID",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"IP ADDR",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"RUNTIME",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,240,480,"SHAFT RPM",       eJustify_t::LEFT,true);
  // Draw some boxes
  tft->drawRect(0  ,0  ,480,106,ILI9488_YELLOW);
  tft->drawRect(0  ,107,480,106,ILI9488_YELLOW);
  tft->drawRect(0  ,214,240,106,ILI9488_YELLOW);
  tft->drawRect(240,214,240,106,ILI9488_YELLOW);

  tft->drawFastHLine(0,17,480,ILI9488_YELLOW);
  tft->drawFastHLine(0,124,480,ILI9488_YELLOW);
}
void createDynamicElements(ILI9488* tft, sDisplayData_t sDisplayData) 
{
  char string[80] = {};
  uint16_t y_pos = 20;
  tft->setTextColor(ILI9488_GREEN,0x0000);
  tft->setTextSize(2);
  y_pos = WriteJustifiedTemp(tft,y_pos,30  ,480,sDisplayData.Port.MotorTemp, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedTemp(tft,y_pos,0  ,440,sDisplayData.Starboard.MotorTemp, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedTemp(tft,y_pos,30  ,480,sDisplayData.Port.ControllerTemp, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedTemp(tft,y_pos,0  ,440,sDisplayData.Starboard.ControllerTemp, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedValue(tft,y_pos,30  ,480,sDisplayData.Port.MotorRpm, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,440,sDisplayData.Starboard.MotorRpm, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedValue(tft,y_pos,30  ,480,sDisplayData.Port.PhaseCurrent, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,440,sDisplayData.Starboard.PhaseCurrent, eJustify_t::RIGHT,true);
  //y_pos = WriteJustifiedValue(tft,y_pos,0  ,480,sDisplayData.Port.ControllerTemp, eJustify_t::LEFT,false);
  //y_pos = WriteJustifiedValue(tft,y_pos,0  ,480,sDisplayData.Starboard.ControllerTemp, eJustify_t::RIGHT,true);
  y_pos = 127;
  y_pos = WriteJustifiedValue(tft,y_pos,30  ,480,sDisplayData.ForwardBattery.Voltage, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,440,sDisplayData.AftBattery.Voltage, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedValue(tft,y_pos,30  ,480,sDisplayData.ForwardBattery.Current, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,440,sDisplayData.AftBattery.Current, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedValue(tft,y_pos,30  ,480,sDisplayData.ForwardBattery.SoC, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,440,sDisplayData.AftBattery.SoC, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedTemp(tft,y_pos,30  ,480,sDisplayData.ForwardBattery.Temp, eJustify_t::LEFT,false);
  y_pos = WriteJustifiedTemp(tft,y_pos,0  ,440,sDisplayData.AftBattery.Temp, eJustify_t::RIGHT,true);
  //y_pos = WriteJustifiedValue(tft,y_pos,0  ,480,sDisplayData.Port.ControllerTemp, eJustify_t::LEFT,false);
  //y_pos = WriteJustifiedValue(tft,y_pos,0  ,480,sDisplayData.Starboard.ControllerTemp, eJustify_t::RIGHT,true);
  y_pos = 215;
  y_pos = WriteJustifiedBool(tft,y_pos,0  ,200,sDisplayData.CoolantFan, eJustify_t::RIGHT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,470,sDisplayData.UpTime, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedBool(tft,y_pos,0  ,200,sDisplayData.AmbientFan, eJustify_t::RIGHT,false);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,470,sDisplayData.Ssid, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedBool(tft,y_pos,0  ,200,sDisplayData.CoolantPump, eJustify_t::RIGHT,false);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,470,sDisplayData.IpAddr, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedTemp(tft,y_pos,0  ,200,sDisplayData.AmbientTemp, eJustify_t::RIGHT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,470,sDisplayData.RunTime, eJustify_t::RIGHT,true);
  y_pos = WriteJustifiedTemp(tft,y_pos,0  ,200,sDisplayData.CoolantTemp, eJustify_t::RIGHT,false);
  y_pos = WriteJustifiedValue(tft,y_pos,0  ,470,sDisplayData.ShaftRpm, eJustify_t::RIGHT,true);
}
unsigned long testFillScreen(ILI9488* tft) {
  unsigned long start = micros();
  tft->fillScreen(ILI9488_BLACK);
  tft->fillScreen(ILI9488_RED);
  tft->fillScreen(ILI9488_GREEN);
  tft->fillScreen(ILI9488_BLUE);
  tft->fillScreen(ILI9488_BLACK);
  return micros() - start;
}

unsigned long testText(ILI9488* tft) {
  tft->fillScreen(ILI9488_BLACK);
  unsigned long start = micros();
  tft->setCursor(0, 0);
  tft->setTextColor(ILI9488_WHITE);
  tft->setTextSize(1);
  tft->println("Hello World!");
  tft->setTextColor(ILI9488_YELLOW);
  tft->setTextSize(2);
  tft->println(1234.56);
  tft->setTextColor(ILI9488_RED);
  tft->setTextSize(3);
  tft->println(0xDEADBEEF, HEX);
  tft->println();
  tft->setTextColor(ILI9488_GREEN);
  tft->setTextSize(5);
  tft->println("Groop");
  tft->setTextSize(2);
  tft->println("I implore thee,");
  tft->setTextSize(1);
  tft->println("my foonting turlingdromes.");
  tft->println("And hooptiously drangle me");
  tft->println("with crinkly bindlewurdles,");
  tft->println("Or I will rend thee");
  tft->println("in the gobberwarts");
  tft->println("with my blurglecruncheon,");
  tft->println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(ILI9488* tft,uint16_t color) {
  unsigned long start, t;
  int x1, y1, x2, y2, w = tft->width(), h = tft->height();

  tft->fillScreen(ILI9488_BLACK);

  x1 = y1 = 0;
  y2 = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  t = micros() - start;  // fillScreen doesn't count against timing

  tft->fillScreen(ILI9488_BLACK);

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  t += micros() - start;

  tft->fillScreen(ILI9488_BLACK);

  x1 = 0;
  y1 = h - 1;
  y2 = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  t += micros() - start;

  tft->fillScreen(ILI9488_BLACK);

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft->drawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft->drawLine(x1, y1, x2, y2, color);

  return micros() - start;
}

unsigned long testFastLines(ILI9488* tft,uint16_t color1, uint16_t color2) {
  unsigned long start;
  int x, y, w = tft->width(), h = tft->height();

  tft->fillScreen(ILI9488_BLACK);
  start = micros();
  for (y = 0; y < h; y += 5) tft->drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5) tft->drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(ILI9488* tft,uint16_t color) {
  unsigned long start;
  int n, i, i2, cx = tft->width() / 2, cy = tft->height() / 2;

  tft->fillScreen(ILI9488_BLACK);
  n = min(tft->width(), tft->height());
  start = micros();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    tft->drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(ILI9488* tft,uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int n, i, i2, cx = tft->width() / 2 - 1, cy = tft->height() / 2 - 1;

  tft->fillScreen(ILI9488_BLACK);
  n = min(tft->width(), tft->height());
  for (i = n; i > 0; i -= 6) {
    i2 = i / 2;
    start = micros();
    tft->fillRect(cx - i2, cy - i2, i, i, color1);
    t += micros() - start;
    // Outlines are not included in timing results
    tft->drawRect(cx - i2, cy - i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(ILI9488* tft,uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft->width(), h = tft->height(), r2 = radius * 2;

  tft->fillScreen(ILI9488_BLACK);
  start = micros();
  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      tft->fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(ILI9488* tft,uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, r2 = radius * 2, w = tft->width() + radius,
            h = tft->height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      tft->drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles(ILI9488* tft) {
  unsigned long start;
  int n, i, cx = tft->width() / 2 - 1, cy = tft->height() / 2 - 1;

  tft->fillScreen(ILI9488_BLACK);
  n = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    tft->drawTriangle(cx, cy - i,      // peak
                     cx - i, cy + i,  // bottom left
                     cx + i, cy + i,  // bottom right
                     tft->color565(0, 0, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles(ILI9488* tft) {
  unsigned long start, t = 0;
  int i, cx = tft->width() / 2 - 1, cy = tft->height() / 2 - 1;

  tft->fillScreen(ILI9488_BLACK);
  start = micros();
  for (i = min(cx, cy); i > 10; i -= 5) {
    start = micros();
    tft->fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft->color565(0, i, i));
    t += micros() - start;
    tft->drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft->color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects(ILI9488* tft) {
  unsigned long start;
  int w, i, i2, cx = tft->width() / 2 - 1, cy = tft->height() / 2 - 1;

  tft->fillScreen(ILI9488_BLACK);
  w = min(tft->width(), tft->height());
  start = micros();
  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    tft->drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft->color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects(ILI9488* tft) {
  unsigned long start;
  int i, i2, cx = tft->width() / 2 - 1, cy = tft->height() / 2 - 1;

  tft->fillScreen(ILI9488_BLACK);
  start = micros();
  for (i = min(tft->width(), tft->height()); i > 20; i -= 6) {
    i2 = i / 2;
    tft->fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft->color565(0, i, 0));
  }

  return micros() - start;
}
void testLcd(ILI9488* tft) {
  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft->readcommand8(ILI9488_RDMODE);
  Serial.print("Display Power Mode: 0x");
  Serial.println(x, HEX);
  x = tft->readcommand8(ILI9488_RDMADCTL);
  Serial.print("MADCTL Mode: 0x");
  Serial.println(x, HEX);
  x = tft->readcommand8(ILI9488_RDPIXFMT);
  Serial.print("Pixel Format: 0x");
  Serial.println(x, HEX);
  x = tft->readcommand8(ILI9488_RDIMGFMT);
  Serial.print("Image Format: 0x");
  Serial.println(x, HEX);
  x = tft->readcommand8(ILI9488_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x");
  Serial.println(x, HEX);

  Serial.println(F("Benchmark                Time (microseconds)"));

  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen( tft));
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText( tft));
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines( tft,ILI9488_CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines( tft,ILI9488_RED, ILI9488_BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects( tft,ILI9488_GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects( tft,ILI9488_YELLOW, ILI9488_MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles( tft,10, ILI9488_MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles( tft,10, ILI9488_WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles( tft));
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles( tft));
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects( tft));
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects( tft));
  delay(500);

  Serial.println(F("Done!"));
}