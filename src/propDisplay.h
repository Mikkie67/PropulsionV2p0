#ifndef PROP_DISPLAY_H
#define PROP_DISPLAY_H

#include "ILI9488.h"

#define PIXEL_WIDTH  12
#define PIXEL_HEIGHT 18

typedef enum
{
    LEFT = 0,
    CENTER = 1,
    RIGHT = 2
} eJustify_t;

typedef struct 
{
    int32_t MotorTemp;
    int32_t ControllerTemp;
    uint32_t MotorRpm;
    uint32_t PhaseCurrent;
    bool ControllerCommsOk;
} sPropData_t;
typedef struct
{
    uint32_t Voltage;
    uint32_t Current;
    uint32_t SoC;
    int32_t Temp;
    bool BmsCommsOk;
} sBmsData_t;
typedef struct
{
    sPropData_t Port;
    sPropData_t Starboard;
    sBmsData_t ForwardBattery;
    sBmsData_t AftBattery;
    bool CoolantFan;
    bool AmbientFan;
    bool CoolantPump;
    int32_t AmbientTemp;
    int32_t CoolantTemp;
    uint32_t UpTime;
    char Ssid[40];
    char IpAddr[16];
    uint32_t RunTime;
    int32_t ShaftRpm;
} sDisplayData_t;

uint16_t WriteJustifiedBool(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, bool value, eJustify_t eJustify, bool newline);
uint16_t WriteJustifiedTemp(ILI9488* tft, uint16_t y_pos, uint16_t x_start, uint16_t x_stop, int32_t value, eJustify_t eJustify, bool newline);
void createFixedElements(ILI9488* tft);
void createDynamicElements(ILI9488* tft, sDisplayData_t sDisplayData);
void testLcd(ILI9488* tft);
unsigned long testFillScreen(ILI9488* tft);
unsigned long testText(ILI9488* tft);
unsigned long testLines(ILI9488* tft,uint16_t color);
unsigned long testFastLines(ILI9488* tft,uint16_t color1, uint16_t color2);
unsigned long testRects(ILI9488* tft,uint16_t color);
unsigned long testFilledRects(ILI9488* tft,uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(ILI9488* tft,uint8_t radius, uint16_t color);
unsigned long testCircles(ILI9488* tft,uint8_t radius, uint16_t color);
unsigned long testTriangles(ILI9488* tft);
unsigned long testFilledTriangles(ILI9488* tft);
unsigned long testRoundRects(ILI9488* tft);
unsigned long testFilledRoundRects(ILI9488* tft);



#endif
