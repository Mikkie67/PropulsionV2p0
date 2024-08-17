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


void createFixedElements(ILI9488* tft) {
  char string[80] = {};
  uint16_t y_pos = 0;
  // draw some temp lines
  //tft->drawFastVLine(240,0,320,ILI9488_YELLOW);
  //tft->drawFastVLine(360,0,320,ILI9488_YELLOW);
  //tft->drawFastVLine(120,0,320,ILI9488_YELLOW);
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
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"INLET TEMP",       eJustify_t::LEFT,true);
  y_pos = WriteJustifiedString(tft,y_pos,0  ,240,"OUTLET TEMP",       eJustify_t::LEFT,true);
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
