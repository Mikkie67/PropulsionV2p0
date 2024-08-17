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

void createFixedElements(ILI9488* tft);
#endif
