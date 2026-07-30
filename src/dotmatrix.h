#pragma once
#include <MD_Parola.h>
#include <MD_MAX72xx.h>

static const uint8_t NR_DOTMATRIX_UNITS = 4; // number of dotmatrix units (square 8x8 pixel modules) in the display

class DotMatrix : public MD_Parola {
  public:
    DotMatrix(uint8_t MOSI_gpio, uint8_t SCK_gpio, uint8_t CS_gpio) : MD_Parola(MD_MAX72XX::FC16_HW, MOSI_gpio, SCK_gpio, CS_gpio, NR_DOTMATRIX_UNITS) {
        begin();
        setIntensity(2); // initial dotmatrix intensity [0,15]

        // Having the 5-pole connector on the left, the dotmatrix display is showing text upside down,
        // so we need to flip the display content vertically and horizontally
        setZoneEffect(0, true, PA_FLIP_UD);
        setZoneEffect(0, true, PA_FLIP_LR);
    }

    //--------------------------------------------------
    // Show a steady message on dotmatrix display
    //--------------------------------------------------
    void showMessage(const char *msg) {
        displayClear();
        displayText(msg, PA_CENTER, 0, 0, PA_PRINT, PA_NO_EFFECT);
        displayReset();
        displayAnimate(); // steady text needs to be animated too
    }
};

extern DotMatrix *dotMatrixPtr;
