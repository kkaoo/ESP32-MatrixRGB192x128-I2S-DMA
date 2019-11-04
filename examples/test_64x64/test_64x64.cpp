
// By kkaoo
#include <SPI.h>
#include "ESP32-RGBMatrixPanel-I2S-DMA.h"

// Define matrix width and height.
#define mw 64
#define mh 64

RGB64x32MatrixPanel_I2S_DMA matrix(mw, mh);


uint32_t data_ready = 0;
void loop() {

}


void setup() {
    Serial.begin(115200);

    // matrix.ESP32_I2S_CLOCK_SPEED = 20000000UL;

////different definitions of IO
#if 0
	matrix.begin(25,27,26, 14,13,12, 22,21,17,32,33, 4,15,16);
#else
    //           R1,G1,B1, R2,G2,B2, A,B,C,D,E,     LAT,OEB,CLK
	// matrix.begin(33,25,32, 27,21,26, 13,12, 2,15,14, 4,17,16);
	matrix.begin(33,32,25, 27,26,21, 13,12, 2,15,14, 4,17,16);
    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);        
#endif

    matrix.setTextWrap(false);
    matrix.setPanelBrightness(16);

    // // Test full bright of all LEDs. If brightness is too high
    // // for your current limit (i.e. USB), decrease it.
    matrix.fillScreen(LED_WHITE_LOW);
    delay(2000);
    matrix.fillScreen(LED_BLACK);


    uint16_t x = 0, y = 0;

    int half_sun [50] = {
        0x0000, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0x0000,
        0x0000, 0xffe0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xffe0, 0x0000,
        0x0000, 0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000, 0x0000,
        0xffe0, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0xffe0,
        0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000,
    };

    matrix.drawIcon (half_sun, x,y,10,5);
    matrix.drawIcon (half_sun, x+51,y+15,10,5);

    matrix.drawCircle(30,35,8,LED_RED_MEDIUM);
    matrix.fillCircle(30,35,5,LED_PURPLE_LOW);
    matrix.fillCircle(23,53,5,LED_BLUE_LOW);
    matrix.fillCircle(37,53,5,LED_RED_LOW);

    const char* kkaoo= "abcde";
    for(x=0; x<5; x++){
        matrix.drawChar(9+6*x,6,kkaoo[x],LED_RED_VERYLOW, LED_GREEN_VERYLOW, 1);
    }

    const char* carson= "carson";
    for(x=0; x<6; x++){
        matrix.drawChar(9+6*x,15,carson[x],LED_CYAN_LOW, 0, 1);
    }
}

// vim:sts=4:sw=4