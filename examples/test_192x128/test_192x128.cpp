
// By kkaoo
#include <SPI.h>
#include "ESP32-RGBMatrixPanel-I2S-DMA.h"

// Define matrix width and height.
#define mw 192
#define mh 128

RGB64x32MatrixPanel_I2S_DMA matrix(mw, mh);


uint32_t data_ready = 0;
void loop() {

}


void setup() {
    Serial.begin(115200);

    //matrix.ESP32_I2S_CLOCK_SPEED = 20000000UL;

    //different definitions of IO
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
    matrix.drawIcon (half_sun, x+5,y+90,10,5);
    matrix.drawIcon (half_sun, x+51+128,y+15,10,5);
    matrix.drawIcon (half_sun, x+51+128,y+15+90,10,5);



    const char kkaoo[]= "abcdefghijklmn";
    for(x=0; x<sizeof(kkaoo)-1; x++){
        matrix.drawChar(9+6*x*2,6,kkaoo[x],LED_RED_VERYLOW, LED_GREEN_VERYLOW, 2);
    }

    const char carson[]= "hi, i am carson";
    for(x=0; x<sizeof(carson)-1; x++){
        matrix.drawChar(9+6*x*2,15*2,carson[x],LED_CYAN_LOW, 0, 2);
    }

    matrix.drawCircle(30*2,35*2,8*2,LED_RED_MEDIUM);
    matrix.fillCircle(30*2,35*2,5*2,LED_PURPLE_LOW);

    matrix.fillCircle(23*2,53*2,5*2,LED_BLUE_LOW);
    matrix.fillCircle(37*2,53*2,5*2,LED_RED_LOW);

    matrix.drawCircle(160,96,8*2,LED_RED_MEDIUM|LED_GREEN_MEDIUM);
    matrix.fillCircle(160,96,6*2,LED_RED_MEDIUM);
}

// vim:sts=4:sw=4