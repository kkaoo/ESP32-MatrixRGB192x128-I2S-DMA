#ifndef _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA
#define _ESP32_RGB_64_32_MATRIX_PANEL_I2S_DMA

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_heap_caps.h"
#include "esp32_i2s_parallel.h"

#include "Adafruit_GFX.h"


/*

    This is example code to driver a p3(2121)64*32 -style RGB LED display. These types of displays do not have memory and need to be refreshed
    continuously. The display has 2 RGB inputs, 4 inputs to select the active line, a pixel clock input, a latch enable input and an output-enable
    input. The display can be seen as 2 64x16 displays consisting of the upper half and the lower half of the display. Each half has a separate 
    RGB pixel input, the rest of the inputs are shared.

    Each display half can only show one line of RGB pixels at a time: to do this, the RGB data for the line is input by setting the RGB input pins
    to the desired value for the first pixel, giving the display a clock pulse, setting the RGB input pins to the desired value for the second pixel,
    giving a clock pulse, etc. Do this 64 times to clock in an entire row. The pixels will not be displayed yet: until the latch input is made high, 
    the display will still send out the previously clocked in line. Pulsing the latch input high will replace the displayed data with the data just 
    clocked in.

    The 4 line select inputs select where the currently active line is displayed: when provided with a binary number (0-15), the latched pixel data
    will immediately appear on this line. Note: While clocking in data for a line, the *previous* line is still displayed, and these lines should
    be set to the value to reflect the position the *previous* line is supposed to be on.

    Finally, the screen has an OE input, which is used to disable the LEDs when latching new data and changing the state of the line select inputs:
    doing so hides any artifacts that appear at this time. The OE line is also used to dim the display by only turning it on for a limited time every
    line.

    All in all, an image can be displayed by 'scanning' the display, say, 100 times per second. The slowness of the human eye hides the fact that
    only one line is showed at a time, and the display looks like every pixel is driven at the same time.

    Now, the RGB inputs for these types of displays are digital, meaning each red, green and blue subpixel can only be on or off. This leads to a
    color palette of 8 pixels, not enough to display nice pictures. To get around this, we use binary code modulation.

    Binary code modulation is somewhat like PWM, but easier to implement in our case. First, we define the time we would refresh the display without
    binary code modulation as the 'frame time'. For, say, a four-bit binary code modulation, the frame time is divided into 15 ticks of equal length.

    We also define 4 subframes (0 to 3), defining which LEDs are on and which LEDs are off during that subframe. (Subframes are the same as a 
    normal frame in non-binary-coded-modulation mode, but are showed faster.)  From our (non-monochrome) input image, we take the (8-bit: bit 7 
    to bit 0) RGB pixel values. If the pixel values have bit 7 set, we turn the corresponding LED on in subframe 3. If they have bit 6 set,
    we turn on the corresponding LED in subframe 2, if bit 5 is set subframe 1, if bit 4 is set in subframe 0.

    Now, in order to (on average within a frame) turn a LED on for the time specified in the pixel value in the input data, we need to weigh the
    subframes. We have 15 pixels: if we show subframe 3 for 8 of them, subframe 2 for 4 of them, subframe 1 for 2 of them and subframe 1 for 1 of
    them, this 'automatically' happens. (We also distribute the subframes evenly over the ticks, which reduces flicker.)

    In this code, we use the I2S peripheral in parallel mode to achieve this. Essentially, first we allocate memory for all subframes. This memory
    contains a sequence of all the signals (2xRGB, line select, latch enable, output enable) that need to be sent to the display for that subframe.
    Then we ask the I2S-parallel driver to set up a DMA chain so the subframes are sent out in a sequence that satisfies the requirement that
    subframe x has to be sent out for (2^x) ticks. Finally, we fill the subframes with image data.

    We use a frontbuffer/backbuffer technique here to make sure the display is refreshed in one go and drawing artifacts do not reach the display.
    In practice, for small displays this is not really necessarily.
    
*/



#define SERIAL_DEBUG_OUTPUT 1


// This could also be defined as matrix.color(255,0,0) but those defines
// are meant to work for Adafruit::GFX backends that are lacking color()
#define LED_BLACK		0

#define LED_RED_VERYLOW 	(3 <<  11)
#define LED_RED_LOW 		(7 <<  11)
#define LED_RED_MEDIUM 		(15 << 11)
#define LED_RED_HIGH 		(31 << 11)

#define LED_GREEN_VERYLOW	(1 <<  5)   
#define LED_GREEN_LOW 		(15 << 5)  
#define LED_GREEN_MEDIUM 	(31 << 5)  
#define LED_GREEN_HIGH 		(63 << 5)  

#define LED_BLUE_VERYLOW	3
#define LED_BLUE_LOW 		7
#define LED_BLUE_MEDIUM 	15
#define LED_BLUE_HIGH 		31

#define LED_ORANGE_VERYLOW	(LED_RED_VERYLOW + LED_GREEN_VERYLOW)
#define LED_ORANGE_LOW		(LED_RED_LOW     + LED_GREEN_LOW)
#define LED_ORANGE_MEDIUM	(LED_RED_MEDIUM  + LED_GREEN_MEDIUM)
#define LED_ORANGE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH)

#define LED_PURPLE_VERYLOW	(LED_RED_VERYLOW + LED_BLUE_VERYLOW)
#define LED_PURPLE_LOW		(LED_RED_LOW     + LED_BLUE_LOW)
#define LED_PURPLE_MEDIUM	(LED_RED_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_PURPLE_HIGH		(LED_RED_HIGH    + LED_BLUE_HIGH)

#define LED_CYAN_VERYLOW	(LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_CYAN_LOW		(LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_CYAN_MEDIUM		(LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_CYAN_HIGH		(LED_GREEN_HIGH    + LED_BLUE_HIGH)

#define LED_WHITE_VERYLOW	(LED_RED_VERYLOW + LED_GREEN_VERYLOW + LED_BLUE_VERYLOW)
#define LED_WHITE_LOW		(LED_RED_LOW     + LED_GREEN_LOW     + LED_BLUE_LOW)
#define LED_WHITE_MEDIUM	(LED_RED_MEDIUM  + LED_GREEN_MEDIUM  + LED_BLUE_MEDIUM)
#define LED_WHITE_HIGH		(LED_RED_HIGH    + LED_GREEN_HIGH    + LED_BLUE_HIGH)


/***************************************************************************************/
/* ESP32 Pin Definition. You can change this, but best if you keep it as is...         */

#define R1_PIN_DEFAULT  33
#define G1_PIN_DEFAULT  32
#define B1_PIN_DEFAULT  25
#define R2_PIN_DEFAULT  27
#define G2_PIN_DEFAULT  26
#define B2_PIN_DEFAULT  21

#define A_PIN_DEFAULT   13
#define B_PIN_DEFAULT   12
#define C_PIN_DEFAULT   2
#define D_PIN_DEFAULT   15
#define E_PIN_DEFAULT   14//-1 // Change to a valid pin if using a 64 pixel row panel.
          
#define LAT_PIN_DEFAULT 4
#define OE_PIN_DEFAULT  17

#define CLK_PIN_DEFAULT 13
/***************************************************************************************/
/* Don't change this stuff unless you know what you are doing */

// Panel Upper half RGB (numbering according to order in DMA gpio_bus configuration)
#define BIT_R1  (1<<0)   
#define BIT_G1  (1<<1)   
#define BIT_B1  (1<<2)   

// Panel Lower half RGB
#define BIT_R2  (1<<3)   
#define BIT_G2  (1<<4)   
#define BIT_B2  (1<<5)   

// Panel Control Signals
#define BIT_LAT (1<<6) 
#define BIT_OE  (1<<7)  

// Panel GPIO Pin Addresses (A, B, C, D etc..)
#define BIT_A (1<<8)    
#define BIT_B (1<<9)    
#define BIT_C (1<<10)   
#define BIT_D (1<<11)   
#define BIT_E (1<<12)   

// RGB Panel Constants / Calculated Values
#define COLOR_CHANNELS_PER_PIXEL  3 
#define PIXELS_PER_LATCH          ((MATRIX_WIDTH * MATRIX_HEIGHT) / MATRIX_HEIGHT)  //  = 64
#define COLOR_DEPTH_BITS          (COLOR_DEPTH/COLOR_CHANNELS_PER_PIXEL)            //  = 8 or 5
#define ROWS_PER_FRAME            (MATRIX_HEIGHT/MATRIX_ROWS_IN_PARALLEL)           //  = 32

/***************************************************************************************/
/* You really don't want to change this stuff                                          */

// #define CLKS_DURING_LATCH   0  // ADDX is output directly using GPIO
#define MATRIX_I2S_MODE I2S_PARALLEL_BITS_16
#define MATRIX_DATA_STORAGE_TYPE uint16_t

// #define ESP32_OE_OFF_CLKS_AFTER_LATCH     1

#define MATRIX_WIDTH_DATA_SIZE   ( MATRIX_WIDTH  + CLKS_DURING_LATCH ) * sizeof(MATRIX_DATA_STORAGE_TYPE)

#define BIT_OFFSET (6+8-COLOR_DEPTH_BITS)
/***************************************************************************************/            

// note: sizeof(data) must be multiple of 32 bits, as ESP32 DMA linked list buffer address pointer must be word-aligned.

typedef struct rgb_24 {
    rgb_24() : rgb_24(0,0,0) {}
    rgb_24(uint8_t r, uint8_t g, uint8_t b) {
        red = r; green = g; blue = b;
    }
    rgb_24& operator=(const rgb_24& col);

    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_24;


typedef enum {
    ONE_PIXEL = false,
    TWO_PIXELS = true
} num_pixels_t;



/***************************************************************************************/
/* HUB75 RGB pixel WIDTH and HEIGHT. 
 *
 * This library has only been tested with a 64 pixel (wide) and 32 (high) RGB panel. 
 * Theoretically, if you want to chain two of these horizontally to make a 128x32 panel
 * you can do so with the cable and then set the MATRIX_WIDTH to '128'.
 *
 * Also, if you use a 64x64 panel, then set the MATRIX_HEIGHT to '64', and it might work.
 *
 * All of this is memory permitting of course (dependant on your sketch etc.) ...
 *
 */ 
class RGB64x32MatrixPanel_I2S_DMA : public Adafruit_GFX {
  // ------- PUBLIC -------
  public:
    int MATRIX_WIDTH;                //384//256
    int MATRIX_HEIGHT;               //64//32
    int MATRIX_ROWS_IN_PARALLEL;     //2
    int COLOR_DEPTH;
    int ESP32_NUM_FRAME_BUFFERS;     //1//2 
    int CLKS_DURING_LATCH;
    int ESP32_I2S_CLOCK_SPEED;       //(26670000UL)
 
    RGB64x32MatrixPanel_I2S_DMA(int16_t width=64, int16_t height=32, bool _doubleBuffer = false) // Double buffer is disabled by default. Any change will display next active DMA buffer output (very quickly). NOTE: Not Implemented
      : Adafruit_GFX(width, height), doubleBuffer(_doubleBuffer)  {
      
        backbuf_id = 0;
        brightness = 16; // If you get ghosting... reduce brightness level. 60 seems to be the limit before ghosting on a 64 pixel wide physical panel for some panels
        min_refresh_rate = 200; // Probably best to leave as is unless you want to experiment. Framerate has an impact on brightness and also power draw - voltage ripple.        

        MATRIX_WIDTH = width;
        MATRIX_HEIGHT = height;
        MATRIX_ROWS_IN_PARALLEL = 2;
        COLOR_DEPTH = 24;
        ESP32_NUM_FRAME_BUFFERS = 1;//2;
        ESP32_I2S_CLOCK_SPEED = (26670000UL);
        CLKS_DURING_LATCH = 0;
        PANEL_Z_MODE = false;

        //如果PANEL太大，降低COLOR_DEPTH和关掉双BUFFER
        if(height>64 || width>192){
          ESP32_NUM_FRAME_BUFFERS = 1;
          COLOR_DEPTH = 15;
          min_refresh_rate = 60;

          //高度最大只支64，如果超64，需要将面板摆成 ‘Z’ 字形，并且需要将地址进行转换
          if(height > 64)
          {
            MATRIX_WIDTH = width*2;
            MATRIX_HEIGHT = height/2;
            PANEL_Z_MODE = true;
          }
        }
    }
    
    // Painfully propagate the DMA pin configuration, or use compiler defaults
    void begin(int dma_r1_pin = R1_PIN_DEFAULT , int dma_g1_pin = G1_PIN_DEFAULT, int dma_b1_pin = B1_PIN_DEFAULT , int dma_r2_pin = R2_PIN_DEFAULT , int dma_g2_pin = G2_PIN_DEFAULT , int dma_b2_pin = B2_PIN_DEFAULT , int dma_a_pin  = A_PIN_DEFAULT  , int dma_b_pin = B_PIN_DEFAULT  , int dma_c_pin = C_PIN_DEFAULT , int dma_d_pin = D_PIN_DEFAULT  , int dma_e_pin = E_PIN_DEFAULT , int dma_lat_pin = LAT_PIN_DEFAULT, int dma_oe_pin = OE_PIN_DEFAULT , int dma_clk_pin = CLK_PIN_DEFAULT)
    {
      Serial.printf("\ninitial: panel=%dx%d, COLOR_DEPTH=%d, num_frame_buffers=%d ---- ", MATRIX_WIDTH, MATRIX_HEIGHT,COLOR_DEPTH, ESP32_NUM_FRAME_BUFFERS);
      Serial.printf("Adafruit_GFX=%dx%d \n", WIDTH, HEIGHT);
        
     /* As DMA buffers are dynamically allocated, we must allocated in begin()
      * Ref: https://github.com/espressif/arduino-esp32/issues/831
      */
      allocateDMAbuffers(); 

	    // Flush the DMA buffers prior to configuring DMA - Avoid visual artefacts on boot.
      flushDMAbuffer();
      if(ESP32_NUM_FRAME_BUFFERS == 2){
        flipDMABuffer();    // flip to backbuffer 1
        flushDMAbuffer();
        flipDMABuffer();    // backbuffer 0
	    }
      // Setup the ESP32 DMA Engine. Sprite_TM built this stuff.
      configureDMA(dma_r1_pin, dma_g1_pin, dma_b1_pin, dma_r2_pin, dma_g2_pin, dma_b2_pin, dma_a_pin,  dma_b_pin, dma_c_pin, dma_d_pin, dma_e_pin, dma_lat_pin,  dma_oe_pin,   dma_clk_pin ); //DMA and I2S configuration and setup
	    showDMABuffer(); // show 0
    }
 
    // Draw pixels
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color);   // overwrite adafruit implementation
    virtual void fillScreen(uint16_t color);                        // overwrite adafruit implementation
    
    inline void clearScreen() { fillScreen(0); } 
    void drawPixelRGB565(int16_t x, int16_t y, uint16_t color);
    void drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b);
    void drawPixelRGB24(int16_t x, int16_t y, rgb_24 color);
    void drawIcon (int *ico, int16_t x, int16_t y, int16_t cols, int16_t rows);
    
    // TODO: Draw a frame! Oooh.
    // LCD Data buffer --> DMA Buffer
    void drawLCDData(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);
    
    // Color 444 is a 4 bit scale, so 0 to 15, color 565 takes a 0-255 bit value, so scale up by 255/15 (i.e. 17)!
    inline uint16_t color444(uint8_t r, uint8_t g, uint8_t b) { return color565(r*17,g*17,b*17); }

    // Converts RGB888 to RGB565, Pass 8-bit (each) R,G,B, get back 16-bit packed color
    inline uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }

    // Converts RGB333 to RGB565,  Promote 3/3/3 RGB to Adafruit_GFX 5/6/5 RRRrrGGGgggBBBbb
    inline uint16_t Color333(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0x7) << 13) | ((r & 0x6) << 10) | ((g & 0x7) << 8) | ((g & 0x7) << 5) | ((b & 0x7) << 2) | ((b & 0x6) >> 1);}


    inline void flipDMABuffer() 
	  {
      // Step 1. Bring backbuffer to the foreground (i.e. show it)
      //showDMABuffer();
      
      // Step 2. Copy foreground to backbuffer
      //matrixUpdateFrames[backbuf_id ^ 1] = matrixUpdateFrames[backbuf_id];	  // copy currently being displayed buffer to backbuffer		
      
      // Step 3. Change to this new buffer as the backbuffer
      //backbuf_id ^=1; // set this now to the  back_buffer to update (not displayed yet though)
      
      #if SERIAL_DEBUG_OUTPUT     
        Serial.printf("Set back buffer to: %d\n", backbuf_id);
      #endif		
    }
	
    inline void showDMABuffer(){ i2s_parallel_flip_to_buffer(&I2S1, backbuf_id); }

    // Change to set the brightness of the display, range of 1 to matrixWidth (i.e. 1 - 64)
    inline void setPanelBrightness(int _brightness){ brightness = _brightness; }

    inline void setMinRefreshRate(int rr){ min_refresh_rate = rr; }

   // ------- PRIVATE -------
  private:
  
    void allocateDMAbuffers() 
    {
        Serial.printf("\r\nAllocating DMA Refresh Buffer...\r\nTotal DMA Memory available: %d bytes total. Largest free block: %d bytes\r\n", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        // frameStruct* matrixUpdateFrames = (frameStruct *)heap_caps_malloc(sizeof(frameStruct) * ESP32_NUM_FRAME_BUFFERS, MALLOC_CAP_DMA);
        matrixUpdateFrames = new MATRIX_DATA_STORAGE_TYPE**[ROWS_PER_FRAME];
        for(int i=0; i<ROWS_PER_FRAME; i++){
          matrixUpdateFrames[i] = new MATRIX_DATA_STORAGE_TYPE*[COLOR_DEPTH_BITS];
        }

        for(int i=0; i<ROWS_PER_FRAME; i++){
          uint8_t* p = (uint8_t*)heap_caps_malloc(MATRIX_WIDTH_DATA_SIZE*COLOR_DEPTH_BITS, MALLOC_CAP_DMA);
          // Serial.printf("\r\n %x %x %d", p, p+1, sizeof(MATRIX_DATA_STORAGE_TYPE));
          for(int j=0; j<COLOR_DEPTH_BITS; j++){
            matrixUpdateFrames[i][j] = (MATRIX_DATA_STORAGE_TYPE*)p;
            p += MATRIX_WIDTH_DATA_SIZE;
            // Serial.printf("  %x", p);
          }
          // Serial.printf("\r\nAllocating DMA Refresh Buffer %d...\r\nTotal DMA Memory available: %d bytes total. Largest free block: %d bytes\r\n", i, heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        }
        Serial.printf("Overload, Total DMA Memory available: %d bytes total. Largest free block: %d bytes\r\n", heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));

        // Serial.printf("rowColorDepthStruct sizeof %d bytes. \r\n", sizeof(rowColorDepthStruct)*ESP32_NUM_FRAME_BUFFERS );
        Serial.printf("matrixUpdateFrames sizeof %d bytes. \r\n", MATRIX_WIDTH_DATA_SIZE*COLOR_DEPTH_BITS*ROWS_PER_FRAME );
    } // end initMatrixDMABuffer()
    
    void flushDMAbuffer()
    {
      Serial.printf("Flushing buffer %d\n", backbuf_id);
      // Need to wipe the contents of the matrix buffers or weird things happen.
      for (int y=0;y<MATRIX_HEIGHT; y++){
        for (int x=0;x<MATRIX_WIDTH; x++)
        {
          //Serial.printf("\r\nFlushing x, y coord %d, %d", x, y);
          updateMatrixDMABuffer( x, y, 0, 0, 0);
        }
      }
    }

    // Get everything setup. Refer to the .c file
    void configureDMA(int r1_pin, int  g1_pin, int  b1_pin, int  r2_pin, int  g2_pin, int  b2_pin, int  a_pin, int   b_pin, int  c_pin, int  d_pin, int  e_pin, int  lat_pin, int   oe_pin, int clk_pin); 
    
    inline void updatePixerDMABuffer(num_pixels_t num_pixels, int16_t x,int16_t y, uint8_t r1,uint8_t g1,uint8_t b1,uint8_t r2,uint8_t g2,uint8_t b2);

    // Update a specific pixel in the DMA buffer to a colour 
    void updateMatrixDMABuffer(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue);
   
    // Update the entire DMA buffer (aka. The RGB Panel) a certain colour (wipe the screen basically)
    void updateMatrixDMABuffer(uint8_t red, uint8_t green, uint8_t blue);
    	
    // Pixel data is organized from LSB to MSB sequentially by row, from row 0 to row matrixHeight/matrixRowsInParallel (two rows of pixels are refreshed in parallel)
    // rowColorDepthStruct *matrixUpdateFrames[ROWS_PER_FRAME];
    // frameStruct *matrixUpdateFrames;
    MATRIX_DATA_STORAGE_TYPE*** matrixUpdateFrames;
   
	  // Setup
	  bool dma_configuration_success;
    	
	  // Internal variables
    bool doubleBuffer; 	// Do we use double buffer mode? Your project code will have to manually flip between both.
    int  backbuf_id; 	  // If using double buffer, which one is NOT active (ie. being displayed) to write too?
	
    int  lsbMsbTransitionBit;
    int  refreshRate;     
    int  brightness;
    int  min_refresh_rate;
    int  PANEL_Z_MODE;

}; // end Class header

/***************************************************************************************/   
// https://stackoverflow.com/questions/5057021/why-are-c-inline-functions-in-the-header
/* 2. functions declared in the header must be marked inline because otherwise, every translation unit which includes the header will contain a definition of the function, and the linker will complain about multiple definitions (a violation of the One Definition Rule). The inline keyword suppresses this, allowing multiple translation units to contain (identical) definitions. */

inline void RGB64x32MatrixPanel_I2S_DMA::drawPixel(int16_t x, int16_t y, uint16_t color) // adafruit virtual void override
{
  drawPixelRGB565( x, y, color);
} 


/*  drawIcon draws a C style bitmap.  
//  Example 10x5px bitmap of a yellow sun 

  int half_sun [50] = {
      0x0000, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0xffe0, 0x0000, 0x0000, 0x0000,
      0x0000, 0xffe0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xffe0, 0x0000,
      0x0000, 0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000, 0x0000,
      0xffe0, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0xffe0,
      0x0000, 0x0000, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0xffe0, 0x0000, 0x0000,
  };
  
  RGB64x32MatrixPanel_I2S_DMA matrix;
  matrix.drawIcon (half_sun, 0,0,10,5);
*/
inline void RGB64x32MatrixPanel_I2S_DMA::drawIcon (int *ico, int16_t x, int16_t y, int16_t cols, int16_t rows) {
  int i, j;
  for (i = 0; i < rows; i++) {
    for (j = 0; j < cols; j++) {
      drawPixelRGB565 (x + j, y + i, ico[i * cols + j]);
    }
  }  
}

inline void RGB64x32MatrixPanel_I2S_DMA::fillScreen(uint16_t color)  // adafruit virtual void override
{
  uint8_t r = ((((color >> 11) & 0x1F) * 527) + 23) >> BIT_OFFSET;
  uint8_t g = ((((color >> 5) & 0x3F) * 259) + 33) >> BIT_OFFSET;
  uint8_t b = (((color & 0x1F) * 527) + 23) >> BIT_OFFSET;
  
  updateMatrixDMABuffer(r, g, b); // the RGB only (no pixel coordinate) version of 'updateMatrixDMABuffer'
} 

// For adafruit
inline void RGB64x32MatrixPanel_I2S_DMA::drawPixelRGB565(int16_t x, int16_t y, uint16_t color) 
{
  uint8_t r = ((((color >> 11) & 0x1F) * 527) + 23) >> BIT_OFFSET;
  uint8_t g = ((((color >> 5) & 0x3F) * 259) + 33) >> BIT_OFFSET;
  uint8_t b = (((color & 0x1F) * 527) + 23) >> BIT_OFFSET;
  
  updateMatrixDMABuffer( x, y, r, g, b);
}

inline void RGB64x32MatrixPanel_I2S_DMA::drawPixelRGB888(int16_t x, int16_t y, uint8_t r, uint8_t g,uint8_t b) 
{
  updateMatrixDMABuffer( x, y, r, g, b);
}

inline void RGB64x32MatrixPanel_I2S_DMA::drawPixelRGB24(int16_t x, int16_t y, rgb_24 color) 
{
  updateMatrixDMABuffer( x, y, color.red, color.green, color.blue);
}


#endif
