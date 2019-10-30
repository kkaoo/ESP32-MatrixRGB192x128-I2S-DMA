// Adafruit_NeoMatrix example for single NeoPixel Shield.
// By Marc MERLIN <marc_soft@merlins.org>
// Contains code (c) Adafruit, license BSD

#include "ESP32-RGBMatrixPanel-I2S-DMA.h"
RGB64x32MatrixPanel_I2S_DMA matrix;
//#include "smileytongue24.h"

#include <SPI.h>
#include "driver/spi_slave.h"

// Panel Matrix doesn't fully work like Neomatrix (which I wrote this 
// demo for), so map a few calls to be compatible. The rest comes from
// Adafruit::GFX and works the same on both backends.
// #define setBrightness(x) fillScreen(0) // no-op, no brightness on this board
#define clear() fillScreen(0)
#define show() drawPixel(0, 0, 0);     // no show method in this GFX implementation
#define Color(x,y,z) color444(x/16,y/16,z/16)

// Define matrix width and height.
#define mw 192
#define mh 128

#define BRIGHTNESS 8

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


// SPI 通信周波数
// static const uint32_t SPI_CLK_HZ = 4500000;

// 通信サイズ
static const uint32_t TRANS_SIZE = 4096;

// 通信バッファ
uint8_t* spi_slave_tx_buf;
uint8_t* spi_slave_rx_buf;
uint8_t* spi_slave_buf;
uint8_t* spi_slave_buf2;

// SPI スレーブの設定
spi_slave_transaction_t spi_slave_trans;
spi_slave_interface_config_t spi_slave_cfg;
spi_bus_config_t spi_slave_bus;

// HSPI の端子設定
// static const uint8_t SPI_SLAVE_CS = 15;
// static const uint8_t SPI_SLAVE_CLK = 14;
// static const uint8_t SPI_SLAVE_MOSI = 13;
// static const uint8_t SPI_SLAVE_MISO = 12;
static const uint8_t SPI_SLAVE_CS = 5;
static const uint8_t SPI_SLAVE_CLK = 18;
static const uint8_t SPI_SLAVE_MOSI = 23;
static const uint8_t SPI_SLAVE_MISO = 19;

// デバッグ用のダンプ関数
void dump_buf(const char* title, uint8_t* buf, uint32_t start, uint32_t len)
{
    if (len == 1) {
        Serial.printf("%s [%d]: ", title, start);
    } else {
        Serial.printf("%s [%d-%d]: ", title, start, start + len - 1);
    }
    for (uint32_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[start + i]);
    }
    Serial.println();
}

void dump_buf(const char* title, uint16_t* buf, uint32_t start, uint32_t len)
{
    if (len == 1) {
        Serial.printf("%s [%d]: ", title, start);
    } else {
        Serial.printf("%s [%d-%d]: ", title, start, start + len - 1);
    }
    for (uint32_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[start + i]);
    }
    Serial.println();
}

// SPI 通信に使用するバッファの初期化
void spi_buf_init()
{
    // spi_slave_tx_buf = (uint8_t*)heap_caps_malloc(TRANS_SIZE, MALLOC_CAP_DMA);
    spi_slave_rx_buf = (uint8_t*)heap_caps_malloc(TRANS_SIZE, MALLOC_CAP_DMA);

    spi_slave_buf = (uint8_t*)heap_caps_malloc(MATRIX_HEIGHT*MATRIX_WIDTH*2, MALLOC_CAP_DEFAULT);
    spi_slave_buf2 = (uint8_t*)heap_caps_malloc(MATRIX_HEIGHT*MATRIX_WIDTH*2, MALLOC_CAP_DEFAULT);

    // memset(spi_slave_tx_buf, 0, TRANS_SIZE);
    memset(spi_slave_rx_buf, 0, TRANS_SIZE);
    memset(spi_slave_buf, 0, MATRIX_HEIGHT*MATRIX_WIDTH*2);
    memset(spi_slave_buf2, 0, MATRIX_HEIGHT*MATRIX_WIDTH*2);
}

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void spi_slave_setup(spi_slave_transaction_t *trans) 
{
	// Serial.println("SPI Slave 通信开始");
}

static uint8_t spi_buf_offset = 0;
// スレーブの通信完了後に呼ばれるコールバック
void spi_slave_tans_done(spi_slave_transaction_t* trans)
{
    // Serial.println("SPI Slave 通信完了 ");
	uint32_t len = trans->trans_len/8;

	if(len > 0)
	{
		if(len == 4096)
		{
			// Serial.printf("%d : %d\n", spi_buf_offset, trans->trans_len);
			memcpy(spi_slave_buf + 4096*spi_buf_offset, spi_slave_rx_buf, 4096);
			spi_buf_offset++;
		}
		else
		{
			// Serial.printf("[%d]\n",trans->trans_len);
			spi_buf_offset = 0;
		}
	}
}


// スレーブとして動作させる HSPI の初期化
void spi_slave_init()
{
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLUP_ONLY);

    spi_slave_trans.length = 8 * TRANS_SIZE;
    spi_slave_trans.rx_buffer = spi_slave_rx_buf;
    spi_slave_trans.tx_buffer = NULL;

    spi_slave_cfg.spics_io_num = SPI_SLAVE_CS;
    spi_slave_cfg.flags = 0;//SPI_SLAVE_BIT_LSBFIRST;
    // spi_slave_cfg.flags = SPI_SLAVE_BIT_LSBFIRST;
    spi_slave_cfg.queue_size = 1;
    spi_slave_cfg.mode = SPI_MODE0;//SPI_MODE3;
    spi_slave_cfg.post_setup_cb = spi_slave_setup;
    spi_slave_cfg.post_trans_cb = spi_slave_tans_done;

    spi_slave_bus.sclk_io_num = SPI_SLAVE_CLK;
    spi_slave_bus.mosi_io_num = SPI_SLAVE_MOSI;
    spi_slave_bus.miso_io_num = SPI_SLAVE_MISO;
    spi_slave_bus.max_transfer_sz = TRANS_SIZE;

    ESP_ERROR_CHECK(
        spi_slave_initialize(VSPI_HOST, &spi_slave_bus, &spi_slave_cfg, 2) // DMA 2ch
	); 
}

void spi_init()
{
    spi_buf_init();
    spi_slave_init();
}


void color_test(uint16_t i, uint16_t offset, uint16_t color)
{
	// Serial.printf("color [%d:%d]: %x [%d] \n", i, offset, color, color);
	matrix.drawPixel( i*2, 0+offset, color);
	matrix.drawPixel( i*2, 1+offset, color);
	matrix.drawPixel( i*2, 2+offset, color);
	matrix.drawPixel( i*2, 3+offset, color);
	matrix.drawPixel( i*2, 4+offset, color);
	matrix.drawPixel( i*2, 5+offset, color);
	matrix.drawPixel( i*2, 6+offset, color);
	matrix.drawPixel( i*2, 7+offset, color);

	matrix.drawPixel( i*2+1, 0+offset, color);
	matrix.drawPixel( i*2+1, 1+offset, color);
	matrix.drawPixel( i*2+1, 2+offset, color);
	matrix.drawPixel( i*2+1, 3+offset, color);
	matrix.drawPixel( i*2+1, 4+offset, color);
	matrix.drawPixel( i*2+1, 5+offset, color);
	matrix.drawPixel( i*2+1, 6+offset, color);
	matrix.drawPixel( i*2+1, 7+offset, color);
}


uint32_t data_ready = 0;
void loop() {
    // スレーブの送信準備
    ESP_ERROR_CHECK(
        spi_slave_queue_trans(VSPI_HOST, &spi_slave_trans, portMAX_DELAY)
	);

	if(spi_buf_offset >= MATRIX_WIDTH*MATRIX_HEIGHT*2/4096)
	{
		spi_buf_offset = 0;
		// Serial.print(0);
		memcpy(spi_slave_buf2, spi_slave_buf, MATRIX_HEIGHT*MATRIX_WIDTH*2);			//0.55ms
		data_ready = 1;
		// Serial.print(0xff);
	}
}


void taskOne( void * parameter )
{
    while(1)
	{
		if(data_ready)
		{
			data_ready = 0;
			matrix.drawLCDData(0, 0, (uint16_t*)spi_slave_buf2, 192, 128);			//37ms
		}
		delay(1);
	}
 
    Serial.println("Ending task 1");
    vTaskDelete( NULL );
}
 


void setup() {

    Serial.begin(460800);
	spi_init();

	//matrix.begin(R1_PIN_DEFAULT ,B1_PIN_DEFAULT, G1_PIN_DEFAULT , R2_PIN_DEFAULT , B2_PIN_DEFAULT, G2_PIN_DEFAULT , 22,21,17,32,33, LAT_PIN_DEFAULT, OE_PIN_DEFAULT , CLK_PIN_DEFAULT);

    // R1,B1,G1,  R2,B2,G2,  A,B,C,D,E,   LAT,OEB,CLK
	matrix.begin(33,25,32, 27,21,26, 13,12,2,15,14, 4,17,16);
        
    matrix.setTextWrap(false);
    matrix.setBrightness(BRIGHTNESS);

    xTaskCreate(
        taskOne,          /* Task function. */
        "TaskOne",        /* String with name of task. */
        2000,            /* Stack size in bytes. */
        NULL,             /* Parameter passed as input of the task */
        1,                /* Priority of the task. */
        NULL);            /* Task handle. */

    pinMode(22, OUTPUT);
    digitalWrite(22, HIGH);

    // // Test full bright of all LEDs. If brightness is too high
    // // for your current limit (i.e. USB), decrease it.
    matrix.fillScreen(LED_WHITE_LOW);
    matrix.show();
    delay(1000);
    matrix.clear();

	for(uint16_t i=0; i<32; i++)
	{
		color_test(i, 0, i);
		color_test(i, 8, i*2<<5);
		color_test(i, 16, i<<11);

		color_test(i, 24, (i*2<<5)+ i);
		color_test(i, 32, (i<<11) + i);
		color_test(i, 40, (i*2<<5) + (i<<11));

		color_test(i, 48, i+ ((i*2)<<5) + (i<<11));
	}

}

// vim:sts=4:sw=4