# RGBMatrixPanel library for the ESP32, utilising I2S DMA

得到[mrfaptastic](https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA)启发, fork并扩展了功能：

* demo : /matrixRGB/matrixRGB.ino
* 修改了IO映射，留出SPI接口可以扩展连接树莓派
* 关掉了双buffer功能，留存更多的RAM来扩展更大的显示屏：192x128
* 受SPI接口速度限制，外部SPI显示传输刷新率只有20HZ
* RAM限制和DMA输出频率限制，RGB Panel刷新率只有70HZ，
* can theoretically provide 5 bit colour: RGB555

## Usage

* 如果不需要那么高的亮度，可以将亮度调低减小电流:
`matrix.setBrightness(8);`
* 注意有的面板是RGB，有的面板是BGR，可以在直接matrix.begin()函数调整
`matrix.begin(R1,G1,B1,R2,G2,B2.....)`
`matrix.begin(B1,G1,R1,R2,G2,B2.....)`
* 暂时没测试FM612X系列驱动IC的面板，这款IC比较特殊，需要调整驱动
* 开发环境PlatformIO
  
## Circuit & PCB

By default the pin mapping is as follows (defaults defined in ESP32-RGBMatrixPanel-I2S-DMA.h).

请注意线路是按单个面板画的（64x64），如果拼接多个面板，电源需要加强加强再加强。即单独外加5V/3A以上电源。

![Circuit](/doc/matrix_sch_v01.jpg)

![Pcb3D](/doc/pcb_v01_3d.jpg)

![Pcb3D](/doc/pcb_v01.jpg)

## 连接树莓派或其它

通过SPI接口连接到树莓派，RGBMatrixPanel模拟成一块SPI TFT显示屏。
...待续

![0](doc/pizero0.jpeg)

![1](doc/pizero1.jpeg)

![2](doc/pizero2.jpeg)

## Credits

* ['ESP32-RGB64x32MatrixPanel-I2S-DMA' library](https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA)
* ['SmartMatrix' project code](https://github.com/pixelmatix/SmartMatrix/tree/teensylc)
* [Sprite_TM's demo implementation here](https://www.esp32.com/viewtopic.php?f=17&t=3188)
