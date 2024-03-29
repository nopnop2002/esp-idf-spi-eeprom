# esp-idf-spi-eeprom
SPI EEPROM Driver for esp-idf.

What is the difference between flash memory and EEPROM?   
[This](https://www.electronicsforu.com/technology-trends/learn-electronics/eeprom-difference-flash-memory) is the difference between flash memory and EEPROM.   

# Software requirements   
ESP-IDF V4.4/V5.0.   
ESP-IDF V5.0 is required when using ESP32-C2.   

# Installation
```
git clone https://github.com/nopnop2002/esp-idf-spi-eeprom
cd esp-idf-spi-eeprom
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3}
idf.py menuconfig
idf.py flash
```

# Configuration
You have to set this config value with menuconfig.   
- CONFIG_MODEL   
- CONFIG_CS_GPIO   
- CONFIG_SCLK_GPIO   
- CONFIG_MISO_GPIO   
- CONFIG_MOSI_GPIO   

![config-main](https://user-images.githubusercontent.com/6020549/137619902-92071025-6549-40a6-8339-d157f1c96ba8.jpg)
![config-app](https://user-images.githubusercontent.com/6020549/203246605-1274b27f-2bc7-4a02-9c5a-90398ab7358c.jpg)
![config-device](https://user-images.githubusercontent.com/6020549/137619926-d42630ed-3665-4b0e-9604-89da93ccdc38.jpg)


# SPI BUS selection   
![config-spi-bus](https://user-images.githubusercontent.com/6020549/203246853-50599729-3dd8-445f-92da-a78cb9cfd984.jpg)

The ESP32 series has three SPI BUSs.   
SPI1_HOST is used for communication with Flash memory.   
You can use SPI2_HOST and SPI3_HOST freely.   
When you use SDSPI(SD Card via SPI), SDSPI uses SPI2_HOST BUS.   
When using this module at the same time as SDSPI or other SPI device using SPI2_HOST, it needs to be changed to SPI3_HOST.   
When you don't use SDSPI, both SPI2_HOST and SPI3_HOST will work.   
Previously it was called HSPI_HOST / VSPI_HOST, but now it is called SPI2_HOST / SPI3_HOST.   


# Memory size

## ST Micro   
8 MHz Clock Rate.   
There are several variations in the M95 series.   
4.5 V to 5.5 V for M95xxx(__Not available on ESP32__)   
2.5 V to 5.5 V for M95xxx-W   
1.8 V to 5.5 V for M95xxx-R   

|Device|# of Bits|# of Bytes|Byte Address range|Page Size(Byte)|Page Address Range|
|:---|:---|:---|:---|:---|:---|
|M95010|1K|128|0x00-0x7F|16|0-7|
|M95020|2K|256|0x00-0xFF|16|0-15|
|M95040|4K|512|0x00-0x1FF|16|0-31|
|M95080|8K|1024|0x00-0x3FF|32|0-31|
|M95160|16K|2048|0x00-0x7FF|32|0-63|
|M95320|32K|4096|0x00-0xFFF|32|0-127|
|M95640|64K|8192|0x00-0x1FFF|32|0-255|
|M95128|128K|16384|0x00-0x3FFF|64|0-255|
|M95256|256K|32768|0x00-0x7FFF|64|0-511|
|M95512|512K|65536|0x00-0xFFFF|128|0-511|

## ATMEL   
2 MHz Clock Rate.   
2.7V to 5.5V   

|Device|# of Bits|# of Bytes|Byte Address Range|Page Size(Byte)|Page Address Range|
|:---|:---|:---|:---|:---|:---|
|AT25010|1K|128|0-0x7F|8|0-15|
|AT25020|2K|256|0-0xFF|8|0-31|
|AT25040|4K|512|0-0x1FF|8|0-63|
|AT25080|8K|1024|0-0x3FF|32|0-31|
|AT25160|16K|2048|0-0x7FF|32|0-63|
|AT25320|32K|4096|0-0xFFF|32|0-127|
|AT25640|64K|8192|0-0x1FFF|32|0-255|
|AT25128|128K|16384|0-0x3FFF|64|0-255|
|AT25256|256K|32768|0-0x7FFF|64|0-511|
|AT25512|512K|65536|0-0xFFFF|128|0-511|


# API
```
// Open device
void spi_master_init(EEPROM_t * dev, uint32_t model, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK)

// Read Status Register (RDSR)
esp_err_t eeprom_ReadStatusReg(EEPROM_t * dev, uint8_t * reg)

// Busy check
bool eeprom_IsBusy(EEPROM_t * dev)

// Write enable check
bool eeprom_IsWriteEnable(EEPROM_t * dev)

// Set write enable
esp_err_t eeprom_WriteEnable(EEPROM_t * dev)

// Set write disable
esp_err_t eeprom_WriteDisable(EEPROM_t * dev)

// Read from Memory Array (READ)
int16_t eeprom_Read(EEPROM_t * dev, uint16_t addr, uint8_t *buf, int16_t n)

// Write to Memory Array (WRITE)
int16_t eeprom_WriteByte(EEPROM_t * dev, uint16_t addr, uint8_t wdata)

// Page Write to Memory Array (WRITE)
int16_t eeprom_WritePage(EEPROM_t * dev, int16_t pages, uint8_t* buf)

// Get total byte
int32_t eeprom_TotalBytes(EEPROM_t * dev)

// Get page size
int16_t eeprom_PageSize(EEPROM_t * dev)

// Get last page
int16_t eeprom_LastPage(EEPROM_t * dev)
```


# Wireing  

|#|EEPROM||ESP32|ESP32-S2/S3|ESP32-C2/C3|
|:-:|:-:|:-:|:-:|:-:|:-:|
|1|/CS|--|GPIO5|GPIO34|GPIO0|
|2|MISO|--|GPIO19|GPIO37|GPIO2|
|3|/WP|--|3.3V|3.3V|3.3V|
|4|VSS|--|GND|GND|GND|
|5|MOSI|--|GPIO23|GPIO35|GPIO3|
|6|SCK|--|GPIO18|GPIO36|GPIO1|
|7|/HOLD|--|3.3V|3.3V|3.3V|
|8|VCC|--|3.3V|3.3V|3.3V|

You can change any GPIO using menuconfig.   


# Serial Monitor   
![monitor](https://user-images.githubusercontent.com/6020549/96329356-09719480-1087-11eb-85b3-0601f77a3772.jpg)
