# esp-idf-spi-eeprom
SPI EEPROM Driver for esp-idf   


# Configure
You have to set this config value with menuconfig.   
- CONFIG_MODEL   
- CONFIG_SPI   
- CONFIG_CS_GPIO   

```
git clone https://github.com/nopnop2002/esp-idf-spi-eeprom
cd esp-idf-esp-eeprom
make menuconfig
make flash
```

![config-1](https://user-images.githubusercontent.com/6020549/96329346-f4950100-1086-11eb-88f9-57d31933c1c3.jpg)
![config-2](https://user-images.githubusercontent.com/6020549/96329347-f78ff180-1086-11eb-9ee7-3c7209b01a60.jpg)
![config-3](https://user-images.githubusercontent.com/6020549/96355105-fe674480-1118-11eb-9b3e-c9d429ce271c.jpg)
![config-4](https://user-images.githubusercontent.com/6020549/96329351-fced3c00-1086-11eb-92e5-1f4bcdbeda8b.jpg)

---

# Memory size

## ST Micro   
10 MHz Clock Rate.   
There are several variations in the M95 series.   
4.5 V to 5.5 V for M95xxx   
2.5 V to 5.5 V for M95xxx-W   
1.8 V to 5.5 V for M95xxx-R   

__M95xxx don't work with ESP32__

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
3.0 MHz Clock Rate.   
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

---

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

---

# Wireing  

|#|EEPROM||ESP32(SPI2)|ESP32(SPI3)
|:-:|:-:|:-:|:-:|:-:|
|1|/CS|--|GPIO15(*)|GPIO5(*)|
|2|MISO|--|GPIO12|GPIO19|
|3|/WP|--|3.3V|3.3V|
|4|VSS|--|GND|GND|
|5|MOSI|--|GPIO13|GPIO23|
|6|SCK|--|GPIO14|GPIO18|
|7|/HOLD|--|3.3V|3.3V|
|8|VCC|--|3.3V|3.3V|

(*) You can change any GPIO using menuconfig.   

---

# Serial Monitor   
![monitor](https://user-images.githubusercontent.com/6020549/96329356-09719480-1087-11eb-85b3-0601f77a3772.jpg)
