# esp-idf-m95
M95 series SPI EEPROM Access Library for esp-idf   

There are several variations in the M95 series.   
4.5 V to 5.5 V for M95xxx   
2.5 V to 5.5 V for M95xxx-W   
1.8 V to 5.5 V for M95xxx-R   

__M95xxx don't work with ESP32__

# Configure
You have to set this config value with menuconfig.   
- CONFIG_MODEL   
- CONFIG_SPI   
- CONFIG_CS_GPIO   

```
git clone https://github.com/nopnop2002/esp-idf-m95
cd esp-idf-m95
make menuconfig
make flash
```

![config-1](https://user-images.githubusercontent.com/6020549/96329346-f4950100-1086-11eb-88f9-57d31933c1c3.jpg)
![config-2](https://user-images.githubusercontent.com/6020549/96329347-f78ff180-1086-11eb-9ee7-3c7209b01a60.jpg)
![config-3](https://user-images.githubusercontent.com/6020549/96329349-fb237880-1086-11eb-9b69-e1c7b3bd3762.jpg)
![config-4](https://user-images.githubusercontent.com/6020549/96329351-fced3c00-1086-11eb-92e5-1f4bcdbeda8b.jpg)

---

# Memory size

|Device|# of Bits|# of Bytes|Byte Address range|Page Size(Byte)|
|:---|:---|:---|:---|:---|
|M95010|1K|128|0x00-0x7F|16|
|M95020|2K|256|0x00-0xFF|16|
|M95040|4K|512|0x00-0x1FF|16|
|M95080|8K|1024|0x00-0x3FF|32|
|M95160|16K|2048|0x00-0x7FF|32|
|M95320|32K|4196|0x00-0xFFF|32|
|M95640|64K|8192|0x00-0x1FFF|32|
|M95128|128K|16384|0x00-0x3FFF|64|
|M95256|256K|32768|0x00-0x7FFF|64|

---

# API
```
// Open device
void spi_master_init(M95_t * dev, int16_t SIZE, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK);

// Read Status Register (RDSR)
esp_err_t M95_ReadStatusReg(M95_t * dev, uint8_t * reg);

// Busy check
bool M95_IsBusy(M95_t * dev);

// Write enable check
bool M95_IsWriteEnable(M95_t * dev);

// Set write enable
esp_err_t M95_WriteEnable(M95_t * dev);

// Set write disable
esp_err_t M95_WriteDisable(M95_t * dev);

// Read from Memory Array (READ)
int16_t M95_Read(M95_t * dev, uint16_t addr, uint8_t *buf, int16_t n);

// Write to Memory Array (WRITE)
int16_t M95_Write(M95_t * dev, uint16_t addr, uint8_t wdata);

// Get page byte size
int16_t M95_PageSize(M95_t * dev)

// Page Write to Memory Array (WRITE)
int16_t M95_PageWrite(M95_t * dev, uint16_t addr, uint8_t* buf, int16_t n)
```

---

# Wireing  

|#|M95 EEPROM||ESP32(SPI2)|ESP32(SPI3)
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
