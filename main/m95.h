#ifndef MAIN_M95_H_
#define MAIN_M95_H_

#include "driver/spi_master.h"

/* EEPROM SPI commands */
#define EEPROM_CMD_WREN    0x06    // Write Enable
#define EEPROM_CMD_WRDI    0x04    // Write Disable
#define EEPROM_CMD_RDSR    0x05    // Read Status Register
#define EEPROM_CMD_WRSR    0x01    // Write Status Register
#define EEPROM_CMD_READ    0x03    // Read from Memory Array
#define EEPROM_CMD_WRITE   0x02    // Write to Memory Array   

/* EEPROM SPI status */
#define EEPROM_STATUS_SRWD    0x80       // Status Register Write Disable
#define EEPROM_STATUS_BP      0x0C       // Block Protect
#define EEPROM_STATUS_WEL     0x02       // Write Enable   
#define EEPROM_STATUS_WIP     0x01       // Write in Progress

typedef struct {
	int16_t	_size;
	spi_device_handle_t _SPIHandle;
} M95_t;

void spi_master_init(M95_t * dev, int16_t SIZE, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK);
esp_err_t M95_ReadStatusReg(M95_t * dev, uint8_t * reg);
bool M95_IsBusy(M95_t * dev);
bool M95_IsWriteEnable(M95_t * dev);
esp_err_t M95_WriteEnable(M95_t * dev);
esp_err_t M95_WriteDisable(M95_t * dev);
int16_t M95_Read(M95_t * dev, uint16_t addr, uint8_t *buf, int16_t n);
int16_t M95_Write(M95_t * dev, uint16_t addr, uint8_t wdata);
int16_t M95_PageSize(M95_t * dev);
int16_t M95_PageWrite(M95_t * dev, uint16_t addr, uint8_t* buf, int16_t n);

#endif /* MAIN_M95_H_ */

