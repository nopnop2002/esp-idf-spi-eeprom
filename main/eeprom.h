#ifndef MAIN_EEPROM_H_
#define MAIN_EEPROM_H_

#include "driver/spi_master.h"

/* EEPROM Model */
#define	M95010	95010
#define	M95020	95020
#define	M95040	95040
#define	M95080	95080
#define	M95160	95160
#define	M95320	95320
#define	M95640	95640
#define	M95128	95128
#define	M95256	95256

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
	int16_t	_totalBytes;
	int16_t	_addressBits;
	int16_t	_pageSize;
	int16_t	_lastPage;
	spi_device_handle_t _SPIHandle;
} EEPROM_t;

void spi_master_init(EEPROM_t * dev, uint32_t model, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK);
esp_err_t eeprom_ReadStatusReg(EEPROM_t * dev, uint8_t * reg);
bool eeprom_IsBusy(EEPROM_t * dev);
bool eeprom_IsWriteEnable(EEPROM_t * dev);
esp_err_t eeprom_WriteEnable(EEPROM_t * dev);
esp_err_t eeprom_WriteDisable(EEPROM_t * dev);
int16_t eeprom_Read(EEPROM_t * dev, uint16_t addr, uint8_t *buf, int16_t n);
int16_t eeprom_WriteByte(EEPROM_t * dev, uint16_t addr, uint8_t wdata);
int16_t eeprom_WritePage(EEPROM_t * dev, int16_t pages, uint8_t* buf);
int16_t eeprom_TotalBytes(EEPROM_t * dev);
int16_t eeprom_PageSize(EEPROM_t * dev);
int16_t eeprom_LastPage(EEPROM_t * dev);

#endif /* MAIN_EEPROM_H_ */

