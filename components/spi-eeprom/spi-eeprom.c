#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "spi-eeprom.h"

#define TAG "EEPROM"

#if APB_CLK_FREQ==80*1000*1000
#define SPI_MASTER_FREQ_2M	   (APB_CLK_FREQ/40)	///< 2MHz
#elif APB_CLK_FREQ==40*1000*1000
#define SPI_MASTER_FREQ_2M	   (APB_CLK_FREQ/20)	///< 4MHz
#endif

#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif


//
// Initialize devive
// SIZE(in):EEPROM SIZE(K bit)
// GPIO_CS(in):GPIO of chip select
// GPIO_MISO(in):GPIO of MISO
// GPIO_MOSI(in):GPIO of MOSI
// GPIO_SCLK(in):GPIO of SCLK
//
void spi_master_init(EEPROM_t * dev, uint32_t model, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "APB_CLK_FREQ=%d", APB_CLK_FREQ);
	ESP_LOGI(TAG, "GPIO_MISO=%d",GPIO_MISO);
	ESP_LOGI(TAG, "GPIO_MOSI=%d",GPIO_MOSI);
	ESP_LOGI(TAG, "GPIO_SCLK=%d",GPIO_SCLK);
	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	//gpio_pad_select_gpio( GPIO_CS );
	gpio_reset_pin( GPIO_CS );
	gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_CS, 0 );

	spi_bus_config_t buscfg = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( HOST_ID, &buscfg, SPI_DMA_CH_AUTO );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	// M95xxx is 8MHz Clock Rate
	int SPI_Frequency = SPI_MASTER_FREQ_8M;
	if (model == M95010) {
		dev->_totalBytes = 128;
		dev->_addressBits = 7;
		dev->_pageSize = 16;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95020) {
		dev->_totalBytes = 256;
		dev->_addressBits = 8;
		dev->_pageSize = 16;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95040) {
		dev->_totalBytes = 512;
		dev->_addressBits = 9;
		dev->_pageSize = 16;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95080) {
		dev->_totalBytes = 1024;
		dev->_addressBits = 10;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95160) {
		dev->_totalBytes = 2048;
		dev->_addressBits = 11;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95320) {
		dev->_totalBytes = 4096;
		dev->_addressBits = 12;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95640) {
		dev->_totalBytes = 8192;
		dev->_addressBits = 13;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95128) {
		dev->_totalBytes = 16384;
		dev->_addressBits = 14;
		dev->_pageSize = 64;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95256) {
		dev->_totalBytes = 32768;
		dev->_addressBits = 15;
		dev->_pageSize = 64;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == M95512) {
		dev->_totalBytes = 65536;
		dev->_addressBits = 16;
		dev->_pageSize = 128;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	}

	// AT24xxx is 2MHz Clock Rate
	if (model == AT25010) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 128;
		dev->_addressBits = 7;
		dev->_pageSize = 8;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25020) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 256;
		dev->_addressBits = 8;
		dev->_pageSize = 8;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25040) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 512;
		dev->_addressBits = 9;
		dev->_pageSize = 8;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25080) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 1024;
		dev->_addressBits = 10;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25160) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 2048;
		dev->_addressBits = 11;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25320) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 4096;
		dev->_addressBits = 12;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25640) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 8192;
		dev->_addressBits = 13;
		dev->_pageSize = 32;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25128) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 16384;
		dev->_addressBits = 14;
		dev->_pageSize = 64;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25256) {
		SPI_Frequency = SPI_MASTER_FREQ_2M;
		dev->_totalBytes = 32768;
		dev->_addressBits = 15;
		dev->_pageSize = 64;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	} else if (model == AT25512) {
		dev->_totalBytes = 65536;
		dev->_addressBits = 16;
		dev->_pageSize = 64;
		dev->_lastPage = (dev->_totalBytes/dev->_pageSize)-1;
	}

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
	devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.spics_io_num = GPIO_CS;
	devcfg.queue_size = 1;
	devcfg.mode = 0;

	spi_device_handle_t handle;
	ret = spi_bus_add_device( HOST_ID, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->_SPIHandle = handle;
}

//
// Read Status Register (RDSR)
// reg(out):Status
//
esp_err_t eeprom_ReadStatusReg(EEPROM_t * dev, uint8_t * reg)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = EEPROM_CMD_RDSR;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	ESP_LOGD(TAG, "eeprom_ReadStatusReg=%x",data[1]);
	*reg = data[1];
	return ret;
}

//
// Busy check
// true:Busy
// false:Idle
//
bool eeprom_IsBusy(EEPROM_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = EEPROM_CMD_RDSR;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;
	if( (data[1] & EEPROM_STATUS_WIP) == EEPROM_STATUS_WIP) return true;
	return false;
}


//
// Write enable check
// true:Write enable
// false:Write disable
//
bool eeprom_IsWriteEnable(EEPROM_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = EEPROM_CMD_RDSR;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;
	if( (data[1] & EEPROM_STATUS_WEL) == EEPROM_STATUS_WEL) return true;
	return false;
}

//
// Set write enable
//
esp_err_t eeprom_WriteEnable(EEPROM_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];
	data[0] = EEPROM_CMD_WREN;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	return ret;
}

//
// Set write disable
//
esp_err_t eeprom_WriteDisable(EEPROM_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];
	data[0] = EEPROM_CMD_WRDI;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	return ret;
}
//
// Read from Memory Array (READ)
// addr(in):Read start address (16-Bit Address)
// buf(out):Read data
// n(in):Number of data bytes to read
//
int16_t eeprom_Read(EEPROM_t * dev, uint16_t addr, uint8_t *buf, int16_t n)
{ 
	esp_err_t ret;
	spi_transaction_t SPITransaction;

	if (addr >= dev->_totalBytes) return 0;

	uint8_t data[4];
	int16_t index = 0;
	for (int i=0;i<n;i++) {
		uint16_t _addr = addr + i;
		data[0] = EEPROM_CMD_READ;
		if (dev->_addressBits == 9 && addr > 0xff) data[0] = data[0] | 0x08;
		if (dev->_addressBits <= 9) {
			data[1] = (_addr & 0xFF);
			index = 2;
		} else {
			data[1] = (_addr>>8) & 0xFF;
			data[2] = _addr & 0xFF;
			index = 3;
		}
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = (index+1) * 8;
		SPITransaction.tx_buffer = data;
		SPITransaction.rx_buffer = data;
		ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
		if (ret != ESP_OK) return 0;
		buf[i] = data[index];
	}
	return n;
}

//
// Write to Memory Array (WRITE)
// addr(in):Write start address (16-Bit Address)
// wdata(in):Write data
//
int16_t eeprom_WriteByte(EEPROM_t * dev, uint16_t addr, uint8_t wdata)
{
	esp_err_t ret;
	spi_transaction_t SPITransaction;

	if (addr >= dev->_totalBytes) return 0;

	// Set write enable
	ret = eeprom_WriteEnable(dev);
	if (ret != ESP_OK) return 0;

#if 0
	// Check busy
	if (eeprom_IsBusy(dev)) return 0;
#endif
	// Wait for idle
	while( eeprom_IsBusy(dev) ) {
		vTaskDelay(1);
	}

	uint8_t data[4];
	int16_t index = 0;
	data[0] = EEPROM_CMD_WRITE;
	if (dev->_addressBits == 9 && addr > 0xff) data[0] = data[0] | 0x08;
	if (dev->_addressBits <= 9) {
		data[1] = (addr & 0xFF);
		data[2] = wdata;
		index = 2;
	} else {
		data[1] = (addr>>8) & 0xFF;
		data[2] = addr & 0xFF;
		data[3] = wdata;
		index = 3;
	}

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (index+1) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return 0;

	// Wait for idle
	while( eeprom_IsBusy(dev) ) {
		vTaskDelay(1);
	}
	return 1;
}

//
// Page Write to Memory Array (WRITE)
// pages(in):Write page address
// buf(in):Write data
//
int16_t eeprom_WritePage(EEPROM_t * dev, int16_t pages, uint8_t* buf)
{
	esp_err_t ret;
	spi_transaction_t SPITransaction;

	if (pages > dev->_lastPage) return 0;

	// Set write enable
	ret = eeprom_WriteEnable(dev);
	if (ret != ESP_OK) return 0;

#if 0
	// Check busy
	if (eeprom_IsBusy(dev)) return 0;	
#endif
	// Wait for idle
	while( eeprom_IsBusy(dev) ) {
		vTaskDelay(1);
	}

	ESP_LOGD(TAG, "_pageSize=%d",dev->_pageSize);
	uint16_t addr = pages * dev->_pageSize;
	uint8_t data[131];
	int16_t index = 0;
	data[0] = EEPROM_CMD_WRITE;
	if (dev->_addressBits == 9 && addr > 0xff) data[0] = data[0] | 0x08;
	if (dev->_addressBits <= 9) {
		data[1] = (addr & 0xFF);
		memcpy( &data[2], buf, dev->_pageSize );
		index = 2;
	} else {
		data[1] = (addr>>8) & 0xFF;
		data[2] = addr & 0xFF;
		memcpy( &data[3], buf, dev->_pageSize );
		index = 3;
	}

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (dev->_pageSize+index) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return 0;

	// Wait for idle
	while( eeprom_IsBusy(dev) ) {
		vTaskDelay(1);
	}
	return dev->_pageSize;
}

// Get total byte
//
int32_t eeprom_TotalBytes(EEPROM_t * dev)
{
	return dev->_totalBytes;
}

// Get page size
//
int16_t eeprom_PageSize(EEPROM_t * dev)
{
	return dev->_pageSize;
}

// Get last page
//
int16_t eeprom_LastPage(EEPROM_t * dev)
{
	return dev->_lastPage;
}
