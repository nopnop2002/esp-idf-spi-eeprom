#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "m95.h"

#define TAG "M95"

#if 0
static const int GPIO_MISO = 12;
static const int GPIO_MOSI = 13;
static const int GPIO_SCLK = 14;
#endif

static const int SPI_Frequency = SPI_MASTER_FREQ_8M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_80M;


//
// Initialize devive
// SIZE(in):EEPROM SIZE(K bit)
// GPIO_CS(in):GPIO of chip select
// GPIO_MISO(in):GPIO of MISO
// GPIO_MOSI(in):GPIO of MOSI
// GPIO_SCLK(in):GPIO of SCLK
//
void spi_master_init(M95_t * dev, int16_t SIZE, int16_t GPIO_CS, int GPIO_MISO, int GPIO_MOSI, int GPIO_SCLK)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "GPIO_CS=%d",GPIO_CS);
	gpio_pad_select_gpio( GPIO_CS );
	gpio_set_direction( GPIO_CS, GPIO_MODE_OUTPUT );
	gpio_set_level( GPIO_CS, 0 );

	spi_bus_config_t spi_bus_config = {
		.sclk_io_num = GPIO_SCLK,
		.mosi_io_num = GPIO_MOSI,
		.miso_io_num = GPIO_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( HSPI_HOST, &spi_bus_config, 1 );
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
	devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.spics_io_num = GPIO_CS;
	devcfg.queue_size = 1;
	devcfg.mode = 0;
	//devcfg.flags = SPI_DEVICE_NO_DUMMY;

	spi_device_handle_t handle;
	ret = spi_bus_add_device( HSPI_HOST, &devcfg, &handle);
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->_size = SIZE;
	dev->_SPIHandle = handle;
}

//
// Read Status Register (RDSR)
// reg(out):Status
//
esp_err_t M95_ReadStatusReg(M95_t * dev, uint8_t * reg)
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
	ESP_LOGI(TAG, "M95_ReadStatusReg=%x",data[1]);
	*reg = data[1];
	return ret;
}

//
// Busy check
// true:Busy
// false:Idle
//
bool M95_IsBusy(M95_t * dev)
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
bool M95_IsWriteEnable(M95_t * dev)
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
esp_err_t M95_WriteEnable(M95_t * dev)
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
esp_err_t M95_WriteDisable(M95_t * dev)
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
int16_t M95_Read(M95_t * dev, uint16_t addr, uint8_t *buf, int16_t n)
{ 
	esp_err_t ret;
	spi_transaction_t SPITransaction;
	uint8_t data[4];
	int16_t offset = 0;
	for (int i=0;i<n;i++) {
		uint16_t _addr = addr + i;
		data[0] = EEPROM_CMD_READ;
		if (dev->_size <= 4) { // M95010,M95020,M95040
			if (dev->_size == 4 && (addr & 0x10) == 0x10) {
				data[0] = EEPROM_CMD_WRITE | 0x08;
			}
			data[1] = (_addr & 0xFF);
			offset = 2;
		} else {
			data[1] = (_addr>>8) & 0xFF;
			data[2] = _addr & 0xFF;
			offset = 3;
		}
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = (offset+1) * 8;
		SPITransaction.tx_buffer = data;
		SPITransaction.rx_buffer = data;
		ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
		if (ret != ESP_OK) return 0;
		buf[i] = data[offset];
	}
	return n;
}

//
// Write to Memory Array (WRITE)
// addr(in):Write start address (16-Bit Address)
// wdata(in):Write data
//
int16_t M95_Write(M95_t * dev, uint16_t addr, uint8_t wdata)
{
	esp_err_t ret;
	spi_transaction_t SPITransaction;

	// Set write enable
	ret = M95_WriteEnable(dev);
	if (ret != ESP_OK) return 0;

	// Check busy
	if (M95_IsBusy(dev)) return 0;

	uint8_t data[4];
	int16_t offset = 0;
	data[0] = EEPROM_CMD_WRITE;
	if (dev->_size <= 4) { // M95010,M95020,M95040
		if (dev->_size == 4 && (addr & 0x10) == 0x10) {
			data[0] = EEPROM_CMD_WRITE | 0x08;
		}
		data[1] = (addr & 0xFF);
		data[2] = wdata;
		offset = 2;
	} else {
		data[1] = (addr>>8) & 0xFF;
		data[2] = addr & 0xFF;
		data[3] = wdata;
		offset = 3;
	}

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (offset+1) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return 0;

	// Wait for idle
	while( M95_IsBusy(dev) ) {
		vTaskDelay(1);
	}
	return 1;
}

// Get page byte size
int16_t M95_PageSize(M95_t * dev)
{
	if (dev->_size <= 4) { // M95010,M95020,M95040
		return 16;
	} else if (dev->_size <= 64) { // M95080,M95160,M95320,M95640
		return 32;
	} else { // M95128,M95256
		return 64;
	}
}
//
// Page Write to Memory Array (WRITE)
// addr(in):Write start address (16-Bit Address)
// buf(in):Write data
//         M32010:Up to 16 Bytes
//         M32020:Up to 16 Bytes
//         M32040:Up to 16 Bytes
//         M32080:Up to 32 Bytes
//         M32160:Up to 32 Bytes
//         M32320:Up to 32 Bytes
//         M32640:Up to 32 Bytes
//         M32128:Up to 64 Bytes
//         M32256:Up to 64 Bytes
// n(in):Number of data bytes to write
//
int16_t M95_PageWrite(M95_t * dev, uint16_t addr, uint8_t* buf, int16_t n)
{
	esp_err_t ret;
	spi_transaction_t SPITransaction;
	ESP_LOGD(TAG, "M95_PageSize=%d",M95_PageSize(dev));
	int16_t _n = n;
	if (_n > M95_PageSize(dev)) _n = M95_PageSize(dev);

	// Set write enable
	ret = M95_WriteEnable(dev);
	if (ret != ESP_OK) return 0;

	// Check busy
	if (M95_IsBusy(dev)) return 0;	

	uint8_t data[67];
	int16_t offset = 0;
	data[0] = EEPROM_CMD_WRITE;
	if (dev->_size <= 4) { // M95010,M95020,M95040
		if (dev->_size == 4 && (addr & 0x10) == 0x10) {
			data[0] = EEPROM_CMD_WRITE | 0x08;
		}
		data[1] = (addr & 0xFF);
		memcpy( &data[2], buf, _n );
		offset = 2;
	} else {
		data[1] = (addr>>8) & 0xFF;
		data[2] = addr & 0xFF;
		memcpy( &data[3], buf, _n );
		offset = 3;
	}

	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (_n+offset) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return 0;

	// Wait for idle
	while( M95_IsBusy(dev) ) {
		vTaskDelay(1);
	}
	return _n;
}
