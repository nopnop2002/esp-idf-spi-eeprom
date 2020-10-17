#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "m95.h"

static const char *TAG = "MAIN";

#if CONFIG_SPI2
static const int GPIO_MISO = 12;
static const int GPIO_MOSI = 13;
static const int GPIO_SCLK = 14;
#endif

#if CONFIG_SPI3
static const int GPIO_MISO = 19;
static const int GPIO_MOSI = 23;
static const int GPIO_SCLK = 18;
#endif

#if CONFIG_M95010
#define	EEPROM_SIZE		1
#endif

#if CONFIG_M95020
#define	EEPROM_SIZE		2
#endif

#if CONFIG_M95040
#define	EEPROM_SIZE		4
#endif

#if CONFIG_M95080
#define	EEPROM_SIZE		8
#endif

#if CONFIG_M95160
#define	EEPROM_SIZE		16
#endif

#if CONFIG_M95320
#define	EEPROM_SIZE		32
#endif

#if CONFIG_M95640
#define	EEPROM_SIZE		64
#endif

#if CONFIG_M95128
#define	EEPROM_SIZE		128
#endif

#if CONFIG_M95256
#define	EEPROM_SIZE		256
#endif


void dump(uint8_t *dt, int n)
{
	uint16_t clm = 0;
	uint8_t data;
	uint32_t saddr =0;
	uint32_t eaddr =n-1;

	printf("--------------------------------------------------------\n");
	uint32_t addr;
	for (addr = saddr; addr <= eaddr; addr++) {
		data = dt[addr];
		if (clm == 0) {
			printf("%05x: ",addr);
		}

		printf("%02x ",data);
		clm++;
		if (clm == 16) {
			printf("| \n");
			clm = 0;
		}
	}
	printf("--------------------------------------------------------\n");
}

void app_main(void)
{
	ESP_LOGI(TAG, "EEPROM_SIZE=%d", EEPROM_SIZE);
	ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d", CONFIG_CS_GPIO);
	M95_t dev;
	spi_master_init(&dev, EEPROM_SIZE, CONFIG_CS_GPIO, GPIO_MISO, GPIO_MOSI, GPIO_SCLK);
	ESP_LOGI(TAG, "PageSize=%d Bytes",M95_PageSize(&dev));

	// Get Status Register
	uint8_t reg;
	esp_err_t ret;
	ret = M95_ReadStatusReg(&dev, &reg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ReadStatusReg fail %d",ret);
		while(1) { vTaskDelay(1); }
	} 
	ESP_LOGI(TAG, "readStatusReg : %x", reg);

	// Page Write
	uint8_t wdata[128];
	int len;
	for (int i=0; i<128; i++) {
		wdata[i]=0xff;	
	}  
	for (uint16_t addr=0; addr<128;addr=addr+16) {
		len =  M95_PageWrite(&dev, addr, &wdata[addr], 16);
		if (len != 16) {
			ESP_LOGE(TAG, "Page Write fail");
			while(1) { vTaskDelay(1); }
		}
		ESP_LOGD(TAG, "Page Write(Address=%d) len=%d", addr, len);
	}

	// Read 128 byte data from Address=0
	uint8_t rbuf[128];
	memset(rbuf, 0, 128);
	len =  M95_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);

	// Page Write
	for (int i=0; i<128; i++) {
		wdata[i]=i;	
	}  
	for (uint16_t addr=0; addr<128;addr=addr+16) {
		len =  M95_PageWrite(&dev, addr, &wdata[addr], 16);
		if (len != 16) {
			ESP_LOGE(TAG, "Page Write fail");
			while(1) { vTaskDelay(1); }
		}
		ESP_LOGD(TAG, "Page Write(Address=%d) len=%d", addr, len);
	}

	// Read 128 byte data from Address=0
	memset(rbuf, 0, 128);
	len =  M95_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);

	// Write data to Address=0x00
	for (int i=0; i<128; i++) {
		wdata[i]=0xff-i;	
	}  
	for (int addr=0; addr<128;addr++) {
		len =  M95_Write(&dev, addr, wdata[addr]);
		if (len != 1) {
			ESP_LOGE(TAG, "Write fail");
			while(1) { vTaskDelay(1); }
		}
		ESP_LOGD(TAG, "Write(Address=%d) len=%d", addr, len);
	}

	// Read 128 byte data from Address=0
	memset(rbuf, 0, 128);
	len =  M95_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);


}


