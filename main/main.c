#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "eeprom.h"

static const char *TAG = "MAIN";

#if CONFIG_M95010
#define	EEPROM_MODEL		M95010
#endif

#if CONFIG_M95020
#define	EEPROM_MODEL		M95020
#endif

#if CONFIG_M95040
#define	EEPROM_MODEL		M95040
#endif

#if CONFIG_M95080
#define	EEPROM_MODEL		M95080
#endif

#if CONFIG_M95160
#define	EEPROM_MODEL		M95160
#endif

#if CONFIG_M95320
#define	EEPROM_MODEL		M95320
#endif

#if CONFIG_M95640
#define	EEPROM_MODEL		M95640
#endif

#if CONFIG_M95128
#define	EEPROM_MODEL		M95128
#endif

#if CONFIG_M95256
#define	EEPROM_MODEL		M95256
#endif

#if CONFIG_M95512
#define	EEPROM_MODEL		M95512
#endif

#if CONFIG_AT25010
#define EEPROM_MODEL        AT25010
#endif

#if CONFIG_AT25020
#define EEPROM_MODEL        AT25020
#endif

#if CONFIG_AT25040
#define EEPROM_MODEL        AT25040
#endif

#if CONFIG_AT25080
#define EEPROM_MODEL        AT25080
#endif

#if CONFIG_AT25160
#define EEPROM_MODEL        AT25160
#endif

#if CONFIG_AT25320
#define EEPROM_MODEL        AT25320
#endif

#if CONFIG_AT25640
#define EEPROM_MODEL        AT25640
#endif

#if CONFIG_AT25128
#define EEPROM_MODEL        AT25128
#endif

#if CONFIG_AT25256
#define EEPROM_MODEL        AT25256
#endif

#if CONFIG_AT25512
#define EEPROM_MODEL        AT25512
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
	ESP_LOGI(TAG, "EEPROM_MODEL=%d", EEPROM_MODEL);
	EEPROM_t dev;
	spi_master_init(&dev, EEPROM_MODEL, CONFIG_CS_GPIO, CONFIG_MISO_GPIO, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO);
	int32_t totalBytes = eeprom_TotalBytes(&dev);
	ESP_LOGI(TAG, "totalBytes=%d Bytes",totalBytes);
	int16_t pageSize = eeprom_PageSize(&dev);
	ESP_LOGI(TAG, "pageSize=%d Bytes",pageSize);
	int16_t lastPage = eeprom_LastPage(&dev);
	ESP_LOGI(TAG, "lastPage=%d Page",lastPage);

	// Get Status Register
	uint8_t reg;
	esp_err_t ret;
	ret = eeprom_ReadStatusReg(&dev, &reg);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "ReadStatusReg Fail %d",ret);
		while(1) { vTaskDelay(1); }
	} 
	ESP_LOGI(TAG, "readStatusReg : 0x%02x", reg);

	// Write Page
	uint8_t wdata[128];
	int len;
	for (int i=0; i<128; i++) {
		wdata[i]=0xff;	
	}  
	int numPage = 128 / pageSize;
	ESP_LOGI(TAG, "numPage : %d", numPage);
	for (uint16_t pages=0; pages<numPage;pages++) {
		int addr = pages * pageSize;
		len =  eeprom_WritePage(&dev, pages, &wdata[addr]);
		ESP_LOGD(TAG, "WritePage(pages=%d) len=%d", pages, len);
		if (len != pageSize) {
			ESP_LOGE(TAG, "WritePage Fail pages=%d", pages);
			while(1) { vTaskDelay(1); }
		}
	}

	// Read 128 byte from Address=0
	uint8_t rbuf[128];
	memset(rbuf, 0, 128);
	len =  eeprom_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read Fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);

	// Write Page
	for (int i=0; i<128; i++) {
		wdata[i]=i;	
	}  
	for (uint16_t pages=0; pages<numPage;pages++) {
		int addr = pages * pageSize;
		len =  eeprom_WritePage(&dev, pages, &wdata[addr]);
		ESP_LOGD(TAG, "WritePage(pages=%d) len=%d", pages, len);
		if (len != pageSize) {
			ESP_LOGE(TAG, "WritePage Fail pages=%d", pages);
			while(1) { vTaskDelay(1); }
		}
	}

	// Read 128 byte from Address=0
	memset(rbuf, 0, 128);
	len =  eeprom_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read Fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);

	// Write Byte
	for (int i=0; i<128; i++) {
		wdata[i]=0xff-i;	
	}  
	for (int addr=0; addr<128;addr++) {
		len =  eeprom_WriteByte(&dev, addr, wdata[addr]);
		ESP_LOGD(TAG, "WriteByte(addr=%d) len=%d", addr, len);
		if (len != 1) {
			ESP_LOGE(TAG, "WriteByte Fail addr=%d", addr);
			while(1) { vTaskDelay(1); }
		}
	}

	// Read 128 byte from Address=0
	memset(rbuf, 0, 128);
	len =  eeprom_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGE(TAG, "Read Fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);
}

