/* MPR121 Example

	 This example code is in the Public Domain (or CC0 licensed, at your option.)

	 Unless required by applicable law or agreed to in writing, this
	 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
	 CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mpr121.h"

static const char *TAG = "MAIN";

void app_main(void)
{
	ESP_LOGI(TAG, "CONFIG_I2C_ADDRESS=0x%X", CONFIG_I2C_ADDRESS);
	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d", CONFIG_SCL_GPIO);
	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d", CONFIG_SDA_GPIO);
	ESP_LOGI(TAG, "CONFIG_IRQ_GPIO=%d", CONFIG_IRQ_GPIO);
	MPR121_t dev;

	uint16_t touchThreshold = 40;
	uint16_t releaseThreshold = 20;
	//uint16_t interruptPin = 4;

	bool ret = MPR121_begin(&dev, CONFIG_I2C_ADDRESS, touchThreshold, releaseThreshold, CONFIG_IRQ_GPIO, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO);
	ESP_LOGI(TAG, "MPR121_begin=%d", ret);
	if (ret == false) {
		switch (MPR121_getError(&dev)) {
			case NO_ERROR:
				ESP_LOGE(TAG, "no error");
				break;
			case ADDRESS_UNKNOWN:
				ESP_LOGE(TAG, "incorrect address");
				break;
			case READBACK_FAIL:
				ESP_LOGE(TAG, "readback failure");
				break;
			case OVERCURRENT_FLAG:
				ESP_LOGE(TAG, "overcurrent on REXT pin");
				break;
			case OUT_OF_RANGE:
				ESP_LOGE(TAG, "electrode out of range");
				break;
			case NOT_INITED:
				ESP_LOGE(TAG, "not initialised");
				break;
			default:
				ESP_LOGE(TAG, "unknown error");
				break;
		}
		while(1) {
			vTaskDelay(1);
		}
	}


#if 0
	MPR121_setTouchThresholdAll(&dev, 40);	// this is the touch threshold - setting it low makes it more like a proximity trigger, default value is 40 for touch
	MPR121_setReleaseThresholdAll(&dev, 20);	// this is the release threshold - must ALWAYS be smaller than the touch threshold, default value is 20 for touch
#endif


	MPR121_setFFI(&dev, FFI_10); // AFE Configuration 1
	MPR121_setSFI(&dev, SFI_10); // AFE Configuration 2
	MPR121_setGlobalCDT(&dev, CDT_4US);  // reasonable for larger capacitances
	MPR121_autoSetElectrodesDefault(&dev, true);	// autoset all electrode settings


	while(1) {
		MPR121_updateAll(&dev);
		for (int i = 0; i < 12; i++) {
			if (MPR121_isNewTouch(&dev, i)) {
				ESP_LOGI(TAG, "electrode %d was just touched", i);
			} else if (MPR121_isNewRelease(&dev, i)) {
				ESP_LOGI(TAG, "electrode %d was just released", i);
			}
		}
		vTaskDelay(10);
	}
}
