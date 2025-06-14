#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "mpr121.h"

void i2c_setRegister(MPR121_t * dev, uint8_t reg, uint8_t value) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write_byte(cmd, value, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGD(__FUNCTION__, "setRegister reg=0x%02x value=0x%02x successfully", reg, value);
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
	} else {
		ESP_LOGE(__FUNCTION__, "setRegister reg=0x%02x value=0x%02x failed. code: 0x%02x", reg, value, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
	i2c_cmd_link_delete(cmd);
}

uint8_t i2c_getRegister(MPR121_t * dev, uint8_t reg) {
	ESP_LOGD(__FUNCTION__, "getRegister reg=0x%02x", reg);
	uint8_t scratch = 0;

	uint8_t buf[2];
	memset (buf, 0, 2);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->_address << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, buf, 1, I2C_MASTER_NACK);

	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
		scratch = buf[0];
		ESP_LOGD(__FUNCTION__, "getRegister reg=0x%02x successfully scratch=0x%02x", reg, scratch);
	} else {
		ESP_LOGE(__FUNCTION__, "getRegister reg=0x%02x failed. code: 0x%02x", reg, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
	i2c_cmd_link_delete(cmd);
	return scratch;
}

void i2c_register(MPR121_t * dev, int16_t sda, int16_t scl){
	ESP_LOGI(__FUNCTION__, "Legacy i2c driver is used");
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,
		.scl_io_num = scl,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));
	ESP_LOGI(__FUNCTION__, "Legacy i2c driver installed");
}
