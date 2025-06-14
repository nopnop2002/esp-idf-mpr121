#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "mpr121.h"

void i2c_setRegister(MPR121_t * dev, uint8_t reg, uint8_t value) {
	uint8_t out_buf[2];
	out_buf[0] = reg;
	out_buf[1] = value;
	esp_err_t espRc = i2c_master_transmit(dev->_i2c_dev_handle, out_buf, 2, I2C_TICKS_TO_WAIT);
	if (espRc == ESP_OK) {
		ESP_LOGD(__FUNCTION__, "setRegister reg=0x%02x value=0x%02x successfully", reg, value);
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
	} else {
		ESP_LOGE(__FUNCTION__, "setRegister reg=0x%02x value=0x%02x failed. code: 0x%02x", reg, value, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
}

uint8_t i2c_getRegister(MPR121_t * dev, uint8_t reg) {
	uint8_t scratch = 0;
	uint8_t out_buf[1];
	out_buf[0] = reg;
	uint8_t in_buf[1];
	esp_err_t espRc = i2c_master_transmit_receive(dev->_i2c_dev_handle, out_buf, 1, in_buf, 1, -1);
	if (espRc == ESP_OK) {
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
		scratch = in_buf[0];
		ESP_LOGD(__FUNCTION__, "getRegister reg=0x%02x successfully scratch=0x%02x", reg, scratch);
	} else {
		ESP_LOGE(__FUNCTION__, "getRegister reg=0x%02x failed. code: 0x%02x", reg, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
	return scratch;
}

void i2c_register(MPR121_t * dev, int16_t sda, int16_t scl){
	ESP_LOGI(__FUNCTION__, "New i2c driver is used");
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM,
		.scl_io_num = scl,
		.sda_io_num = sda,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t i2c_bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = dev->_address,
		.scl_speed_hz = I2C_MASTER_FREQ_HZ,
	};
	i2c_master_dev_handle_t i2c_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &i2c_dev_handle));

	dev->_i2c_bus_handle = i2c_bus_handle;
	dev->_i2c_dev_handle = i2c_dev_handle;
	ESP_LOGI(__FUNCTION__, "New i2c driver installed");
}
