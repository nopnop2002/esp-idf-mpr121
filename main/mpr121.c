/*******************************************************************************

 Bare Conductive MPR121 library
 ------------------------------

 MPR121.cpp - MPR121 class implementation file

 Based on code by Jim Lindblom and plenty of inspiration from the Freescale
 Semiconductor datasheets and application notes.

 Bare Conductive code written by Stefan Dzisiewski-Smith, Peter Krige
 and Szymon Kaliski.

 This work is licensed under a MIT license https://opensource.org/licenses/MIT

 Copyright (c) 2016, Bare Conductive

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mpr121.h"


#define NOT_INITED_BIT 0
#define ADDRESS_UNKNOWN_BIT 1
#define READBACK_FAIL_BIT 2
#define OVERCURRENT_FLAG_BIT 3
#define OUT_OF_RANGE_BIT 4

#define I2C_NUM I2C_NUM_0
//#define I2C_NUM I2C_NUM_1

#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency. no higher than 1MHz for now */

static const char *TAG = "MPR121";

void MPR121_type(MPR121_t * dev){
	//dev->address = 0x5C;		// default address is 0x5C, for use with Bare Conductive Touch Board
	dev->address = 0x5A;		// default address is 0x5A, for use with Bare Conductive Touch Board
	dev->ECR_backup = 0x00;
	dev->running = false;
	dev->error = 1<<NOT_INITED_BIT; // initially, we're not initialised
	dev->touchData = 0;
	dev->lastTouchData = 0;
	dev->autoTouchStatusFlag = false;
}

void MPR121_setRegister(MPR121_t * dev, uint8_t reg, uint8_t value){

	ESP_LOGD(TAG, "setRegister reg=0x%02x value=0x%02x", reg, value);
	bool wasRunning = false;;

	if(reg==MPR121_ECR){	// if we are modding MPR121_ECR, update our internal running status
		if(value&0x3F){
			dev->running = true;
		} else {
			dev->running = false;
		}
	} else if(reg<MPR121_CTL0){
		wasRunning = dev->running;
		if(wasRunning) MPR121_stop(dev);	// we should ALWAYS be in stop mode for this
								// unless modding MPR121_ECR or GPIO / LED register
	}

#if 0
		Wire.beginTransmission(address);
		Wire.write(reg);
		Wire.write(value);
		if(Wire.endTransmission()!=0){
			dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
		} else {
			dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
		}
#endif

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);
	i2c_master_write_byte(cmd, value, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGD(TAG, "setRegister reg=0x%02x value=0x%02x successfully", reg, value);
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
	} else {
		ESP_LOGE(TAG, "setRegister reg=0x%02x value=0x%02x failed. code: 0x%02x", reg, value, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
	i2c_cmd_link_delete(cmd);

	if(wasRunning) MPR121_run(dev);		// restore run mode if necessary
}

uint8_t MPR121_getRegister(MPR121_t * dev, uint8_t reg){
	ESP_LOGD(TAG, "getRegister reg=0x%02x", reg);
	uint8_t scratch = 0;


#if 0
		Wire.beginTransmission(address);
		Wire.write(reg); // set address to read from our requested register
		Wire.endTransmission(false); // repeated start

		if(Wire.requestFrom(address,(uint8_t)1) == 1){	// just a single byte
			dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT); // all good, clear the bit
			scratch = Wire.read();
		} else {
			dev->error |= 1<<ADDRESS_UNKNOWN_BIT; //set the bit - something went wrong
		}
#endif

	uint8_t buf[2];
	memset (buf, 0, 2);

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
	i2c_master_read(cmd, buf, 1, I2C_MASTER_NACK);

	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		dev->error &= ~(1<<ADDRESS_UNKNOWN_BIT);
		scratch = buf[0];
		ESP_LOGD(TAG, "getRegister reg=0x%02x successfully scratch=0x%02x", reg, scratch);
	} else {
		ESP_LOGE(TAG, "getRegister reg=0x%02x failed. code: 0x%02x", reg, espRc);
		dev->error |= 1<<ADDRESS_UNKNOWN_BIT; // set address unknown bit
	}
	i2c_cmd_link_delete(cmd);


	// auto update errors for registers with error data
	if(reg == MPR121_TS2 && ((scratch&0x80)!=0)){
		dev->error |= 1<<OVERCURRENT_FLAG_BIT;
	} else {
		dev->error &= ~(1<<OVERCURRENT_FLAG_BIT);
	}
	if((reg == MPR121_OORS1 || reg == MPR121_OORS2) && (scratch!=0)){
		dev->error |= 1<<OUT_OF_RANGE_BIT;
	} else {
		dev->error &= ~(1<<OUT_OF_RANGE_BIT);
	}
	return scratch;
}



bool MPR121_begin(MPR121_t * dev, int16_t address, int16_t touchThreshold, int16_t releaseThreshold, int16_t interruptPin, int16_t sda, int16_t scl){

	// SDA and SCL should idle high, but MPR121 can get stuck waiting to complete a transaction
	// this code detects this state and releases us from it

#if 0
	//boolean stuck_transaction = false;
	bool stuck_transaction = false;
	uint8_t stuck_transaction_retry_count = 0;
	const uint8_t stuck_transaction_retry_MAX = 10;

	::pinMode( PIN_WIRE_SDA, INPUT_PULLUP );
	::pinMode( PIN_WIRE_SCL, INPUT_PULLUP );

	do{
		if(( ::digitalRead( PIN_WIRE_SDA ) == LOW ) && ( ::digitalRead( PIN_WIRE_SCL ) == HIGH )){
			Serial.println("MPR121_begin stuck_transaction = true");
			stuck_transaction = true;
			::pinMode( PIN_WIRE_SCL, OUTPUT );
			::digitalWrite( PIN_WIRE_SCL, LOW );
			delay( 1 ); // this is way longer than required (would be 1.25us at 400kHz) but err on side of caution
			::pinMode( PIN_WIRE_SCL, INPUT_PULLUP );
			stuck_transaction_retry_count++;
		} else {
			Serial.println("MPR121_begin stuck_transaction = false");
			stuck_transaction = false;
		}
	} while ( stuck_transaction && ( stuck_transaction_retry_count < stuck_transaction_retry_MAX ));

	// TODO: add new error code that can be handled externally
	if( stuck_transaction_retry_count > 0){
		if( stuck_transaction ){
		} else {
		}
	}

	// now we've released (if necessary) we can get on with things
	Wire.begin();
#endif

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

	// addresses only valid 0x5A to 0x5D - if we don't change the address it stays at default
	if(address>=0x5A && address<=0x5D)
	{
		dev->address = address; // need to be specific here
	}

	dev->error &= ~(1<<NOT_INITED_BIT); // clear NOT_INITED error flag

	if( MPR121_reset(dev) ){
		// default values...
		//MPR121_applySettings(dev, dev->defaultSettings );
		MPR121_settingsType(&(dev->defaultSettings));
		MPR121_applySettings(dev);

		// only apply thresholds if they differ from existing defaults
		if( touchThreshold != dev->defaultSettings._TTHRESH ){
			MPR121_setTouchThresholdAll(dev, touchThreshold );
		}

		if( releaseThreshold != dev->defaultSettings._RTHRESH ){
			MPR121_setReleaseThresholdAll(dev, releaseThreshold );
		}

		if( interruptPin != dev->defaultSettings._INTERRUPT ){
			MPR121_setInterruptPin(dev, interruptPin );
		}

		return true;

	} else {
		return false;
	}
}

#if 0
void MPR121_clearSavedThresholds(MPR121_t * dev) {
	#ifdef ARDUINO_ARCH_AVR
		uint8_t maxElectrodes = 12;
		int len = E2END;

		for(uint8_t i=0; i<13; i++){
			EEPROM.write(len - (i + 1), 255);
			EEPROM.write(len - (i + 1) - maxElectrodes, 255);
		}
	#endif
}
#endif

#if 0
void MPR121_restoreSavedThresholds(MPR121_t * dev) {
	#ifdef ARDUINO_ARCH_AVR
		uint8_t maxElectrodes = 12;
		int len = E2END;

		for(uint8_t i=0; i<13; i++){
			uint8_t releaseThreshold = EEPROM.read(len - (i + 1));
			uint8_t touchThreshold = EEPROM.read(len - (i + 1) - maxElectrodes);

			if (touchThreshold < 255) {
				MPR121_setTouchThreshold(dev, i, touchThreshold + 1); // EEPROM values are saved off-by-one
			}
			else {
				MPR121_setTouchThreshold(dev, i, defaultSettings.TTHRESH);
			}

			if (releaseThreshold < 255) {
				MPR121_setReleaseThreshold(dev, i, releaseThreshold + 1); // EEPROM values are saved off-by-one
			}
			else {
				MPR121_setReleaseThreshold(dev, i, defaultSettings.RTHRESH);
			}
		}
	#endif
}
#endif

#if 0
void MPR121_goSlow(MPR121_t * dev){
	Wire.setClock(100000L); // set I2C clock to 100kHz
}

void MPR121_goFast(MPR121_t * dev){
	Wire.setClock(400000L); // set I2C clock to 400kHz
}
#endif

void MPR121_run(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return;
	MPR121_setRegister(dev, MPR121_ECR, dev->ECR_backup); // restore backup to return to run mode
}

void MPR121_stop(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return;
	dev->ECR_backup = MPR121_getRegister(dev, MPR121_ECR); // backup MPR121_ECR to restore when we enter run
	MPR121_setRegister(dev, MPR121_ECR, dev->ECR_backup & 0xC0); // turn off all electrodes to stop
}

bool MPR121_reset(MPR121_t * dev){
	// return true if we successfully reset a device at the
	// address we are expecting

	// MPR121_AFE2 is one of the few registers that defaults to a non-zero value -
	// checking it is sensible as reading back an incorrect value implies
	// something went wrong - we also check MPR121_TS2 bit 7 to see if we have an
	// overcurrent flag set

	MPR121_setRegister(dev, MPR121_SRST, 0x63); // soft reset

	if(MPR121_getRegister(dev, MPR121_AFE2)!=0x24){
		dev->error |= 1<<READBACK_FAIL_BIT;
	} else {
		dev->error &= ~(1<<READBACK_FAIL_BIT);
	}

	if((MPR121_getRegister(dev, MPR121_TS2)&0x80)!=0){
		dev->error |= 1<<OVERCURRENT_FLAG_BIT;
	} else {
		dev->error &= ~(1<<OVERCURRENT_FLAG_BIT);
	}

	if(MPR121_getError(dev)==NOT_INITED || MPR121_getError(dev)==NO_ERROR){ // if our only error is that we are not inited...
		return true;
	} else {
		return false;
	}
	return false;
}

void MPR121_settingsType(MPR121_settings_type *defaultSettings){
	defaultSettings->_TTHRESH=40;
	defaultSettings->_RTHRESH=20;
	defaultSettings->_INTERRUPT=0;		// note that this is not a hardware interrupt, just the digital
										// pin that the MPR121 ~INT pin is connected to
	defaultSettings->_MHDR=0x01;
	defaultSettings->_NHDR=0x01;
	defaultSettings->_NCLR=0x10;
	defaultSettings->_FDLR=0x20;
	defaultSettings->_MHDF=0x01;
	defaultSettings->_NHDF=0x01;
	defaultSettings->_NCLF=0x10;
	defaultSettings->_FDLF=0x20;
	defaultSettings->_NHDT=0x01;
	defaultSettings->_NCLT=0x10;
	defaultSettings->_FDLT=0xFF;
	defaultSettings->_MHDPROXR=0x0F;
	defaultSettings->_NHDPROXR=0x0F;
	defaultSettings->_NCLPROXR=0x00;
	defaultSettings->_FDLPROXR=0x00;
	defaultSettings->_MHDPROXF=0x01;
	defaultSettings->_NHDPROXF=0x01;
	defaultSettings->_NCLPROXF=0xFF;
	defaultSettings->_FDLPROXF=0xFF;
	defaultSettings->_NHDPROXT=0x00;
	defaultSettings->_NCLPROXT=0x00;
	defaultSettings->_FDLPROXT=0x00;
	defaultSettings->_DTR=0x11;
	defaultSettings->_AFE1=0xFF;
	defaultSettings->_AFE2=0x30;
	defaultSettings->_ECR=0xCC; // default to fast baseline startup and 12 electrodes enabled, no prox
	defaultSettings->_ACCR0=0x00;
	defaultSettings->_ACCR1=0x00;
	defaultSettings->_USL=0x00;
	defaultSettings->_LSL=0x00;
	defaultSettings->_TL=0x00;
}


//void MPR121_applySettings(MPR121_t * dev, MPR121_settings_type *settings){
void MPR121_applySettings(MPR121_t * dev){
	bool wasRunning = dev->running;
	if(wasRunning) MPR121_stop(dev);	// can't change most regs when running - checking
							// here avoids multiple stop() / run() calls

	MPR121_setRegister(dev, MPR121_MHDR, dev->defaultSettings._MHDR);
	MPR121_setRegister(dev, MPR121_NHDR, dev->defaultSettings._NHDR);
	MPR121_setRegister(dev, MPR121_NCLR, dev->defaultSettings._NCLR);
	MPR121_setRegister(dev, MPR121_FDLR, dev->defaultSettings._FDLR);
	MPR121_setRegister(dev, MPR121_MHDF, dev->defaultSettings._MHDF);
	MPR121_setRegister(dev, MPR121_NHDF, dev->defaultSettings._NHDF);
	MPR121_setRegister(dev, MPR121_NCLF, dev->defaultSettings._NCLF);
	MPR121_setRegister(dev, MPR121_FDLF, dev->defaultSettings._FDLF);
	MPR121_setRegister(dev, MPR121_NHDT, dev->defaultSettings._NHDT);
	MPR121_setRegister(dev, MPR121_NCLT, dev->defaultSettings._NCLT);
	MPR121_setRegister(dev, MPR121_FDLT, dev->defaultSettings._FDLT);
	MPR121_setRegister(dev, MPR121_MHDPROXR, dev->defaultSettings._MHDPROXR);
	MPR121_setRegister(dev, MPR121_NHDPROXR, dev->defaultSettings._NHDPROXR);
	MPR121_setRegister(dev, MPR121_NCLPROXR, dev->defaultSettings._NCLPROXR);
	MPR121_setRegister(dev, MPR121_FDLPROXR, dev->defaultSettings._FDLPROXR);
	MPR121_setRegister(dev, MPR121_MHDPROXF, dev->defaultSettings._MHDPROXF);
	MPR121_setRegister(dev, MPR121_NHDPROXF, dev->defaultSettings._NHDPROXF);
	MPR121_setRegister(dev, MPR121_NCLPROXF, dev->defaultSettings._NCLPROXF);
	MPR121_setRegister(dev, MPR121_FDLPROXF, dev->defaultSettings._FDLPROXF);
	MPR121_setRegister(dev, MPR121_NHDPROXT, dev->defaultSettings._NHDPROXT);
	MPR121_setRegister(dev, MPR121_NCLPROXT, dev->defaultSettings._NCLPROXT);
	MPR121_setRegister(dev, MPR121_FDLPROXT, dev->defaultSettings._FDLPROXT);
	MPR121_setRegister(dev, MPR121_DTR, dev->defaultSettings._DTR);
	MPR121_setRegister(dev, MPR121_AFE1, dev->defaultSettings._AFE1);
	MPR121_setRegister(dev, MPR121_AFE2, dev->defaultSettings._AFE2);
	MPR121_setRegister(dev, MPR121_ACCR0, dev->defaultSettings._ACCR0);
	MPR121_setRegister(dev, MPR121_ACCR1, dev->defaultSettings._ACCR1);
	MPR121_setRegister(dev, MPR121_USL, dev->defaultSettings._USL);
	MPR121_setRegister(dev, MPR121_LSL, dev->defaultSettings._LSL);
	MPR121_setRegister(dev, MPR121_TL, dev->defaultSettings._TL);

	MPR121_setRegister(dev, MPR121_ECR, dev->defaultSettings._ECR);

	dev->error &= ~(1<<NOT_INITED_BIT); // clear not inited error as we have just inited!
	MPR121_setTouchThresholdAll(dev, dev->defaultSettings._TTHRESH);
	MPR121_setReleaseThresholdAll(dev, dev->defaultSettings._RTHRESH);
	//MPR121_setInterruptPin(dev, dev->defaultSettings._INTERRUPT);

	if(wasRunning) MPR121_run(dev);
}

//mpr121_error_type MPR121_getError(MPR121_t * dev){
uint8_t MPR121_getError(MPR121_t * dev){
	// important - this resets the IRQ pin - as does any I2C comms

	MPR121_getRegister(dev, MPR121_OORS1);	// OOR registers - we may not have read them yet,
	MPR121_getRegister(dev, MPR121_OORS2);	// whereas the other errors should have been caught

	// order of error precedence is determined in this logic block

	if(!MPR121_isInited(dev)) return NOT_INITED; // this has its own checker function

	if((dev->error & (1<<ADDRESS_UNKNOWN_BIT)) != 0){
		return ADDRESS_UNKNOWN;
	} else if((dev->error & (1<<READBACK_FAIL_BIT)) != 0){
		return READBACK_FAIL;
	} else if((dev->error & (1<<OVERCURRENT_FLAG_BIT)) != 0){
		return OVERCURRENT_FLAG;
	} else if((dev->error & (1<<OUT_OF_RANGE_BIT)) != 0){
		return OUT_OF_RANGE;
	} else return NO_ERROR;

}

void MPR121_clearError(MPR121_t * dev){
	dev->error = 0;
}

bool MPR121_isRunning(MPR121_t * dev){
	return dev->running;
}

bool MPR121_isInited(MPR121_t * dev){
	return (dev->error & (1<<NOT_INITED_BIT)) == 0;
}

void MPR121_updateTouchData(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return;

	dev->autoTouchStatusFlag = false;

	dev->lastTouchData = dev->touchData;
	dev->touchData = (unsigned int)MPR121_getRegister(dev, MPR121_TS1) + ((unsigned int)MPR121_getRegister(dev, MPR121_TS2)<<8);
}

bool MPR121_getTouchData(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return false; // avoid out of bounds behaviour

	return((dev->touchData>>electrode)&1);
}

uint8_t MPR121_getNumTouches(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return(0xFF);

	uint8_t scratch = 0;
	for(uint8_t i=0; i<13; i++){
		if(MPR121_getTouchData(dev, i)) scratch++;
	}

	return(scratch);
}

bool MPR121_getLastTouchData(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return false; // avoid out of bounds behaviour

	return((dev->lastTouchData>>electrode)&1);
}

bool MPR121_updateFilteredData(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return(false);

	uint8_t buf[26];
	memset (buf, 0, 26);
	uint8_t LSB, MSB;

	if(MPR121_touchStatusChanged(dev)) {
		dev->autoTouchStatusFlag = true;
	}

  for(int i=0; i<26; i++){ // 13 filtered values
    buf[i] = MPR121_getRegister(dev, MPR121_E0FDL+i);
  }

	for(int i=0; i<13; i++){ // 13 filtered values
		if(MPR121_touchStatusChanged(dev)) {
			dev->autoTouchStatusFlag = true;
		}
		LSB = buf[i*2];
		if(MPR121_touchStatusChanged(dev)) {
			dev->autoTouchStatusFlag = true;
		}
		MSB = buf[i*2+1];
		dev->filteredData[i] = ((MSB << 8) | LSB);
	}

	return true;

#if 0
	uint8_t LSB, MSB;

	Wire.beginTransmission(address);
	Wire.write(MPR121_E0FDL); // set address register to read from the start of the
								//filtered data
	Wire.endTransmission(false); // repeated start

	if(MPR121_touchStatusChanged(dev)) {
		dev->autoTouchStatusFlag = true;
	}

	if(Wire.requestFrom(address,(uint8_t)26)==26){
		for(int i=0; i<13; i++){ // 13 filtered values
			if(MPR121_touchStatusChanged(dev)) {
				dev->autoTouchStatusFlag = true;
			}
			LSB = Wire.read();
			if(MPR121_touchStatusChanged(dev)) {
				dev->autoTouchStatusFlag = true;
			}
			MSB = Wire.read();
			dev->filteredData[i] = ((MSB << 8) | LSB);
		}
		return(true);
	} else {
		// if we don't get back all 26 values we requested, don't update the FDAT values
		// and return false
		return(false);
	}
#endif
}

int MPR121_getFilteredData(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(0xFFFF); // avoid out of bounds behaviour

	return(dev->filteredData[electrode]);
}

bool MPR121_updateBaselineData(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return(false);

	uint8_t buf[13];
	memset (buf, 0, 13);

	if(MPR121_touchStatusChanged(dev)) {
		dev->autoTouchStatusFlag = true;
	}

	for(int i=0; i<13; i++){ // 13 filtered values
		buf[i] = MPR121_getRegister(dev, MPR121_E0BV+i);
		if(MPR121_touchStatusChanged(dev)) {
			dev->autoTouchStatusFlag = true;
		}
		dev->baselineData[i] = buf[i]<<2;
	}

	return true;

#if 0
	Wire.beginTransmission(address);
	Wire.write(MPR121_E0BV);	// set address register to read from the start of the
								// baseline data
	Wire.endTransmission(false); // repeated start

	if(MPR121_touchStatusChanged(dev)) {
		dev->autoTouchStatusFlag = true;
	}

	if(Wire.requestFrom(address,(uint8_t)13)==13){
		for(int i=0; i<13; i++){ // 13 filtered values
			if(MPR121_touchStatusChanged(dev)) {
				dev->autoTouchStatusFlag = true;
			}
			dev->baselineData[i] = Wire.read()<<2;
		}
		return(true);
		} else {
			// if we don't get back all 26 values we requested, don't update the BVAL values
			// and return false
		return(false);
	}
#endif
}

int MPR121_getBaselineData(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(0xFFFF); // avoid out of bounds behaviour

	return(dev->baselineData[electrode]);
}

bool MPR121_isNewTouch(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(false); // avoid out of bounds behaviour
	return((MPR121_getLastTouchData(dev, electrode) == false) && (MPR121_getTouchData(dev, electrode) == true));
}

bool MPR121_isNewRelease(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(false); // avoid out of bounds behaviour
	return((MPR121_getLastTouchData(dev, electrode) == true) && (MPR121_getTouchData(dev, electrode) == false));
}

void MPR121_updateAll(MPR121_t * dev){
	MPR121_updateTouchData(dev);
	MPR121_updateBaselineData(dev);
	MPR121_updateFilteredData(dev);
}

void MPR121_setTouchThresholdAll(MPR121_t * dev, uint8_t val){
	if(!MPR121_isInited(dev)) return;
	bool wasRunning = dev->running;

	if(wasRunning) MPR121_stop(dev);	// can only change thresholds when not running
							// checking here avoids multiple stop() / run()
							// calls

	for(uint8_t i=0; i<13; i++){
		MPR121_setTouchThreshold(dev, i, val);
	}

	if(wasRunning) MPR121_run(dev);
}

#if 0
void MPR121_saveTouchThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val){
	#ifdef ARDUINO_ARCH_AVR
		if(electrode>12 || !MPR121_isInited(dev)) return; // avoid out of bounds behaviour

		MPR121_setTouchThreshold(dev, electrode, val);

		// store to EEPROM
		uint8_t maxElectrodes = 12;
		int len = E2END;
		int addr = len - maxElectrodes - (electrode + 1);
		EEPROM.write(addr, val - 1); // val - 1 so 255 stays as never-written-to
	#endif
}
#endif

void MPR121_setTouchThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val){
	if(electrode>12 || !MPR121_isInited(dev)) return; // avoid out of bounds behaviour

	// this relies on the internal register map of the MPR121
	MPR121_setRegister(dev, MPR121_E0TTH + (electrode<<1), val);
}

void MPR121_setReleaseThresholdAll(MPR121_t * dev, uint8_t val){
	if(!MPR121_isInited(dev)) return;
	bool wasRunning = dev->running;

	if(wasRunning) MPR121_stop(dev);	// can only change thresholds when not running
							// checking here avoids multiple stop / starts

	for(uint8_t i=0; i<13; i++){
		MPR121_setReleaseThreshold(dev, i, val);
	}

	if(wasRunning) MPR121_run(dev);
}

void MPR121_setReleaseThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val){
	if(electrode>12 || !MPR121_isInited(dev)) return; // avoid out of bounds behaviour

	// this relies on the internal register map of the MPR121
	MPR121_setRegister(dev, MPR121_E0RTH + (electrode<<1), val);
}

#if 0
void MPR121_saveReleaseThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val){
	#ifdef ARDUINO_ARCH_AVR
		if(electrode>12 || !MPR121_isInited(dev)) return; // avoid out of bounds behaviour

		MPR121_setReleaseThreshold(dev, electrode, val);

		// store to EEPROM
		int len = E2END;
		int addr = len - (electrode + 1);
		EEPROM.write(addr, val - 1); // val - 1 so 255 stays as never-written-to
	#endif
}
#endif

uint8_t MPR121_getTouchThreshold(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(0xFF); // avoid out of bounds behaviour
	return(MPR121_getRegister(dev, MPR121_E0TTH+(electrode<<1))); // "255" issue is in here somewhere
	//return(101);
}

uint8_t MPR121_getReleaseThreshold(MPR121_t * dev, uint8_t electrode){
	if(electrode>12 || !MPR121_isInited(dev)) return(0xFF); // avoid out of bounds behaviour
	return(MPR121_getRegister(dev, MPR121_E0RTH+(electrode<<1))); // "255" issue is in here somewhere
	//return(51);
}

void MPR121_setInterruptPin(MPR121_t * dev, uint8_t pin){
	// :: here forces the compiler to use Arduino's pinMode, not MPR121's
	if(!MPR121_isInited(dev)) return;
	//::pinMode(pin, INPUT_PULLUP);
	ESP_LOGD(TAG, "setInterruptPin pin=%d", pin);
	gpio_reset_pin(pin);
	gpio_set_direction(pin, GPIO_MODE_INPUT);
	dev->interruptPin = pin;
}

bool MPR121_touchStatusChanged(MPR121_t * dev){
	// :: here forces the compiler to use Arduino's digitalRead, not MPR121's
	//return(dev->autoTouchStatusFlag || (!::digitalRead(dev->interruptPin)));
	return(dev->autoTouchStatusFlag || (!gpio_get_level(dev->interruptPin)));
}

//void MPR121_setProxMode(MPR121_t * dev, mpr121_proxmode_type mode){
void MPR121_setProxMode(MPR121_t * dev, uint8_t mode){

	if(!MPR121_isInited(dev)) return;

	bool wasRunning = dev->running;

	if(wasRunning) MPR121_stop(dev);

	switch(mode){
		case PROX_DISABLED:
			dev->ECR_backup &= ~(3<<4);  // ELEPROX_EN[1:0] = 00
			break;
		case PROX_0_1:
			dev->ECR_backup |=	(1<<4);  // ELEPROX_EN[1:0] = 01
			dev->ECR_backup &= ~(1<<5);
			break;
		case PROX_0_3:
			dev->ECR_backup &= ~(1<<4);  // ELEPROX_EN[1:0] = 10
			dev->ECR_backup |=	(1<<5);
			break;
		case PROX_0_11:
			dev->ECR_backup |=	(3<<4);  // ELEPROX_EN[1:0] = 11
			break;
	}

	if(wasRunning) MPR121_run(dev);
}

//void MPR121_setCalibrationLock(MPR121_t * dev, mpr121_cal_lock_type lock){
void MPR121_setCalibrationLock(MPR121_t * dev, uint8_t lock){

	if(!MPR121_isInited(dev)) return;

	bool wasRunning = dev->running;

	if(wasRunning) MPR121_stop(dev);

	switch(lock){
		case CAL_LOCK_ENABLED:
			dev->ECR_backup &= ~(3<<6);  // CL[1:0] = 00
			break;
		case CAL_LOCK_DISABLED:
			dev->ECR_backup |=	(1<<6);  // CL[1:0] = 01
			dev->ECR_backup &= ~(1<<7);
			break;
		case CAL_LOCK_ENABLED_5_BIT_COPY:
			dev->ECR_backup &= ~(1<<6);  // CL[1:0] = 10
			dev->ECR_backup |=	(1<<7);
			break;
		case CAL_LOCK_ENABLED_10_BIT_COPY:
			dev->ECR_backup |=	(3<<4);  // CL[1:0] = 11
			break;
	}

	if(wasRunning) MPR121_run(dev);
}

void MPR121_setGlobalCDC(MPR121_t * dev, uint8_t CDC){
	if(CDC > 63) return; // current is only valid 0..63uA

	MPR121_setRegister(dev, MPR121_AFE1, (MPR121_getRegister(dev, MPR121_AFE1) & 0xC0) | CDC);
}

void MPR121_setElectrodeCDC(MPR121_t * dev, uint8_t electrode, uint8_t CDC){
	if(CDC > 63 || electrode > 12) return; // current is only valid 0..63uA, electrode only valid 0..12

	MPR121_setRegister(dev, MPR121_CDC0 + electrode, CDC);
}

//void MPR121_setGlobalCDT(MPR121_t * dev, mpr121_CDT_type CDT){
void MPR121_setGlobalCDT(MPR121_t * dev, uint8_t CDT){
	MPR121_setRegister(dev, MPR121_AFE2, (MPR121_getRegister(dev, MPR121_AFE2) & 0x1F) | (CDT << 5));
}

//void MPR121_setElectrodeCDT(MPR121_t * dev, uint8_t electrode, mpr121_CDT_type CDT){
void MPR121_setElectrodeCDT(MPR121_t * dev, uint8_t electrode, uint8_t CDT){
	MPR121_setRegister(dev, MPR121_CDT01 + (electrode >> 1), (MPR121_getRegister(dev, MPR121_CDT01 + (electrode >> 1)) & (0x0F << (((electrode + 1) % 2)<<2))) | (CDT << ((electrode % 2)<<2)));
}

bool MPR121_autoSetElectrodeCDC(MPR121_t * dev, uint8_t electrode, uint16_t VCC_mV){
	uint16_t upper_limit_FDAT = (uint16_t)((((uint32_t)VCC_mV - 700)*256)/VCC_mV) << 2;
	uint16_t target_FDAT = (uint16_t)(((uint32_t)upper_limit_FDAT * 90) / 100);
	uint16_t lower_limit_FDAT = (uint16_t)(((uint32_t)upper_limit_FDAT * 65) / 100);

	uint16_t this_value;
	int16_t last_distance = 0;
	int16_t this_distance = 0;

	const uint8_t max_num_delay_loops = 100;
	uint8_t num_delay_loops;

	bool scratch = false; // default to failure
	uint8_t saved_num_enabled_electrodes = MPR121_getNumEnabledElectrodes(dev);

	MPR121_setNumEnabledElectrodes(dev, electrode + 1); // reducing the number of running electrodes to a minimum speeds things up
	if(!dev->running) MPR121_run(dev);

	for(uint8_t CDC = 1; CDC < 63; CDC ++){
		MPR121_setElectrodeCDC(dev, electrode, CDC);
		num_delay_loops = 0;

		do{
			MPR121_updateFilteredData(dev);
		} while((MPR121_getFilteredData(dev, electrode) == 0) && (num_delay_loops++ < max_num_delay_loops));

		this_value = MPR121_getFilteredData(dev, electrode);

		this_distance = (uint16_t)(abs((int16_t)this_value - (int16_t)target_FDAT)); // TODO: tidy up signed / unsigned types here
		if(CDC > 1){ // only need to see if we need to quit once we have at least two measurements to compare
			if(this_distance > last_distance){ // if we got further away from our target this setting should work (slightly prefer higher values)
				MPR121_setElectrodeCDC(dev, electrode, CDC);
				if((this_value >= lower_limit_FDAT) && (this_value <= upper_limit_FDAT)){
					scratch = true; // success
				}
				break;
			} else if(CDC == 63){ // or if we're at the end of the available adjustment, see if we're close enough
				MPR121_setElectrodeCDC(dev, electrode, CDC);
				if((this_value >= lower_limit_FDAT) && (this_value <= upper_limit_FDAT)){
					scratch = true; // success
				}
				break;
			}
		}
		last_distance = this_distance;
	}

	MPR121_setRegister(dev, MPR121_ECR, dev->ECR_backup);
	MPR121_setNumEnabledElectrodes(dev, saved_num_enabled_electrodes); // have to do this separately as ECR_backup gets invalidated by setNumEnabledElectrodes(electrode + 1);

	return(scratch);
}

bool MPR121_autoSetElectrodeCDCDefault(MPR121_t * dev, uint8_t electrode){
	// default to 3.3V VCC if not explicitly stated
	return(MPR121_autoSetElectrodeCDC(dev, electrode, 3300));
}

bool MPR121_autoSetElectrodeCDCAll(MPR121_t * dev){
	bool scratch = true;
	for(uint8_t i=0; i<MPR121_getNumEnabledElectrodes(dev); i++){
		scratch = MPR121_autoSetElectrodeCDCDefault(dev, i) ? scratch : false;
	}

	return(scratch);
}

bool MPR121_autoSetElectrodes(MPR121_t * dev, uint16_t VCC_mV, bool fixedChargeTime){
	uint8_t USL = (uint8_t)((((uint32_t)VCC_mV - 700)*256)/VCC_mV);
	uint8_t T_L = (uint8_t)(((uint16_t)USL * 90) / 100);
	uint8_t LSL = (uint8_t)(((uint16_t)USL * 65) / 100);
	bool wasRunning = dev->running;

	MPR121_stop(dev);

	MPR121_setRegister(dev, MPR121_USL, USL);
	MPR121_setRegister(dev, MPR121_TL, T_L);
	MPR121_setRegister(dev, MPR121_LSL, LSL);

	// don't enable retry, copy other settings from elsewhere
	MPR121_setRegister(dev, MPR121_ACCR0, 1 | ((dev->ECR_backup & 0xC0) >> 4) | (MPR121_getRegister(dev, MPR121_AFE1) & 0xC0)); 
	// fixed charge time is useful for designs with higher lead-in resistance - e.g. using Bare Electric Paint
	MPR121_setRegister(dev, MPR121_ACCR1, fixedChargeTime ? 1 << 7 : 0); 

	if(wasRunning){
		MPR121_run(dev);
	}

	return(!(MPR121_getRegister(dev, MPR121_OORS2) & 0xC0));
}

bool MPR121_autoSetElectrodesDefault(MPR121_t * dev, bool fixedChargeTime){
	return(MPR121_autoSetElectrodes(dev, 3300, fixedChargeTime));
}

void MPR121_setNumDigPins(MPR121_t * dev, uint8_t numPins){
	if(!MPR121_isInited(dev)) return;
	bool wasRunning = dev->running;

	if(numPins>8) numPins = 8; // maximum number of GPIO pins is 8 out of 12

	if(wasRunning){
		MPR121_stop(dev); // have to stop to change MPR121_ECR
	}
	dev->ECR_backup = (0x0F&(12-numPins)) | (dev->ECR_backup&0xF0);
	if(wasRunning){
		MPR121_run(dev);
	}
}

void MPR121_setNumEnabledElectrodes(MPR121_t * dev, uint8_t numElectrodes){
	if(!MPR121_isInited(dev)) return;
	bool wasRunning = dev->running;

	if(numElectrodes>12) numElectrodes = 12; // avoid out-of-bounds behaviour

	if(wasRunning){
		MPR121_stop(dev); // have to stop to change MPR121_ECR
	}
	dev->ECR_backup = (0x0F&numElectrodes) | (dev->ECR_backup&0xF0);
	if(wasRunning){
		MPR121_run(dev);
	}
}

uint8_t MPR121_getNumEnabledElectrodes(MPR121_t * dev){
	if(!MPR121_isInited(dev)) return(0xFF);

	return(MPR121_getRegister(dev, MPR121_ECR) & 0x0F);
}

#if 0
void MPR121_pinMode(MPR121_t * dev, uint8_t electrode, mpr121_pinf_type mode){
	// only valid for ELE4..ELE11
	if(electrode<4 || electrode >11 || !MPR121_isInited(dev)) return;

	// LED0..LED7
	uint8_t bitmask = 1<<(electrode-4);

	switch(mode){
		case INPUT_PULLDOWN:
			// MPR121_EN = 1
			// MPR121_DIR = 0
			// MPR121_CTL0 = 1
			// MPR121_CTL1 = 0
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) & ~bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) & ~bitmask);
			break;

		case OUTPUT_HIGHSIDE:
			// MPR121_EN = 1
			// MPR121_DIR = 1
			// MPR121_CTL0 = 1
			// MPR121_CTL1 = 1
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) | bitmask);
			break;

		case OUTPUT_LOWSIDE:
			// MPR121_EN = 1
			// MPR121_DIR = 1
			// MPR121_CTL0 = 1
			// MPR121_CTL1 = 0
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) & ~bitmask);
			break;

		default:
			break;
	}
}
#endif

void MPR121_pinMode(MPR121_t * dev, uint8_t electrode, int mode){
	// this is to catch the fact that Arduino prefers its definitions of
	// INPUT, OUTPUT and INPUT_PULLUP to ours...

	// only valid for ELE4..ELE11
	if(electrode<4 || electrode >11 || !MPR121_isInited(dev)) return;

	uint8_t bitmask = 1<<(electrode-4);

	switch(mode){
		case OUTPUT:
			// MPR121_EN = 1
			// MPR121_DIR = 1
			// MPR121_CTL0 = 0
			// MPR121_CTL1 = 0
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) & ~bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) & ~bitmask);
			break;

		case INPUT:
			// MPR121_EN = 1
			// MPR121_DIR = 0
			// MPR121_CTL0 = 0
			// MPR121_CTL1 = 0
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) & ~bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) & ~bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) & ~bitmask);
			break;

		case INPUT_PULLUP:
			// MPR121_EN = 1
			// MPR121_DIR = 0
			// MPR121_CTL0 = 1
			// MPR121_CTL1 = 1
			MPR121_setRegister(dev, MPR121_EN, MPR121_getRegister(dev, MPR121_EN) | bitmask);
			MPR121_setRegister(dev, MPR121_DIR, MPR121_getRegister(dev, MPR121_DIR) & ~bitmask);
			MPR121_setRegister(dev, MPR121_CTL0, MPR121_getRegister(dev, MPR121_CTL0) | bitmask);
			MPR121_setRegister(dev, MPR121_CTL1, MPR121_getRegister(dev, MPR121_CTL1) | bitmask);
			break;

		default:
			break;
	}
}

void MPR121_digitalWrite(MPR121_t * dev, uint8_t electrode, uint8_t val){

	// avoid out of bounds behaviour

	if(electrode<4 || electrode>11 || !MPR121_isInited(dev)) return;

	if(val){
		MPR121_setRegister(dev, MPR121_SET, 1<<(electrode-4));
	} else {
		MPR121_setRegister(dev, MPR121_CLR, 1<<(electrode-4));
	}
}

void MPR121_digitalToggle(MPR121_t * dev, uint8_t electrode){

	// avoid out of bounds behaviour

	if(electrode<4 || electrode>11 || !MPR121_isInited(dev)) return;

	MPR121_setRegister(dev, MPR121_TOG, 1<<(electrode-4));
}

bool MPR121_digitalRead(MPR121_t * dev, uint8_t electrode){

	// avoid out of bounds behaviour

	if(electrode<4 || electrode>11 || !MPR121_isInited(dev)) return false;

	return(((MPR121_getRegister(dev, MPR121_DAT)>>(electrode-4))&1)==1);
}

void MPR121_analogWrite(MPR121_t * dev, uint8_t electrode, uint8_t value){
	// LED output 5 (ELE9) and output 6 (ELE10) have a PWM bug
	// https://community.nxp.com/thread/305474

	// avoid out of bounds behaviour

	if(electrode<4 || electrode>11 || !MPR121_isInited(dev)) return;

	uint8_t shiftedVal = value>>4;

	if(shiftedVal > 0){
		MPR121_setRegister(dev, MPR121_SET, 1<<(electrode-4)); // normal PWM operation
	} else {
		// this make a 0 PWM setting turn off the output
		MPR121_setRegister(dev, MPR121_CLR, 1<<(electrode-4));
	}

	switch(electrode-4){

	case 0:
		MPR121_setRegister(dev, MPR121_PWM0, (shiftedVal & 0x0F) | (MPR121_getRegister(dev, MPR121_PWM0) & 0xF0));
		break;
	case 1:
		MPR121_setRegister(dev, MPR121_PWM0, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(dev, MPR121_PWM0) & 0x0F));
		break;
	case 2:
		MPR121_setRegister(dev, MPR121_PWM1, (shiftedVal & 0x0F) | (MPR121_getRegister(dev, MPR121_PWM1) & 0xF0));
		break;
	case 3:
		MPR121_setRegister(dev, MPR121_PWM1, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(dev, MPR121_PWM1) & 0x0F));
		break;
	case 4:
		MPR121_setRegister(dev, MPR121_PWM2, (shiftedVal & 0x0F) | (MPR121_getRegister(dev, MPR121_PWM2) & 0xF0));
		break;
	case 5:
		MPR121_setRegister(dev, MPR121_PWM2, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(dev, MPR121_PWM2) & 0x0F));
		break;
	case 6:
		MPR121_setRegister(dev, MPR121_PWM3, (shiftedVal & 0x0F) | (MPR121_getRegister(dev, MPR121_PWM3) & 0xF0));
		break;
	case 7:
		MPR121_setRegister(dev, MPR121_PWM3, ((shiftedVal & 0x0F)<<4) | (MPR121_getRegister(dev, MPR121_PWM3) & 0x0F));
		break;
	}
}

//void MPR121_setSamplePeriod(MPR121_t * dev, mpr121_sample_interval_type period){
void MPR121_setSamplePeriod(MPR121_t * dev, uint8_t period){
	MPR121_setRegister(dev, MPR121_AFE2, (MPR121_getRegister(dev, MPR121_AFE2) & 0xF8) | (period & 0x07));
}

//void MPR121_setFFI(MPR121_t * dev, mpr121_FFI_type FFI){
void MPR121_setFFI(MPR121_t * dev, uint8_t FFI){
	MPR121_setRegister(dev, MPR121_AFE1, (MPR121_getRegister(dev, MPR121_AFE1) & 0x3F) | ((FFI & 0x03) << 6));
}

//void MPR121_setSFI(MPR121_t * dev, mpr121_SFI_type SFI){
void MPR121_setSFI(MPR121_t * dev, uint8_t SFI){
	MPR121_setRegister(dev, MPR121_AFE2, (MPR121_getRegister(dev, MPR121_AFE2) & 0xE7) | ((SFI & 0x03) << 3));
}

//MPR121_type MPR121 = MPR121_type();


