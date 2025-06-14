/*******************************************************************************

 Bare Conductive MPR121 library
 ------------------------------

 MPR121.h - MPR121 class header file

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

#ifndef MPR121_H
#define MPR121_H

#include "mpr121_defs.h"


// idea behind this is to create a settings structure that we can use to store
// all the setup variables for a particular setup - comes pre-instantiated with
// defaults and can be easily tweaked - we pass by reference (as a pointer) to
// save RAM

typedef struct
{
	// touch and release thresholds
	uint8_t _TTHRESH;
	uint8_t _RTHRESH;

	uint8_t _INTERRUPT;

	// general electrode touch sense baseline filters
	// rising filter
	uint8_t _MHDR;
	uint8_t _NHDR;
	uint8_t _NCLR;
	uint8_t _FDLR;

	// falling filter
	uint8_t _MHDF;
	uint8_t _NHDF;
	uint8_t _NCLF;
	uint8_t _FDLF;

	// touched filter
	uint8_t _NHDT;
	uint8_t _NCLT;
	uint8_t _FDLT;

	// proximity electrode touch sense baseline filters
	// rising filter
	uint8_t _MHDPROXR;
	uint8_t _NHDPROXR;
	uint8_t _NCLPROXR;
	uint8_t _FDLPROXR;

	// falling filter
	uint8_t _MHDPROXF;
	uint8_t _NHDPROXF;
	uint8_t _NCLPROXF;
	uint8_t _FDLPROXF;

	// touched filter
	uint8_t _NHDPROXT;
	uint8_t _NCLPROXT;
	uint8_t _FDLPROXT;

	// debounce settings
	uint8_t _DTR;

	// configuration registers
	uint8_t _AFE1;
	uint8_t _AFE2;
	uint8_t _ECR;

	// auto-configuration registers
	uint8_t _ACCR0;
	uint8_t _ACCR1;
	uint8_t _USL;
	uint8_t _LSL;
	uint8_t _TL;
} MPR121_settings_type;

typedef struct {
	uint8_t address;
	MPR121_settings_type defaultSettings;
	uint8_t ECR_backup; // so we can re-enable the correct number of electrodes
									// when recovering from stop mode
	uint8_t error;
	bool running;
	uint8_t interruptPin;

	int16_t filteredData[13];
	int16_t baselineData[13];
	uint16_t touchData;
	uint16_t lastTouchData;
	bool autoTouchStatusFlag; // we use this to catch touch / release events that happen
																				// during other update calls
} MPR121_t;


// GPIO pin function constants
enum mpr121_pinf_type
{
	// INPUT and OUTPUT (and others) are already defined by Arduino, use its definitions if they exist
#ifndef INPUT
	INPUT,			// digital input
#endif
#ifndef INPUT_PULLUP
	INPUT_PULLUP,		// digital input with pullup
#endif
#ifndef INPUT_PULLDOWN
	INPUT_PULLDOWN,		// digital input with pulldown
#endif
#ifndef OUTPUT
	OUTPUT,			// digital output (push-pull)
#endif
#ifndef OUTPUT_HIGHSIDE
	OUTPUT_HIGHSIDE,	// digital output, open collector (high side)
#endif
#ifndef OUTPUT_LOWSIDE
	OUTPUT_LOWSIDE		// digital output, open collector (low side)
#endif
};

// "13th electrode" proximity modes
// N.B. this does not relate to normal proximity detection
// see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
enum mpr121_proxmode_type
{
	PROX_DISABLED,		// proximity mode disabled
	PROX_0_1,			// proximity mode for ELE0..ELE1
	PROX_0_3,			// proximity mode for ELE0..ELE3
	PROX_0_11			// proximity mode for ELE0..ELE11
};

// baseline calibration lock modes
enum mpr121_cal_lock_type
{
	CAL_LOCK_ENABLED,				// baseline tracking enabled, no value change on run
	CAL_LOCK_DISABLED,				// baseline tracking disabled
	CAL_LOCK_ENABLED_5_BIT_COPY,	// baseline tracking enabled, load 5 MSB of filtered data on entering run mode
	CAL_LOCK_ENABLED_10_BIT_COPY	// baseline tracking enabled, load 10 MSB of filtered data on entering run mode
};

// error codes
enum mpr121_error_type
{
	NO_ERROR,			// no error
	RETURN_TO_SENDER, // not implemented
	ADDRESS_UNKNOWN,	// no MPR121 found at specified I2C address
	READBACK_FAIL,		// readback from MPR121 was not as expected
	OVERCURRENT_FLAG, // overcurrent on REXT pin
	OUT_OF_RANGE,		// autoconfiguration fail, often a result of shorted pins
	NOT_INITED			// device has not been initialised
};

// sample intervals
enum mpr121_sample_interval_type
{
	SAMPLE_INTERVAL_1MS		= 0x00,
	SAMPLE_INTERVAL_2MS		= 0x01,
	SAMPLE_INTERVAL_4MS		= 0x02,
	SAMPLE_INTERVAL_8MS		= 0x03,
	SAMPLE_INTERVAL_16MS	= 0x04,
	SAMPLE_INTERVAL_32MS	= 0x05,
	SAMPLE_INTERVAL_64MS	= 0x06,
	SAMPLE_INTERVAL_128MS = 0x07
};

// charge / discharge times (CDT)
enum mpr121_CDT_type
{
	CDT_500NS = 0x01,
	CDT_1US		= 0x02,
	CDT_2US		= 0x03,
	CDT_4US		= 0x04,
	CDT_8US		= 0x05,
	CDT_16US	= 0x06,
	CDT_32US	= 0x07
};

// first filter iterations (FFI)
enum mpr121_FFI_type
{
	FFI_6  = 0x00,
	FFI_10 = 0x01,
	FFI_18 = 0x02,
	FFI_34 = 0x03
};

// second filter iterations (SFI)
enum mpr121_SFI_type
{
	SFI_4  = 0x00,
	SFI_6  = 0x01,
	SFI_10 = 0x02,
	SFI_18 = 0x03
};


// -------------------- BASIC FUNCTIONS --------------------

void MPR121_type(MPR121_t * dev);

// begin() must be called before using any other function
// address is optional, default is 0x5C
bool MPR121_begin(MPR121_t * dev, int16_t address, int16_t touchThreshold, int16_t releaseThreshold, int16_t interruptPin, int16_t sda, int16_t scl);

// read touch and release threshold saved to EEPROM using
// saveTouchThreshold and releaseTouchThreshold
void MPR121_restoreSavedThresholds(MPR121_t * dev);

// reset saved thresholds to 255, which will force the MPR121
// to load default thresholds
void MPR121_clearSavedThresholds(MPR121_t * dev);

// I2C speed control functions - goFast() sets the SCL clock
// to 400kHz - goSlow() sets the SCL clock to 100kHz. Defaults
// to 100kHz and affects all devices on the I2C bus. Included
// for speed freaks only.
void MPR121_goSlow(MPR121_t * dev);
void MPR121_goFast(MPR121_t * dev);

// getError() returns an mpr121_error_type indicating the current
// error on the MPR121 - clearError() clears this
uint8_t MPR121_getError(MPR121_t * dev);
void MPR121_clearError(MPR121_t * dev);

// returns status of the MPR121 INT pin as read via digitalRead() on the
// Arduino board - this tells us if there has been a change in touch status
// on any active electrode since we last read any data
bool MPR121_touchStatusChanged(MPR121_t * dev);

// updates the data from the MPR121 into our internal buffer
// updateTouchData() does this only for touch on / off status
// updateBaseLineData() does this for background baseline
// updateFilteredData() does this for continuous proximity data
// updateAll() does all three

// the appropriate function from these must be called before data
// from getTouchData(), getFilteredData() etc. can be considered
// valid
void MPR121_updateTouchData(MPR121_t * dev);
bool MPR121_updateBaselineData(MPR121_t * dev);
bool MPR121_updateFilteredData(MPR121_t * dev);
void MPR121_updateAll(MPR121_t * dev);

// returns a boolean indicating the touch status of a given electrode
bool MPR121_getTouchData(MPR121_t * dev, uint8_t electrode);

// returns the number of touches currently detected
uint8_t MPR121_getNumTouches(MPR121_t * dev);

bool MPR121_getLastTouchData(MPR121_t * dev, uint8_t electrode);

// returns continous proximity or baseline data for a given electrode
int MPR121_getFilteredData(MPR121_t * dev, uint8_t electrode);
int MPR121_getBaselineData(MPR121_t * dev, uint8_t electrode);

// returns boolean indicating whether a new touch or release has been
// detected since the last time updateTouchData() was called
bool MPR121_isNewTouch(MPR121_t * dev, uint8_t electrode);
bool MPR121_isNewRelease(MPR121_t * dev, uint8_t electrode);

// sets touch and release thresholds either for all electrodes, or
// for a specfic electrode - higher values = less sensitive and
// release threshold must ALWAYS be lower than touch threshold
void MPR121_setTouchThresholdAll(MPR121_t * dev, uint8_t val);
void MPR121_setTouchThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val);
void MPR121_saveTouchThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val);

void MPR121_setReleaseThresholdAll(MPR121_t * dev, uint8_t val);
void MPR121_setReleaseThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val);
void MPR121_saveReleaseThreshold(MPR121_t * dev, uint8_t electrode, uint8_t val);

// returns the current touch or release threshold for a specified electrode
uint8_t MPR121_getTouchThreshold(MPR121_t * dev, uint8_t electrode);
uint8_t MPR121_getReleaseThreshold(MPR121_t * dev, uint8_t electrode);

// ------------------ ADVANCED FUNCTIONS ------------------

void MPR121_settingsType(MPR121_settings_type *defaultSettings);

// applies a complete array of settings from an
// MPR121_settings_type variable passed as a pointer
// useful if you want to do a bulk setup of the device
//void MPR121_applySettings(MPR121_t * dev, MPR121_settings_type *settings);
void MPR121_applySettings(MPR121_t * dev);

// setRegister() and getRegister() manipulate registers on
// the MPR121 directly, whilst correctly stopping and
// restarting the MPR121 if necessary
void MPR121_setRegister(MPR121_t * dev, uint8_t reg, uint8_t value);
uint8_t MPR121_getRegister(MPR121_t * dev, uint8_t reg);

// stop() and run() take the MPR121 in and out of stop mode
// which reduces current consumption to 3uA
void MPR121_run(MPR121_t * dev);
void MPR121_stop(MPR121_t * dev);

// resets the MPR121
bool MPR121_reset(MPR121_t * dev);

// tells us if we are in run mode, and if we have inited the
// MPR121
bool MPR121_isRunning(MPR121_t * dev);
bool MPR121_isInited(MPR121_t * dev);

// sets the pin that the MPR121 INT output is connected to on the
// Arduino board - does not have to be a hardware interrupt pin
// if it is, however, an interrupt service routine will automatically
// set an internal flag when a touch event occurs - thus minimising
// lost events if you are also reading other data types (filtered data,
// baseline data)
void MPR121_setInterruptPin(MPR121_t * dev, uint8_t pin);

// set number of electrodes to use to generate virtual "13th"
// proximity electrode
// see http://cache.freescale.com/files/sensors/doc/app_note/AN3893.pdf
//
// N.B. - this is not related to general proximity detection or
// reading back continuous proximity data
void MPR121_setProxMode(MPR121_t * dev, uint8_t mode);

// set calibration lock mode for baseline tracking
// this can be useful to disable baseline tracking, set it to
// current values (with 5 or 10 most significant bits copied across from
// current filtered values to baseline the next time run mode is entered)
// or leave it enabled with its current values in place (and no copy-across
// when run mode is re-entered)
void MPR121_setCalibrationLock(MPR121_t * dev, uint8_t lock);

// set global or per-electrode charge / discharge current in microamps
// current_uA is valid 0..63 (for global value, 0 means use electrode-specific setting)
// electrode is valid 0..12 (includes virtual proximity electrode)
void MPR121_setGlobalCDC(MPR121_t * dev, uint8_t CDC);
void MPR121_setElectrodeCDC(MPR121_t * dev, uint8_t electrode, uint8_t CDC);

// runs hardware routine for automatic electrode calibration for all electrodes
// this is implemented in the MPR121 hardware itself
// fixedChargeTime flag keeps CDT constant (useful for carbon-based inks with high lead-in resistance)
bool MPR121_autoSetElectrodes(MPR121_t * dev, uint16_t VCC_mV, bool fixedChargeTime);
bool MPR121_autoSetElectrodesDefault(MPR121_t * dev, bool fixedChargeTime);

// software routine to keep CDT constant and adjust CDC to fit
bool MPR121_autoSetElectrodeCDC(MPR121_t * dev, uint8_t electrode, uint16_t VCC_mV);
bool MPR121_autoSetElectrodeCDCDefault(MPR121_t * dev, uint8_t electrode);
bool MPR121_autoSetElectrodeCDCAll(MPR121_t * dev); // runs autocal for all electrodes

// set global or per-electrode charge / discharge time
// CDT follows a 0.5(2^(n-1))uS rule, so an enum type is provided for clarity
void MPR121_setGlobalCDT(MPR121_t * dev, uint8_t CDT);
void MPR121_setElectrodeCDT(MPR121_t * dev, uint8_t electrode, uint8_t CDT);

// Set / get the number of enabled electrodes, from 0 (which implicitly enters
// stop mode) up to 12. This allows for a reduction in power consumption
// when using fewer electrodes and faster update rates. Implementation is
// similar to setNumDigPins below, butwith a different intent.
void MPR121_setNumEnabledElectrodes(MPR121_t * dev, uint8_t numElectrodes);
uint8_t MPR121_getNumEnabledElectrodes(MPR121_t * dev);

// Enables GPIO mode for up to 8 of the MPR121 electrodes
// starts with electrode 11 - i.e. setNumDigPins(1) sets just
// electrode 11 as GPIO, setNumDigPins(2) sets electrodes 11
// & 10 as GPIO, and so on. Electrodes 0 to 3 cannot be used
// as GPIO
//
// N.B. electrodes are 3.3V and WILL be damaged if driven by
// a greater voltage
void MPR121_setNumDigPins(MPR121_t * dev, uint8_t numPins);

// Sets pin mode for an electrode already set as GPIO by
// setNumDigPins() - see section "GPIO pin function constants"
// for details
#if 0
void MPR121_pinMode(MPR121_t * dev, uint8_t electrode, mpr121_pinf_type mode);
#endif
void MPR121_pinMode(MPR121_t * dev, uint8_t electrode, int mode);

// Similar to digitalWrite in Arduino for GPIO electrode
void MPR121_digitalWrite(MPR121_t * dev, uint8_t electrode, uint8_t val);

// Toggles electrode set as GPIO output
void MPR121_digitalToggle(MPR121_t * dev, uint8_t electrode);

// Reads electrode set as GPIO input
bool MPR121_digitalRead(MPR121_t * dev, uint8_t electrode);

// Writes PWM value to electrode set as GPIO output - very limited
// (4 bit, although input to function is 0..255 to match Arduino,
// internally reduced to 4 bit) and broken on ELE9 and ELE10
// see https://community.freescale.com/thread/305474
void MPR121_analogWrite(MPR121_t * dev, uint8_t electrode, uint8_t val);

// Sets the sample period of the MPR121 - the time between capacitive
// readings. Higher values consume less power, but are less responsive.
void MPR121_setSamplePeriod(MPR121_t * dev, uint8_t period);

void MPR121_setFFI(MPR121_t * dev, uint8_t FFI);

void MPR121_setSFI(MPR121_t * dev, uint8_t SFI);

//extern MPR121_type MPR121;


#endif // MPR121_H

