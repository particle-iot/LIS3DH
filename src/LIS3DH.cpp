
#include "Particle.h"
#include "LIS3DH.h"
#include <math.h>

// Official project location:
// https://github.com/rickkas7/LIS3DH

#if ACCEL_DEBUG
    static  Logger local_log("accel");
	#define LOGI   local_log.info
	#define LOGE   local_log.error
#else 
    #define LOGI(...) 
    #define LOGE(...)  
#endif

#ifndef MIN
	#define MIN(a, b)      		(((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
	#define MAX(a, b)      		(((a) > (b)) ? (a) : (b))
#endif

#define MEASURE_RANGE   	LIS3DH_RANGE_16_G
#define MEASURE_DIV   		LIS3DH_DIV_16_G

LIS3DHConfig::LIS3DHConfig()
{
}

/**
 * @brief Sets low power wake on move mode
 *
 * @param wakeAxis setting the axis to trigger wakeup event, value are define in enum LIS3DHAxis
 * @param duration trigger event duration, unit is ms
 * @param sampleRate sample frequency in low power mode, value are define in enum LIS3DHRate
 * @param thresholdMode trigger condition, value are define in enum LIS3DHThresMode
 * @param threshold trigger threshold value. float value with unit g
 * @param int_pin interrupt pins output
 */
LIS3DHConfig &LIS3DHConfig::setLowPowerWakeMode(LIS3DHAxis wakeAxis, uint16_t duration, LIS3DHRate sampleRate, LIS3DHThresMode thresholdMode, float threshold, LIS3DHEnableIntPin int_pin, bool active_low)
{	
    uint8_t int_ths = 0;
	uint8_t int_duration = 0;
	uint8_t int_cfg = 0;

	// 250 mg threshold = 16, max value 127, full-range 2g
	if (threshold >= 2) {
		int_ths = 127;
	} else {
		int_ths = threshold / 2 * 127;
	}

	// Set duration
	switch (sampleRate) {
	case LIS3DH_RATE_1_HZ:
		int_duration = duration / 1000;
		break;
	case LIS3DH_RATE_10_HZ:
		int_duration = duration / 100;
		break;
	case LIS3DH_RATE_25_HZ:
		int_duration = duration / 40;
		break;
	case LIS3DH_RATE_50_HZ:
		int_duration = duration / 20;
		break;
	case LIS3DH_RATE_100_HZ:
		int_duration = duration / 10;
		break;
	default:
		int_duration = 0;
		break;
	}

	// Set interrupt source
	switch (wakeAxis) {
	case LIS3DH_AXIS_X:
		if (thresholdMode == LIS3DH_THRES_MODE_CEILING) {
			int_cfg = LIS3DH::INT_CFG_XHIE_XUPE;
		} else {
			int_cfg = LIS3DH::INT_CFG_XLIE_XDOWNE;
		}
		break;
	case LIS3DH_AXIS_Y:
		if (thresholdMode == LIS3DH_THRES_MODE_CEILING) {
			int_cfg = LIS3DH::INT_CFG_YHIE_YUPE;
		} else {
			int_cfg = LIS3DH::INT_CFG_YLIE_YDOWNE;
		}
		break;
	case LIS3DH_AXIS_Z:
		if (thresholdMode == LIS3DH_THRES_MODE_CEILING) {
			int_cfg = LIS3DH::INT_CFG_ZHIE_ZUPE;
		} else {
			int_cfg = LIS3DH::INT_CFG_ZLIE_ZDOWNE;
		}
		break;
	case LIS3DH_AXIS_ALL:
	default:
		if (thresholdMode == LIS3DH_THRES_MODE_CEILING) {
			int_cfg = LIS3DH::INT_CFG_ZHIE_ZUPE | LIS3DH::INT_CFG_YHIE_YUPE | LIS3DH::INT_CFG_XHIE_XUPE;
		} else {
			int_cfg = LIS3DH::INT_CFG_ZLIE_ZDOWNE | LIS3DH::INT_CFG_YLIE_YDOWNE | LIS3DH::INT_CFG_XLIE_XDOWNE;
		}
		break;
	}

	// set sample rate, enable low power with XYZ detection enabled
	reg1 = sampleRate | LIS3DH::CTRL_REG1_LPEN | LIS3DH::CTRL_REG1_ZEN | LIS3DH::CTRL_REG1_YEN | LIS3DH::CTRL_REG1_XEN;

	// Enable high-pass filter
	reg2 = LIS3DH::CTRL_REG2_FDS | LIS3DH::CTRL_REG2_HPIS1;
    
	// Choose full scale, disable high-resolution
	reg4 = MEASURE_RANGE;

    if(int_pin == LIS3DH_ENABLE_INT1_PIN)
	{
        // Enable INT1
	    reg3 = LIS3DH::CTRL_REG3_I1_INT1;
        // Disable INT2
        reg6 = active_low ? LIS3DH::CTRL_REG6_INT_POLARITY : 0;
        // Disable FIFO, enable latch interrupt on INT1_SRC
	    reg5 = LIS3DH::CTRL_REG5_LIR_INT1;

        int1_ths      = int_ths;
        int1_duration = int_duration;
        int1_cfg      = int_cfg;
        int2_ths      = 0;
        int2_duration = 0;
        int2_cfg      = 0;
        int_pin       = LIS3DH_ENABLE_INT1_PIN;
    }
    else
    {
        // Disable INT1
	    reg3 = 0;
        // Enable INT2
        reg6 = LIS3DH::CTRL_REG6_I2_INT2 | (active_low ? LIS3DH::CTRL_REG6_INT_POLARITY : 0);
        // Disable FIFO, enable latch interrupt on INT2_SRC
	    reg5 = LIS3DH::CTRL_REG5_LIR_INT2;

        int1_ths      = 0;
        int1_duration = 0;
        int1_cfg      = 0;
        int2_ths      = int_ths;
        int2_duration = int_duration;
        int2_cfg      = int_cfg;
        int_pin       = LIS3DH_ENABLE_INT2_PIN;
    }

	return *this;
}

/**
 * @brief Sets continuous acceleration monitoring mode
 *
 * @param rate The sampling rate. Values are setting in enum LIS3DHRate
 */
LIS3DHConfig &LIS3DHConfig::setSampleRate(LIS3DHRate sampleRate)
{
	// Enable specified rate, with XYZ detection enabled
	reg1 = sampleRate | LIS3DH::CTRL_REG1_ZEN | LIS3DH::CTRL_REG1_YEN | LIS3DH::CTRL_REG1_XEN;

	// Choose full scale, enable high-resolution
	reg4 = MEASURE_RANGE | LIS3DH::CTRL_REG4_HR;

	return *this;
}

/**
 * @brief Sets orientation interrupt mode
 *
 * This interrupts when the orientation changes.
 *
 * @param movementThreshold Lower values are more sensitive. A common value is 16.
 */
LIS3DHConfig &LIS3DHConfig::setPositionInterrupt(uint8_t threshold, LIS3DHEnableIntPin int_pin, bool active_low)
{

	uint8_t int_ths = 0;
	uint8_t int_cfg = 0;


	int_ths = threshold;

	// For position detection, enable both AOI and 6D
	int_cfg = LIS3DH::INT_CFG_AOI | LIS3DH::INT_CFG_6D |
	           LIS3DH::INT_CFG_ZHIE_ZUPE | LIS3DH::INT_CFG_ZLIE_ZDOWNE |
	           LIS3DH::INT_CFG_YHIE_YUPE | LIS3DH::INT_CFG_YLIE_YDOWNE |
	           LIS3DH::INT_CFG_XHIE_XUPE | LIS3DH::INT_CFG_XLIE_XDOWNE;

	// Enable specified rate, with XYZ detection enabled
	reg1 = LIS3DH_RATE_100_HZ | LIS3DH::CTRL_REG1_ZEN | LIS3DH::CTRL_REG1_YEN | LIS3DH::CTRL_REG1_XEN;

    if(int_pin == LIS3DH_ENABLE_INT1_PIN)
	{
	    // Enable INT1
	    reg3 = LIS3DH::CTRL_REG3_I1_INT1;
        // Disable INT2
        reg6 = active_low ? LIS3DH::CTRL_REG6_INT_POLARITY : 0;

        int1_ths      = int_ths;
        int1_cfg      = int_cfg;
        int1_duration = 0;
        int2_ths      = 0;
        int2_cfg      = 0;
        int2_duration = 0;
        int_pin       = LIS3DH_ENABLE_INT1_PIN;
    }
    else
    {
        // Disable INT1
	    reg3 = 0;
        // Enable INT2
        reg6 = LIS3DH::CTRL_REG6_I2_INT2 | (active_low ? LIS3DH::CTRL_REG6_INT_POLARITY : 0);

        int1_ths      = 0;
        int1_cfg      = 0;
        int1_duration = 0;
        int2_ths      = int_ths;
        int2_cfg      = int_cfg;
        int2_duration = 0;
        int_pin       = LIS3DH_ENABLE_INT2_PIN;
    }

	return *this;
}


LIS3DH::LIS3DH(int intPin) : intPin(intPin) {

}


LIS3DH::~LIS3DH() {

}


bool LIS3DH::hasDevice() {
	bool found = false;
	for(int tries = 0; tries < 10; tries++) {
		uint8_t whoami = readRegister8(REG_WHO_AM_I);
		if (whoami == WHO_AM_I) {
			found = true;
			break;
		}
		delay(1);
	}
	return found;
}

void LIS3DH::on(float movingTh, float crashTh)
{
	movingThreshold = movingTh;
	crashThreshold = crashTh;
	on();
}

void LIS3DH::on(void)
{
	setup(config);

	updated = false;
	sensorActive = true;
	measInit();
	event.init();
}

void LIS3DH::off(void)
{
	setupSample(0, 0);
	setup(config);

	updated = false;
	sensorActive = false;
	measInit();
	event.init();

}

void LIS3DH::measInit(void)
{
	measAvg.x	= 0;
	measAvg.y	= 0;
	measAvg.z	= 0;
	measAvg.mag	= 0;
	measMax.x	= -999999;
	measMax.y	= -999999;
	measMax.z	= -999999;
	measMax.mag	= -999999;
	measMin.x	= 0;
	measMin.y	= 0;
	measMin.z	= 0;
	measMin.mag	= 0;
	count = 0;
}

bool LIS3DH::setWakeMode(LIS3DHAxis wakeAxis, uint16_t duration, uint16_t sampleRate, LIS3DHThresMode thresholdMode, float threshold, LIS3DHEnableIntPin int_pin, bool int_active_low)
{
	bool err = false;
	LIS3DHRate rateConfig = LIS3DH_RATE_POWERDOWN;

	switch (sampleRate) {
	case 1:
		rateConfig = LIS3DH_RATE_1_HZ;
		break;
	case 10:
		rateConfig = LIS3DH_RATE_10_HZ;
		break;
	case 25:
		rateConfig = LIS3DH_RATE_25_HZ;
		break;
	case 50:
		rateConfig = LIS3DH_RATE_50_HZ;
		break;
	case 100:
		rateConfig = LIS3DH_RATE_100_HZ;
		break;
	case 200:
		rateConfig = LIS3DH_RATE_200_HZ;
		break;
	case 400:
		rateConfig = LIS3DH_RATE_400_HZ;
		break;
	case 1600:
		rateConfig = LIS3DH_RATE_LOWPOWER_1K6HZ;
		break;
	case 5000:
		rateConfig = LIS3DH_RATE_LOWPOWER_5KHZ;
		break;
	default:
		err = true;
		break;
	}

	wakeConfig.setLowPowerWakeMode(wakeAxis, duration, rateConfig, thresholdMode, threshold, int_pin, int_active_low);
	setup(wakeConfig);
	sensorActive = false;
	return err;
}

bool LIS3DH::setupSample(uint16_t duration, uint16_t sampleRate)
{
	bool err = false;
	LIS3DHRate rateConfig = LIS3DH_RATE_POWERDOWN;

	readDuration = duration;
	switch (sampleRate) {
	case 0:
		rateConfig = LIS3DH_RATE_POWERDOWN;
		filterSize = 0;
		break;
	case 1:
		rateConfig = LIS3DH_RATE_1_HZ;
		filterSize = (uint16_t)(duration / 1000.0);
		break;
	case 10:
		rateConfig = LIS3DH_RATE_10_HZ;
		filterSize = (uint16_t)(duration / 100.0);
		break;
	case 25:
		rateConfig = LIS3DH_RATE_25_HZ;
		filterSize = (uint16_t)(duration / 40.0);
		break;
	case 50:
		rateConfig = LIS3DH_RATE_50_HZ;
		filterSize = (uint16_t)(duration / 20.0);
		break;
	case 100:
		rateConfig = LIS3DH_RATE_100_HZ;
		filterSize = (uint16_t)(duration / 10.0);
		break;
	case 200:
		rateConfig = LIS3DH_RATE_200_HZ;
		filterSize = (uint16_t)(duration / 5.0);
		break;
	case 400:
		rateConfig = LIS3DH_RATE_400_HZ;
		filterSize = (uint16_t)(duration / 2.5);
		break;
	default:
		err = true;
		filterSize = 0;
		break;
	}

	config.setSampleRate(rateConfig);
	return err;
}

void LIS3DH::setupMode(LIS3DHReadMode mode)
{
	readMode = mode;
}

float LIS3DH::read(LIS3DHAxis axis)
{
	float value = 0;
	LIS3DHRead *output;

	switch (readMode) {
	case LIS3DH_READ_MAX:
		output = &readMax;
		break;
	case LIS3DH_READ_MIN:
		output = &readMin;
		break;
	default:
	case LIS3DH_READ_AVERAGE:
		output = &readAvg;
		break;
	}

	switch (axis) {
	case LIS3DH_AXIS_X:
		value = output->x;
		break;
	case LIS3DH_AXIS_Y:
		value = output->y;
		break;
	case LIS3DH_AXIS_Z:
		value = output->z;
		break;
	default:
		value = 0;
		break;
	}

	updated = false;
	return (sensorActive == true ? value : 0);
}

bool LIS3DH::readSensor(LIS3DHRead &data)
{
	bool hasData = false;
	LIS3DHSample sample;
	LIS3DHDivider div;

	if (sensorActive && getSample(sample)) {
		hasData = true;
		switch (MEASURE_RANGE) {
		case LIS3DH_RANGE_2_G  :
			div = LIS3DH_DIV_2_G;
			break;
		case LIS3DH_RANGE_4_G  :
			div = LIS3DH_DIV_4_G;
			break;
		case LIS3DH_RANGE_8_G  :
			div = LIS3DH_DIV_8_G;
			break;
		case LIS3DH_RANGE_16_G :
			div = LIS3DH_DIV_16_G;
			break;
		}

		data.x = (float)sample.x / div;
		data.y = (float)sample.y / div;
		data.z = (float)sample.z / div;
	} else {
		data.x = 0;
		data.y = 0;
		data.z = 0;
	}

	return hasData;
}

float LIS3DH::readMagnitude(void)
{
	LIS3DHRead *output;

	switch (readMode) {
	case LIS3DH_READ_MAX:
		output = &readMax;
		break;
	case LIS3DH_READ_MIN:
		output = &readMin;
		break;
	default:
	case LIS3DH_READ_AVERAGE:
		output = &readAvg;
		break;
	}

	return (sensorActive == true ? output->mag : 0);
}

// detect moving & crash event
void LIS3DH::updateEvent(float magnitude)
{
	event.moving = magnitude >= (1 + movingThreshold) || magnitude <= (1 - movingThreshold) ? true : false;
	event.crash  = magnitude >= crashThreshold  ? true : false;
	event.updated = event.moving || event.crash;
	event.valid = true;
	if (event.updated) {
		LOGI("new event: accel=%f, moving-%d, crash-%d", magnitude, event.moving ? 1 : 0, event.crash ? 1 : 0);
	}
}

// bit0-moving event, bit1-crash event
uint8_t LIS3DH::readEvent(void)
{
	uint8_t newEvent = 0;
	if (event.updated) {
		newEvent |= event.moving ? 0x01 : 0;
		newEvent |= event.crash  ? 0x02 : 0;
	}
	event.updated = false;
	return newEvent;
}

// accel sensor process thread
void LIS3DH::updateAccel(void)
{
	LIS3DHSample  readData;
	float readMag;

	if (sensorActive && getSample(readData)) {
		count++;
		readMag = sqrt((readData.x * readData.x) + (readData.y * readData.y) + (readData.z * readData.z));

		// update moving & crash event
		updateEvent(readMag / MEASURE_DIV);
		// update maximum value
		measMax.x   = MAX(readData.x, measMax.x);
		measMax.y   = MAX(readData.y, measMax.y);
		measMax.z   = MAX(readData.z, measMax.z);
		measMax.mag = MAX(readMag,    measMax.mag);
		// update maximum value
		measMin.x   = MIN(readData.x, measMin.x);
		measMin.y   = MIN(readData.y, measMin.y);
		measMin.z   = MIN(readData.z, measMin.z);
		measMin.mag = MIN(readMag,    measMin.mag);
		// update average value
		measAvg.x   += readData.x;
		measAvg.y   += readData.y;
		measAvg.z   += readData.z;
		measAvg.mag += readMag;

		if (count >= filterSize) {
			// update to output struct
			readAvg.x   = measAvg.x   / (count * MEASURE_DIV);
			readAvg.y   = measAvg.y   / (count * MEASURE_DIV);
			readAvg.z   = measAvg.z   / (count * MEASURE_DIV);
			readAvg.mag = measAvg.mag / (count * MEASURE_DIV);
			readMax.x   = measMax.x   / MEASURE_DIV;
			readMax.y   = measMax.y   / MEASURE_DIV;
			readMax.z   = measMax.z   / MEASURE_DIV;
			readMax.mag = measMax.mag / MEASURE_DIV;
			readMin.x   = measMin.x   / MEASURE_DIV;
			readMin.y   = measMin.y   / MEASURE_DIV;
			readMin.z   = measMin.z   / MEASURE_DIV;
			readMin.mag = measMin.mag / MEASURE_DIV;
			updated = true;
			measInit();
		}
	}
}

bool LIS3DH::setup(LIS3DHConfig &config)
{
	if (!hasDevice()) {
		LOGE("LIS3DH not found!");
		return false;
	}

	writeRegister8(REG_CTRL_REG1, config.reg1);
	writeRegister8(REG_CTRL_REG2, config.reg2);
	writeRegister8(REG_CTRL_REG3, config.reg3);
	writeRegister8(REG_CTRL_REG4, config.reg4);
	writeRegister8(REG_CTRL_REG5, config.reg5);
	writeRegister8(REG_CTRL_REG6, config.reg6);

	if (config.setReference) {
		// In normal mode, reading the reference register sets it for the current normal force
		// (the normal force of gravity acting on the device)
		readRegister8(REG_REFERENCE);
	}
	// Set FIFO mode
	writeRegister8(REG_FIFO_CTRL_REG, config.fifoCtrlReg);



	if ((config.reg3 & CTRL_REG3_I1_INT1) != 0) {

		writeRegister8(REG_INT1_THS, config.int1_ths);
		writeRegister8(REG_INT1_DURATION, config.int1_duration);

		if (intPin >= 0) {
			// There are instructions to set the INT1_CFG in a loop in the appnote on page 24. As far
			// as I can tell this never works. Merely setting the INT1_CFG does not ever generate an
			// interrupt for me.

			// Remember the INT1_CFG setting because we're apparently supposed to set it again after
			// clearing an interrupt.
			int1_cfg = config.int1_cfg;
			writeRegister8(REG_INT1_CFG, int1_cfg);

			// Clear the interrupt just in case
			readRegister8(REG_INT1_SRC);
		}
		else {
			int1_cfg = 0;
			writeRegister8(REG_INT1_CFG, 0);
		}
	}

	if ((config.reg6 & CTRL_REG6_I2_INT2) != 0) {

		writeRegister8(REG_INT2_THS, config.int2_ths);
		writeRegister8(REG_INT2_DURATION, config.int2_duration);

        // Remember the INT2_CFG setting because we're apparently supposed to set it again after
        // clearing an interrupt.
        int2_cfg = config.int2_cfg;
        writeRegister8(REG_INT2_CFG, int2_cfg);

        // Clear the interrupt just in case
        readRegister8(REG_INT2_SRC);
	}

    enable_int_pin = config.int_pin;

	return true;
}


bool LIS3DH::calibrateFilter(unsigned long stationaryTime, unsigned long maxWaitTime) {
	bool ready = false;

	unsigned long start = millis();
	unsigned long lastMovement = start;
	unsigned long lastRecalibrate = start - RECALIBRATION_MOVEMENT_DELAY;

	while (maxWaitTime == 0 || millis() - start < maxWaitTime) {
		uint8_t int_src = readRegister8(REG_INT1_SRC);
		if ((int_src & INT_SRC_IA) != 0) {
			LOGI("resetting lastMovement int_src=0x%x", int_src);
			lastMovement = lastRecalibrate = millis();
			clearInterrupt();
		}

		if (lastRecalibrate != 0 && millis() - lastRecalibrate >= RECALIBRATION_MOVEMENT_DELAY) {
			LOGI("recalibrating");
			lastRecalibrate = 0;
			readRegister8(REG_REFERENCE);
			clearInterrupt();
		}

		if (millis() - lastMovement >= stationaryTime) {
			ready = true;
			break;
		}
	}

	return ready;
}

uint8_t LIS3DH::clearInterrupt() {
	uint8_t int_src = readRegister8(REG_INT1_SRC);
	writeRegister8(REG_INT1_CFG, int1_cfg);
	LOGI("start clear interrupt flag");

	if (intPin >= 0) {
		while(digitalRead(intPin) == HIGH) {
			delay(10);
			readRegister8(REG_INT1_SRC);
			writeRegister8(REG_INT1_CFG, int1_cfg);
		}
	}

    readRegister8(REG_INT2_SRC);
    writeRegister8(REG_INT2_CFG, int2_cfg);
	LOGI("finish clear interrupt flag");
	return int_src;
}

bool LIS3DH::readInterrupt(LIS3DHEnableIntPin int_pin)
{
    uint8_t  int_src = 0;
    if(int_pin == LIS3DH_ENABLE_INT1_PIN)
    {
	    int_src = readRegister8(REG_INT1_SRC);
    }
    else
    {
	    int_src = readRegister8(REG_INT2_SRC);
    }
    return (int_src & 0x40) == 0 ? false : true;
}

void LIS3DH::enableTemperature(boolean enable)
{
	writeRegister8(REG_TEMP_CFG_REG, enable ? (TEMP_CFG_TEMP_EN | TEMP_CFG_ADC_PD) : 0);
}


int16_t LIS3DH::getTemperature() {

	// https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32F%20MEMS%20%20iNEMO/flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32F%20MEMS%20%20iNEMO%2FLIS3DH%20temperature%20sensor&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580003E26E7DD54228C428E8F9FB5EE9C5185&currentviews=2886

	int16_t result = ((int16_t) readRegister16(REG_OUT_ADC3_L)) / 256;

	return result;
}


bool LIS3DH::getSample(LIS3DHSample &sample) {
	uint8_t statusAuxReg = readRegister8(REG_STATUS_AUX);

	bool hasData = ((statusAuxReg & STATUS_AUX_321DA) != 0);
	//LOGI("fifoSrcReg=0x%02x", fifoSrcReg);

	if (hasData) {
		uint8_t resp[6];
		readData(REG_OUT_X_L, resp, sizeof(resp));

		sample.x = (int16_t) (resp[0] | (((uint16_t)resp[1]) << 8));
		sample.y = (int16_t) (resp[2] | (((uint16_t)resp[3]) << 8));
		sample.z = (int16_t) (resp[4] | (((uint16_t)resp[5]) << 8));
	}
	return hasData;
}

uint8_t LIS3DH::readPositionInterrupt(LIS3DHEnableIntPin int_pin)
{
	uint8_t pos = 0;
	uint8_t int_src = 0;
    
    if(int_pin== LIS3DH_ENABLE_INT1_PIN)
    {   
       int_src = readRegister8(REG_INT1_SRC);
    }
    else
    {
        int_src = readRegister8(REG_INT2_SRC);
    }
    
	if (int_src & INT_SRC_IA) {
		// Clear the IA bit so we only have to test the XYZ flags
		int_src &= ~INT_SRC_IA;

		// See page 28 of the Application Note AN3308 for more information.
		if (int_src == INT_SRC_YL) {
			pos = 1; // case a
		}
		else
		if (int_src == INT_SRC_XH) {
			pos = 2; // case b
		}
		else
		if (int_src == INT_SRC_XL) {
			pos = 3; // case c
		}
		else
		if (int_src == INT_SRC_YH) {
			pos = 4; // case d
		}
		else
		if (int_src == INT_SRC_ZH) {
			pos = 5; // case e - normal case sitting flat
		}
		else
		if (int_src == INT_SRC_ZL) {
			pos = 6; // case f - upside down
		}
	}

	return pos;
}


uint8_t LIS3DH::readRegister8(uint8_t addr) {

	uint8_t resp[1];
	readData(addr, resp, sizeof(resp));

	return resp[0];
}

uint16_t LIS3DH::readRegister16(uint8_t addr) {

	uint8_t resp[2];
	readData(addr, resp, sizeof(resp));

	return resp[0] | (((uint16_t)resp[1]) << 8);
}


void LIS3DH::writeRegister8(uint8_t addr, uint8_t value) {
	// LOGI("writeRegister addr=%02x value=%02x", addr, value);
	uint8_t req[1];
	req[0] = value;

	writeData(addr, req, sizeof(req));
}

void LIS3DH::writeRegister16(uint8_t addr, uint16_t value) {
	// LOGI("writeRegister addr=%02x value=%04x", addr, value);

	uint8_t req[2];
	req[0] = value & 0xff;
	req[1] = value >> 8;

	writeData(addr, req, sizeof(req));
}


//
//
//

LIS3DHSPI::LIS3DHSPI(SPIClass &spi, int ss, int intPin, int speed) : LIS3DH(intPin), spi(spi), ss(ss), spiSettings(speed * MHZ, MSBFIRST, SPI_MODE0) {
}

LIS3DHSPI::~LIS3DHSPI() {
}

bool LIS3DHSPI::hasDevice() {
	spi.begin(ss);

	return LIS3DH::hasDevice();
}


void LIS3DHSPI::beginTransaction() {
	spi.beginTransaction(spiSettings);

	digitalWrite(ss, LOW);
}

void LIS3DHSPI::endTransaction() {
	digitalWrite(ss, HIGH);

	spi.endTransaction();
}

bool LIS3DHSPI::readData(uint8_t addr, uint8_t *buf, size_t numBytes) {
	beginTransaction();

	if (numBytes > 1) {
		addr |= SPI_INCREMENT;
	}

	spi.transfer(SPI_READ | addr);

	for(size_t ii = 0; ii < numBytes; ii++) {
		buf[ii] = spi.transfer(0);
	}

	endTransaction();

	return true;
}

bool LIS3DHSPI::writeData(uint8_t addr, const uint8_t *buf, size_t numBytes) {
	beginTransaction();

	if (numBytes > 1) {
		addr |= SPI_INCREMENT;
	}

	spi.transfer(addr);
	for(size_t ii = 0; ii < numBytes; ii++) {
		spi.transfer(buf[ii]);
	}

	endTransaction();

	return true;
}

//
//
//

LIS3DHI2C::LIS3DHI2C(TwoWire &wire, uint8_t sad0, int intPin) : LIS3DH(intPin), wire(wire), sad0(sad0) {

}

LIS3DHI2C::~LIS3DHI2C() {

}

LIS3DHI2C::LIS3DHI2C(uint8_t sad0, int intPin) : LIS3DH(intPin), wire(Wire), sad0(sad0) {

}

bool LIS3DHI2C::readData(uint8_t addr, uint8_t *buf, size_t numBytes) {
	wire.beginTransmission(getI2CAddr());

	if (numBytes > 1) {
		addr |= I2C_INCREMENT;
	}
	wire.write(addr);

	uint8_t res = wire.endTransmission();
	if (res != 0) {
		return false;
	}

	wire.requestFrom(getI2CAddr(), (uint8_t)numBytes);
	for (size_t ii = 0; ii < numBytes && wire.available(); ii++) {
		buf[ii] = wire.read();
	}
	return true;
}

bool LIS3DHI2C::writeData(uint8_t addr, const uint8_t *buf, size_t numBytes) {

	wire.beginTransmission(getI2CAddr());

	if (numBytes > 1) {
		addr |= I2C_INCREMENT;
	}
	wire.write(addr);
	for(size_t ii = 0; ii < numBytes; ii++) {
		wire.write(buf[ii]);
	}

	uint8_t res = wire.endTransmission();

	return (res == 0);
}

uint8_t LIS3DHI2C::getI2CAddr() const {
	uint8_t addr = (0b0011000 | sad0);

	return addr;
}


