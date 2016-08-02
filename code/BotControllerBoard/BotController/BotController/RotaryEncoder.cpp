/* 
* RotaryEncoder.cpp
*
* Created: 23.04.2016 23:13:20
* Author: JochenAlt
*/

#include "Arduino.h"
#include "RotaryEncoder.h"
extern void doI2CPortScan();

void RotaryEncoder::switchConflictingSensor(bool powerOn) {
	if (powerOn) {
		pinMode(I2C_ADDRESS_ADDON_VDD_PIN,OUTPUT);
		pinMode(I2C_ADDRESS_ADDON_GND_PIN,OUTPUT);

		digitalWrite(I2C_ADDRESS_ADDON_VDD_PIN, HIGH);
		digitalWrite(I2C_ADDRESS_ADDON_GND_PIN, LOW);
	} else {
		pinMode(I2C_ADDRESS_ADDON_VDD_PIN,INPUT);
		digitalWrite(I2C_ADDRESS_ADDON_VDD_PIN, LOW); // disable internal pullup

		pinMode(I2C_ADDRESS_ADDON_GND_PIN,INPUT);
		digitalWrite(I2C_ADDRESS_ADDON_GND_PIN, LOW); // disable internal pullup
	}
}


void RotaryEncoder::setup(RotaryEncoderConfig* pConfigData, RotaryEncoderSetupData* pSetupData)
{
	configData = pConfigData;
	setupData = pSetupData;

	passedCheck= false;	
	
	if (logSetup) {
		logger->println(F("setup encoder"));
		configData->print();
		logger->print(F("   "));
		setupData->print();
	}
	
	bool doProgI2CAddr = reprogrammei2CAddress();					// true, if this sensor needs reprogrammed i2c address 
	uint8_t i2cAddress = i2CAddress(false);							// i2c address before reprogramming
	uint8_t proggedI2CAddr = i2cAddress + (I2C_ADDRESS_ADDON<<2);	// i2c address after reprogramming

	if (logSetup) {
		logger->print(F("connecting to I2C 0x"));
		logger->print(i2cAddress, HEX);
		if (doProgI2CAddr) {
			logger->print(F(", reprogramm to 0x"));
			logger->print(proggedI2CAddr, HEX);
		}
		logger->println();
	}

	if (doProgI2CAddr) {
		Wire.beginTransmission(proggedI2CAddr );
		byte error = Wire.endTransmission();
		if (error == 0) {
			if (logSetup)
				logger->println(F("new I2C works already."));
			// new address already set, dont do anything
			doProgI2CAddr = false;			
			sensor.setI2CAddress(proggedI2CAddr);
			sensor.begin(); // restart sensor with new I2C address
			switchConflictingSensor(true /* = power on */);
		}
	} else {
		sensor.setI2CAddress(i2cAddress);
		sensor.begin();		
	}

	//set clock wise counting
	sensor.setClockWise(isClockwise());

	// check communication		
	currentSensorAngle = sensor.angleR(U_DEG, true);
	
	// do we have to reprogramm the I2C address?
	if (doProgI2CAddr) {
		// address reg contains i2c addr bit 0..4, while bit 4 is inverted. This register gives bit 2..6 of i2c address, 0..1 is in hardware pins
		uint8_t i2cAddressReg = sensor.addressRegR();
		// new i2c address out of old address reg is done setting 1. bit, and xor the inverted 4. bit and shifting by 2 (for i2c part in hardware)
		uint8_t newi2cAddress = ((i2cAddressReg+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2; // see datasheet of AS5048B, computation of I2C address 
		if (newi2cAddress != proggedI2CAddr)
			fatalError(F("new I2C address wrong"));

		if (logSetup) {
			logger->println(F("reprogramme."));
			logger->print(F("AddrR(old)=0x"));
			logger->print(i2cAddressReg, HEX);
			logger->print(F("AddrR(new)=0x"));
			logger->print(i2cAddressReg+I2C_ADDRESS_ADDON, HEX);

			logger->print(F(" i2cAddr(new)=0x"));
			logger->println(newi2cAddress, HEX);

		}
		sensor.addressRegW(i2cAddressReg+I2C_ADDRESS_ADDON);		
		sensor.setI2CAddress(newi2cAddress); 
		sensor.begin(); // restart sensor with new I2C address
		
		// check new i2c address
		uint8_t i2cAddressRegCheck = sensor.addressRegR();
		if (i2cAddressRegCheck != (i2cAddressReg+I2C_ADDRESS_ADDON))
			fatalError(F("i2c AddrW failed"));
		else {
			// sensor.doProgCurrI2CAddress();
		}
		
		// now boot the other device with the same i2c address, there is no conflict anymore
		switchConflictingSensor(true /* = power on */);
	}

	// check communication
	communicationWorks = false;
	Wire.beginTransmission(i2CAddress(true));
	byte error = Wire.endTransmission();
	communicationWorks = (error == 0);
	
	currentSensorAngle = 0.0;
	if (communicationWorks) {
		currentSensorAngle = sensor.angleR(U_DEG, true);	
	}

	// Serial.print("current angle=");
	// Serial.println(currentSensorAngle);
} //RotaryEncode


float RotaryEncoder::getAngle() {
	float angle = currentSensorAngle - getNullAngle();
	
	// make angle between -180°..+180°
	if (angle> 180.0)
		angle-=360.0;
	if (angle< -180.0)
		angle+=360.0;

	return angle;
}

void RotaryEncoder::setNullAngle(float rawAngle) {
	configData->nullAngle = rawAngle;
}

float RotaryEncoder::getNullAngle() {
	return configData->nullAngle;
}

float RotaryEncoder::getRawSensorAngle() {
	return currentSensorAngle;
}

bool RotaryEncoder::getNewAngleFromSensor() {
	float rawAngle = sensor.angleR(U_DEG, true); // returns angle between 0..360
	if (rawAngle>180)
		rawAngle -= 360;
	
	currentSensorAngle = rawAngle;
		
	return true;
}


bool RotaryEncoder::fetchSample(bool raw, uint8_t no, float sample[], float& avr, float &variance) {
	avr = 0.;
	for (int check = 0;check<no;check++) {
		if (check > 0) {
			delay(ENCODER_SAMPLE_RATE); // that's not bad, this function is called for calibration only, not during runtime
		}
		getNewAngleFromSensor(); // measure the encoder's angle
		float x;
		if (raw)
			x = getRawSensorAngle();
		else
			x = getAngle();
		sample[check] = x;
		avr += x;
	}
	
	avr = avr/float(no);
	// compute average and variance, and check if values are reasonable;
	variance = 0;
	for ( int check = 0;check<no;check++) {
		float d = sample[check]-avr;
		variance += d*d;
	}
	variance = variance/no;
	return (variance <= ENCODER_CHECK_MAX_VARIANCE);
}

bool RotaryEncoder::fetchSample(bool raw,float& avr, float &variance) {
	float sample[ENCODER_CHECK_NO_OF_SAMPLES];
	bool ok = fetchSample(raw,ENCODER_CHECK_NO_OF_SAMPLES,sample,avr, variance);
	return ok;
}

float RotaryEncoder::checkEncoderVariance() {
	
	// collect samples of all encoders
	float value[ENCODER_CHECK_NO_OF_SAMPLES];
	float avr, variance;
	passedCheck = fetchSample(true,ENCODER_CHECK_NO_OF_SAMPLES,value, avr, variance);

	if (logEncoder) {
		logger->print(F("encoder("));
		printActuator(setupData->id);
		logger->print(")");

		if (!passedCheck) {
			logger->print(" avr=");
			logger->print(avr);
	
			logger->print(F(" var="));
			logger->print(variance);
			logger->print(" not");
		}
		else
			logger->print(" is");
		logger->println(" stable.");
	}
	return variance;
}