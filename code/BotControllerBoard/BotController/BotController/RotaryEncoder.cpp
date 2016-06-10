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


void RotaryEncoder::setup(RotaryEncoderConfig& pConfigData, RotaryEncoderSetupData& pSetupData)
{
	configData = &pConfigData;
	setupData = &pSetupData;

	passedCheck= false;	
	Serial.print("setup encoder");
	configData->print();
	setupData->print();
	
	bool reprogrammeI2CAddress = reprogrammei2CAddress();
	if (reprogrammeI2CAddress) {
		uint8_t reprogrammedi2cAddress = i2CAddress(false) + (I2C_ADDRESS_ADDON<<2);

		Wire.beginTransmission(reprogrammedi2cAddress );
		byte error = Wire.endTransmission();
		if (error == 0) {
			Serial.print(F("I2C address amended"));
			Serial.println(reprogrammedi2cAddress,HEX);
			
			reprogrammeI2CAddress = false;
			
			sensor.setI2CAddress(reprogrammedi2cAddress);
			Serial.print("set I2C to ");
			Serial.println(reprogrammedi2cAddress,HEX);


			sensor.begin(); // restart sensor with new I2C address
			switchConflictingSensor(true /* = power on */);
		}
	} else {
		//init AMS_AS5048B object
		uint8_t i2cAddress = i2CAddress(false);
		sensor.setI2CAddress(i2cAddress);
		sensor.begin();		
	}

	//set clock wise counting
	sensor.setClockWise(isClockwise());

	// check communication		
	currentSensorAngle = sensor.angleR(U_DEG, true);
	
	// do we have to reprogramm the I2C address?

	if (reprogrammeI2CAddress) {
		Serial.println("repogramm I2c address");
		// reprogramm I2C address of this sensor by register programming
		uint8_t i2cAddress = sensor.addressRegR();	
		sensor.addressRegW(i2cAddress+I2C_ADDRESS_ADDON);		
		sensor.setI2CAddress(((i2cAddress+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2); // see datasheet of AS5048B, computation of I2C address 
		Serial.print("reprogramm I2C address to 0x");
		Serial.println(((i2cAddress+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2,HEX);
		sensor.begin(); // restart sensor with new I2C address
		
		// now boot the device with the same i2c address, there is no conflict anymore
		switchConflictingSensor(true /* = power on */);
	}
	Serial.println("reading angle");

	currentSensorAngle = sensor.angleR(U_DEG, true);
	Serial.print("current angle=");
	Serial.println(currentSensorAngle);
} //RotaryEncode


float RotaryEncoder::getAngle() {
	float angle = currentSensorAngle - getNullAngle();
	
	// make angle between -180°..+180°
	while (angle < -180.0)
		angle += 360.0;
	while (angle > 180.0)
		angle -= 360.0;
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

void RotaryEncoder::getNewAngleFromSensor() {
	currentSensorAngle = sensor.angleR(U_DEG, true);
	if (isReverse())
		currentSensorAngle = -currentSensorAngle;
}


bool RotaryEncoder::fetchSample(bool raw, uint8_t no, float sample[], float& avr, float &variance) {
	avr = 0.;
	for (int check = 0;check<no;check++) {
		if (check > 0) {
			delay(ENCODER_SAMPLE_RATE);
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

	Serial.println();
	Serial.print(F("encoder("));
	// SerialPrintLn_P(setupData->name_P);
	Serial.print(")");

	if (!passedCheck) {
		Serial.print("] failed(avr=");
		Serial.print(avr);
		
		Serial.print(F(" var="));
		Serial.print(variance);
		Serial.println("!");
	}
	else
		Serial.println("ok");
	return variance;
}