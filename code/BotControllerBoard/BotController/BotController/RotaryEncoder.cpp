/* 
* RotaryEncoder.cpp
*
* Created: 23.04.2016 23:13:20
* Author: JochenAlt
*/

#include "Arduino.h"
#include "RotaryEncoder.h"
#include "BotMemory.h"
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


void RotaryEncoder::setup(uint8_t number)
{
	passedCheck= false;

	if ((number < 1) || (number>MAX_MOTORS)) {
		Serial.print(F("setup encoder error"));
		delay(100);
		exit(0);
	};
	
	myNumber = number;
	// Serial.print("setup encoder");
	// Serial.println(number);

	bool reprogrammeI2CAddress = reprogrammei2CAddress();
	if (reprogrammeI2CAddress) {
		uint8_t reprogrammedi2cAddress = i2CAddress(false) + (I2C_ADDRESS_ADDON<<2);

		Wire.beginTransmission(reprogrammedi2cAddress );
		byte error = Wire.endTransmission();
		if (error == 0) {
			// Serial.print(F("I2C address amended"));
			// Serial.println(reprogrammedi2cAddress,HEX);
			
			reprogrammeI2CAddress = false;
			
			sensor.setI2CAddress(reprogrammedi2cAddress);
			// Serial.print("set I2C to ");
			// Serial.println(reprogrammedi2cAddress,HEX);


			sensor.begin(); // restart sensor with new I2C address
			switchConflictingSensor(true /* = power on */);
		}
	} else {
		//init AMS_AS5048B object
		uint8_t i2cAddress = i2CAddress(false);
		sensor.setI2CAddress(i2cAddress);
		sensor.begin();		
	}

	// Serial.print("I2c Address ");
	// Serial.println(i2cAddress,HEX);

	//set clock wise counting
	sensor.setClockWise(isClockwise());

	// check communication		
	currentSensorAngle = sensor.angleR(U_DEG, true);
	
	// do we have to reprogramm the I2C address?

	if (reprogrammeI2CAddress) {
		// Serial.println("repogramm I2c address");
		// reprogramm I2C address of this sensor by register programming
		uint8_t i2cAddress = sensor.addressRegR();	
		sensor.addressRegW(i2cAddress+I2C_ADDRESS_ADDON);		
		sensor.setI2CAddress(((i2cAddress+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2); // see datasheet of AS5048B, computation of I2C address 
		Serial.print("reprogramm to ");
		Serial.println(((i2cAddress+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2,HEX);
		sensor.begin(); // restart sensor with new I2C address
		
		// now boot the device with the same i2c address, there is no conflict anymore
		switchConflictingSensor(true /* = power on */);
	}
	// Serial.println("reading angle");

	currentSensorAngle = sensor.angleR(U_DEG, true);
	// Serial.print("current angle=");
	// Serial.println(currentSensorAngle);
} //RotaryEncode


float RotaryEncoder::getAngle() {
	return currentSensorAngle - memory.persistentMem.motorConfig[myNumber].nullAngle;
}

float RotaryEncoder::getRawAngle() {
	return currentSensorAngle;
}

void RotaryEncoder::fetchAngle() {
	currentSensorAngle = sensor.angleR(U_DEG, true);
}


float RotaryEncoder::checkEncoderVariance() {
	
	// collect samples of all encoders
	float value[ENCODER_CHECK_NO_OF_SAMPLES];
	for (int check = 0;check<ENCODER_CHECK_NO_OF_SAMPLES;check++) {
		if (check > 0) {
			delay(ENCODER_SAMPLE_RATE);
			Serial.println();
		}
		fetchAngle(); // measure the encoder's angle
		float x = getAngle();
		value[check] = x;
	}

	// check values, compute average and deviation and decide if we start
	float avr = 0;
	for (int check = 0;check<ENCODER_CHECK_NO_OF_SAMPLES;check++) {
		avr += value[check];
	}
	avr = avr/ENCODER_CHECK_NO_OF_SAMPLES;
	float variance = 0;
	for ( int check = 0;check<ENCODER_CHECK_NO_OF_SAMPLES;check++) {
		float d = value[check]-avr;
		variance += d*d;
	}
	variance = variance/ENCODER_CHECK_NO_OF_SAMPLES;
	if (variance <= ENCODER_CHECK_MAX_VARIANCE)
		passedCheck = true;
	else {
		Serial.println();
		Serial.print(F("encodercheck["));
		Serial.print(myNumber-1);
		Serial.print("] failed(avr=");
		Serial.print(avr);
		
		Serial.print(F(" var="));
		Serial.print(variance);
		Serial.println(")");

	}
	return variance;
}