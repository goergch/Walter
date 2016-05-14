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


void RotaryEncoder::setup(uint8_t number)
{
	if ((number < 1) || (number>MAX_MOTORS)) {
		Serial.print(F("setup encoder error"));
		delay(100);
		exit(0);
	};
	
	myNumber = number;
	// Serial.print("setup encoder");
	// Serial.println(number);

	//init AMS_AS5048B object
	uint8_t i2cAddress = i2CAddress(false);
	sensor.setI2CAddress(i2cAddress);
	sensor.begin();
	// Serial.print("I2c Address ");
	// Serial.println(i2cAddress,HEX);

	//set clock wise counting
	sensor.setClockWise(isClockwise());

	// check communication		
	currentSensorAngle = sensor.angleR(U_DEG, true);
	
	// do we have to reprogramm the I2C address?

	if (reprogrammei2CAddress()) {
		// Serial.println("repogramm I2c address");
		// reprogramm I2C address of this sensor by register programming
		uint8_t i2cAddress = sensor.addressRegR();	
		sensor.addressRegW(i2cAddress+I2C_ADDRESS_ADDON);		
		sensor.setI2CAddress(((i2cAddress+I2C_ADDRESS_ADDON) ^ (1<<4))<< 2); // see datasheet of AS5048B, computation of I2C address 
		sensor.begin(); // restart sensor with new I2C address
		
		// now boot the device with the same i2c address, there is no conflict anymore
		switchConflictingSensor(true /* = power on */);
	}
	// Serial.println("reading angle");

	currentSensorAngle = sensor.angleR(U_DEG, true);
	// Serial.print("current angle=");
	// Serial.println(currentSensorAngle);
} //RotaryEncode


void RotaryEncoder::setNullPosition() {
	zeroPosition = currentSensorAngle;
}

float RotaryEncoder::getAngle() {
	return currentSensorAngle - zeroPosition;
}

void RotaryEncoder::fetchAngle() {
	currentSensorAngle = sensor.angleR(U_DEG, true);
}
