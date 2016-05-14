/* 
* RotaryEncoder.cpp
*
* Created: 23.04.2016 23:13:20
* Author: JochenAlt
*/

#include "Arduino.h"
#include "RotaryEncoder.h"
extern void doI2CPortScan();

void RotaryEncoder::switchOffConflictingSensor() {
	pinMode(I2C_ADDRESS_ADDON_VDD_PIN,INPUT);
	pinMode(I2C_ADDRESS_ADDON_VDD_GND,INPUT);
	digitalWrite(I2C_ADDRESS_ADDON_VDD_PIN, LOW); // disable internal pullup
	digitalWrite(I2C_ADDRESS_ADDON_VDD_GND, LOW); // disable internal pullup
}

void RotaryEncoder::switchOnConflictingSensor() {
	// now boot the device with the same i2c address, there is no conflict anymore
	pinMode(I2C_ADDRESS_ADDON_VDD_PIN,OUTPUT);
	digitalWrite(I2C_ADDRESS_ADDON_VDD_PIN, HIGH);
	pinMode(I2C_ADDRESS_ADDON_VDD_GND,OUTPUT);
	digitalWrite(I2C_ADDRESS_ADDON_VDD_GND, LOW);
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
		switchOnConflictingSensor();
	}
	// Serial.println("reading angle");

	currentSensorAngle = sensor.angleR(U_DEG, true);
	// Serial.print("current angle=");
	// Serial.println(currentSensorAngle);
} //RotaryEncode


/*
void RotaryEncoder::programmeI2CAddress(uint8_t currentI2cAddress, uint8_t newChipI2cAddress, uint8_t hardwareI2CAddOn) {

	//init AMS_AS5048B object
	sensor.setI2CAddress(currentI2cAddress);
	sensor.begin();
	
	// check communication
	currentSensorAngle = sensor.angleR(U_DEG, true);

	// read internal I2C Address
	Serial.println(F("reading chip address"));
	uint8_t i2cAddress = sensor.addressRegR() ;
	Serial.print(F("chip address="));
	Serial.println(i2cAddress);

	Serial.print("writing chip address 0x");
	Serial.println(newChipI2cAddress,HEX);

	sensor.addressRegW(newChipI2cAddress);

	Serial.print(F("restarting communication with address=0x"));
	
	i2cAddress = (newChipI2cAddress ^ (1<<4))<< 2;
	Serial.println(i2cAddress,HEX);

	sensor.setI2CAddress(i2cAddress);
	sensor.begin();
	doI2CPortScan();
	
	i2cAddress = sensor.addressRegR();
	Serial.print(F("new chip address="));
	Serial.println(newChipI2cAddress);

	float currentSensorAngle = sensor.angleR(U_DEG, true);
	Serial.print("current angle=");
	Serial.println(currentSensorAngle);
	
	sensor.doProgI2CAddress(newChipI2cAddress);
}
*/

void RotaryEncoder::setNullPosition() {
	zeroPosition = currentSensorAngle;
}

float RotaryEncoder::getAngle() {
	return currentSensorAngle - zeroPosition;
}

void RotaryEncoder::fetchAngle() {
	currentSensorAngle = sensor.angleR(U_DEG, true);
}
