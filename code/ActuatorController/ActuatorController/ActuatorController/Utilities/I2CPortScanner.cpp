/* 
* I2CPortScanner.h
*
* Created: 02.05.2016 10:03:43
* Author: JochenAlt
*/


#ifndef __I2CPORTSCANNER_H__
#define __I2CPORTSCANNER_H__

#include "Arduino.h"
#include "Wire.h"
#include <avr/wdt.h>


void doI2CPortScan(Stream* logger)
{
	byte error, address;
	int nDevices;
		
	nDevices = 0;
	bool deviceFound = false;
	for(address = 1; address < 127; address++ )
	{
		wdt_reset();

		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.

		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		
		if (error == 0)
		{
			if (!deviceFound) {
				logger->print(F("I2C devices at "));
				deviceFound = true;
			}
			else 
				logger->print(F(" "));
				
			logger->print("0x");
			if (address<16)
				logger->print("0");
			logger->print(address,HEX);
			
			nDevices++;
		}
		else if (error==4)
		{
			logger->print(F("Unknown error at address 0x"));
			if (address<16)
				logger->print("0");
			logger->println(address,HEX);
		}
	}
	if (deviceFound)
		logger->println();
	if (nDevices == 0)
		logger->println(F("No I2C devices found"));	
}


#endif //__I2CPORTSCANNER_H__
