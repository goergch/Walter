/* 
* I2CPortScanner.h
*
* Created: 02.05.2016 10:03:43
* Author: SuperJochenAlt
*/


#ifndef __I2CPORTSCANNER_H__
#define __I2CPORTSCANNER_H__

#include "Wire.h"
void doI2CPortScan()
{
	byte error, address;
	int nDevices;
	
	Serial.println("Scanning...");
	
	nDevices = 0;
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
			Serial.print(F("I2C device found at address 0x"));
			if (address<16)
			Serial.print("0");
			Serial.print(address,HEX);
			Serial.println("  !");
			
			nDevices++;
		}
		else if (error==4)
		{
			Serial.print(F("Unknow error at address 0x"));
			if (address<16)
				Serial.print("0");
			Serial.println(address,HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
	
}


#endif //__I2CPORTSCANNER_H__
