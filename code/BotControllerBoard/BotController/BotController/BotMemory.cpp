/* 
* BotMemory.cpp
*
* Created: 21.04.2016 11:17:33
* Author: SuperJochenAlt
*/


#include "MemoryBase.h"
#include "BotMemory.h"
#include <avr/eeprom.h>

char botMemory_EE[sizeof(BotMemory::persMem)] EEMEM;
BotMemory memory;

BotMemory::BotMemory()
: MemoryBase(botMemory_EE,(char*)&(persMem),sizeof(BotMemory::persMem)) {
	// initialization for the very first start, when EEPROM is not yet initialized
	BotMemory::setDefaults();
}

void BotMemory::setDefaults() {
	ArmConfig::setDefaults();	
}


void BotMemory::println() {
	Serial.println(F("EEPROM"));
	for (int i = 0;i<MAX_MOTORS;i++) {
		Serial.print(F("   motor["));Serial.print(i);Serial.print("]:");
		persMem.armConfig[i].print();
		Serial.println();
	}
}
