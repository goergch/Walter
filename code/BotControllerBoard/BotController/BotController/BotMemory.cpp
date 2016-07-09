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
	ActuatorConfig::setDefaults();	
}


void BotMemory::println() {
	logger->println(F("EEPROM"));
	for (int i = 0;i<MAX_ACTUATORS;i++) {
		if (persMem.armConfig[i].actuatorType != NO_ACTUATOR) {
			logger->print(F("   motor["));
			logger->print(i);logger->print("]:");
			persMem.armConfig[i].print();
			logger->println();
		}
	}
}
