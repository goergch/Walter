/* 
* BotMemory.cpp
*
* Created: 21.04.2016 11:17:33
* Author: SuperJochenAlt
*/


#include "MemoryBase.h"
#include "BotMemory.h"
#include <avr/eeprom.h>

char botMemory_EE[sizeof(BotMemory::persistentMem)] EEMEM;
BotMemory memory;

BotMemory::BotMemory()
: MemoryBase(botMemory_EE,(char*)&(persistentMem),sizeof(BotMemory::persistentMem)) {
	// initialization for the very first start, when EEPROM is not yet initialized
	MotorDriverConfig::setDefaults();
}


void BotMemory::println() {
	for (int i = 0;i<MAX_MOTORS;i++)
		persistentMem.motorConfig[i].println();
}
