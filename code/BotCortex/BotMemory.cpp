#include "MemoryBase.h"
#include "BotMemory.h"
#include <avr/eeprom.h>
#include "utilities.h"

BotMemory memory;

BotMemory::BotMemory()
: MemoryBase((void*)&(persMem),sizeof(BotMemory::persMem)) {
	// initialization for the very first start, when EEPROM is not yet initialized
	BotMemory::setDefaults();
}

void BotMemory::setDefaults() {
	memory.persMem.logSetup = true;
	memory.persMem.logServo = false;
	memory.persMem.logStepper = false;
	memory.persMem.logEncoder = false;
	memory.persMem.logLoop = true;

	ActuatorConfig::setDefaults();	
}


void BotMemory::println() {

	logger->println(F("EEPROM"));

	logger->print(F("   LOG=("));

	if (memory.persMem.logSetup)
		logger->print(F(" setup"));
	if (memory.persMem.logServo)
		logger->print(F(" servo"));
	if (memory.persMem.logStepper )
		logger->print(F(" stepper"));
	if (memory.persMem.logEncoder)
		logger->print(F(" encoder"));
	if (memory.persMem.logLoop)
		logger->print(F(" loop"));

	logger->println(")");

	for (int i = 0;i<MAX_ACTUATORS;i++) {
		if (persMem.armConfig[i].actuatorType != NO_ACTUATOR) {
			logger->print(F("   motor["));
			logger->print(i);logger->print("]:");
			persMem.armConfig[i].print();
			logger->println();
		}
	}
}
