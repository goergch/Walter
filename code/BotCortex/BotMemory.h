/* 
* BotMemory.h
*
* Persistent Memory of Walter. Stores actuator configuration and logging state in EEPROM.
*
* Author: JochenAlt
*/


#ifndef __BOTMEMORY_H__
#define __BOTMEMORY_H__
#include "Arduino.h"
#include "MemoryBase.h"
#include "Config.h"

class BotMemory;
extern BotMemory memory;

class BotMemory : public MemoryBase {
	public:
		// initialize  default values of memory for the very first start
		BotMemory();
		void println();
		static void setDefaults();
		static bool logSetup() { return memory.persMem.logSetup;};
		static bool logServo() { return memory.persMem.logServo;};
		static bool logStepper() { return memory.persMem.logStepper;};
		static bool logEncoder() { return memory.persMem.logEncoder;};

	struct  {
		bool logSetup;
		bool logServo;
		bool logStepper;
		bool logEncoder;
		bool logLoop;

		ActuatorConfig armConfig[MAX_ACTUATORS];
	} persMem;
};


#endif //__BOTMEMORY_H__
