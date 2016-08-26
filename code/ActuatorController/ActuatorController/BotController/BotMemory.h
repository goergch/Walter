/* 
* BotMemory.h
*
* Created: 21.04.2016 11:17:33
* Author: JochenAlt
*/


#ifndef __BOTMEMORY_H__
#define __BOTMEMORY_H__

/*
 * MainMemory.h
 *
 * Created: 04.04.2013 18:07:06
 *  Author: JochenAlt
 */ 


#ifndef MAINMEMORY_H_
#define MAINMEMORY_H_

#include "Arduino.h"
#include "MemoryBase.h"

// #include "HerkulexServoDrive.h"
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
		ActuatorConfig armConfig[MAX_ACTUATORS];
	} persMem;
};




#endif /* MAINMEMORY_H_ */

#endif //__BOTMEMORY_H__
