/* 
* BotMemory.h
*
* Created: 21.04.2016 11:17:33
* Author: SuperJochenAlt
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
#include "setup.h"

// #include "HerkulexServoDrive.h"
#include "ActuatorConfig.h"


class BotMemory : public MemoryBase {
	public:
		// initialize  default values of memory for the very first start
		BotMemory();
		void println();
		static void setDefaults();
	struct  {
		ActuatorConfigurator armConfig[MAX_ACTUATORS];
	} persMem;
	struct {
	
	} config;
};


extern BotMemory memory;


#endif /* MAINMEMORY_H_ */

#endif //__BOTMEMORY_H__
