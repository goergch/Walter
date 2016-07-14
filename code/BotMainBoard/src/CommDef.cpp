/*
 * CommDef.cpp
 *
 *  Created on: 14.07.2016
 *      Author: SuperJochenAlt
 */

#include "CommDef.h"

extern void cmdLED();
extern void cmdPOWER();
extern void cmdECHO();
extern void cmdMOVETO();
extern void cmdDISABLE();
extern void cmdENABLE();
extern void cmdGET();
extern void cmdSET();
extern void cmdSTEP();
extern void cmdMEM();
extern void cmdCHECKSUM();
extern void cmdKNOB();
extern void cmdLOG();
extern void cmdHELP();
extern void cmdINFO();


CommDefType commDef[COMMAND_NO] {
//    cmd ID	Name, timeout
	{ CommDefType::LED_CMD,		    "LED", 10, cmdLED },
	{ CommDefType::HELP_CMD,	    "HELP", 10, cmdHELP },
	{ CommDefType::ECHO_CMD,	    "ECHO", 10, cmdECHO },
	{ CommDefType::ENABLE_CMD,		"ENABLE", 10 , cmdENABLE},
	{ CommDefType::DISABLE_CMD,		"DISABLE", 10, cmdDISABLE },
	{ CommDefType::POWER_CMD,		"POWER", 10, cmdPOWER },
	{ CommDefType::KNOB_CMD,		"KNOB", 10, cmdKNOB },
	{ CommDefType::STEP_CMD,		"STEP", 10, cmdSTEP },
	{ CommDefType::CHECKSUM_CMD,	"CHECKSUM", 10 , cmdCHECKSUM},
	{ CommDefType::MEM_CMD,	        "MEM", 10 , cmdMEM},
	{ CommDefType::SET_CMD,	        "SET", 10 , cmdSET},
	{ CommDefType::GET_CMD,	        "GET", 10 , cmdGET},
	{ CommDefType::MOVETO_CMD,	    "MOVETO", 10 , cmdMOVETO},
	{ CommDefType::LOG_CMD,	        "Log", 10, cmdLOG },
	{ CommDefType::INFO_CMD,	    "INFO", 10, cmdINFO }
};


CommDefType* CommDefType::get(CommDefType::CommandType cmd) {
	for (int i = 0;i<COMMAND_NO;i++) {
		if (commDef[i].cmd == cmd)
			return &commDef[i];
	}
	return 0;
}
