/*
 * CommDef.cpp
 *
 *  Created on: 14.07.2016
 *      Author: JochenAlt
 */

#include "CommDef.h"

extern void cmdLED();
extern void cmdPOWER();
extern void cmdECHO();
extern void cmdSETUP();
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


CommDefType commDef[CommDefType::NumberOfCommands] {
//    cmd ID	Name, timeout
	{ CommDefType::LED_CMD,		    "LED",		100, 	cmdLED },
	{ CommDefType::HELP_CMD,	    "HELP", 	100, 	cmdHELP },
	{ CommDefType::ECHO_CMD,	    "ECHO", 	100, 	cmdECHO },
	{ CommDefType::SETUP_CMD,		"SETUP", 	1000 , 	cmdSETUP},
	{ CommDefType::ENABLE_CMD,		"ENABLE", 	100, 	cmdENABLE},
	{ CommDefType::DISABLE_CMD,		"DISABLE", 	100, 	cmdDISABLE },
	{ CommDefType::POWER_CMD,		"POWER", 	100, 	cmdPOWER },
	{ CommDefType::KNOB_CMD,		"KNOB", 	100, 	cmdKNOB },
	{ CommDefType::STEP_CMD,		"STEP", 	100, 	cmdSTEP },
	{ CommDefType::CHECKSUM_CMD,	"CHECKSUM", 100, 	cmdCHECKSUM},
	{ CommDefType::MEM_CMD,	        "MEM", 		100, 	cmdMEM},
	{ CommDefType::SET_CMD,	        "SET", 		100, 	cmdSET},
	{ CommDefType::GET_CMD,	        "GET", 		100, 	cmdGET},
	{ CommDefType::MOVETO_CMD,	    "MOVETO", 	100, 	cmdMOVETO},
	{ CommDefType::LOG_CMD,	        "Log", 		100, 	cmdLOG },
	{ CommDefType::INFO_CMD,	    "INFO", 	100, 	cmdINFO }
};


CommDefType* CommDefType::get(CommDefType::CommandType cmd) {
	for (int i = 0;i<NumberOfCommands;i++) {
		if (commDef[i].cmd == cmd)
			return &commDef[i];
	}
	return 0;
}
