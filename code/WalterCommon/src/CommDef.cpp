/*
 * CommDef.cpp
 *
 *  Created on: 14.07.2016
 *      Author: JochenAlt
 */

#include "CommDef.h"

// functions pointers implementing a command. Used on uC side only.
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
extern void cmdCONFIG();


CommDefType commDef[CommDefType::NumberOfCommands] {
	//cmd ID						Name, 		timeout,function pointer
	{ CommDefType::LED_CMD,		    "LED",		500, 	cmdLED },
	{ CommDefType::HELP_CMD,	    "HELP", 	500, 	cmdHELP },
	{ CommDefType::ECHO_CMD,	    "ECHO", 	500, 	cmdECHO },
	{ CommDefType::ENABLE_CMD,		"ENABLE", 	500, 	cmdENABLE},
	{ CommDefType::DISABLE_CMD,		"DISABLE", 	200, 	cmdDISABLE },
	{ CommDefType::SETUP_CMD,		"SETUP", 	1000, 	cmdSETUP},
	{ CommDefType::POWER_CMD,		"POWER", 	500, 	cmdPOWER },
	{ CommDefType::KNOB_CMD,		"KNOB", 	200, 	cmdKNOB },
	{ CommDefType::STEP_CMD,		"STEP", 	200, 	cmdSTEP },
	{ CommDefType::CHECKSUM_CMD,	"CHECKSUM", 200, 	cmdCHECKSUM},
	{ CommDefType::MEM_CMD,	        "MEM", 		200, 	cmdMEM},
	{ CommDefType::SET_CMD,	        "SET", 		100, 	cmdSET},
	{ CommDefType::GET_CMD,	        "GET", 		100, 	cmdGET},
	{ CommDefType::MOVETO_CMD,	    "MOVETO", 	75, 	cmdMOVETO},
	{ CommDefType::LOG_CMD,	        "Log", 		200, 	cmdLOG },
	{ CommDefType::INFO_CMD,	    "INFO", 	200, 	cmdINFO }
};

// return  command definition of the passed command
CommDefType* CommDefType::get(CommDefType::CommandType cmd) {
	for (int i = 0;i<NumberOfCommands;i++) {
		if (commDef[i].cmd == cmd)
			return &commDef[i];
	}
	return 0;
}
