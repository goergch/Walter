/*
 * CommDef.cpp
 *
 * Definition of serial communication protocol used between Walter's Cortex and Walters Webserver
 *
 * Author: JochenAlt
 */


#ifndef COMM_DEF_H_
#define COMM_DEF_H_

struct CommDefType {
	static const int NumberOfCommands = 18;

	// all possible commands the uC provides
	enum CommandType { 	LED_CMD = 0,
						HELP_CMD = 1,
						ECHO_CMD = 2,
						ENABLE_CMD = 3,
						DISABLE_CMD = 4,
						POWER_CMD = 5,
						KNOB_CMD = 6,
						STEP_CMD = 7,
						CHECKSUM_CMD = 8,
						MEM_CMD = 9,
						GET_CMD = 10,
						SET_CMD = 11,
						MOVETO_CMD = 12,
						LOG_CMD = 13,
						INFO_CMD = 14,
						SETUP_CMD = 15,
						PRINT_CMD = 16,
						PRINTLN_CMD = 17

	};
	CommandType cmd;
	const char*  name;
	int expectedExecutionTime_ms;

	// Pointer to the default handler function
    void (*cmdFunction)();
	static CommDefType* get(CommandType cmd);
};

extern CommDefType commDef[];

#endif
