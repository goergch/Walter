/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "HostCommunication.h"

HostCommunication hostComm;

enum errorCode { PARAM_WRONG, PARAM_NUMBER_WRONG, UNRECOGNIZED_CMD }; 
// Commands:
// LED (on|off|blink)				// switch LED
// ECHO "text"						// test
// h interactive mode				// switch to menu mode

extern void setLED(bool onOff);
extern void setLEDPattern();

void replyOk() {
	Serial.println(F("ok"));
}
void replyError(int errorCode) {
	Serial.print(F("nok"));
	Serial.print(errorCode);
}
void cmdLED() {
	bool paramsOK = true;
  
	char* param = 0;
	paramsOK = paramsOK && hostComm.sCmd.getParamString(param);

	if (paramsOK) {
		bool valueOK = false;
        if (strncmp(param, "on", 2) == 0) {
			setLED(true);
			valueOK = true;
		}
        if (strncmp(param, "off", 2) == 0) {
			setLED(false);
			valueOK = true;
		}		
        if (strncmp(param, "blink", 2) == 0) {
	        setLEDPattern();
			valueOK = true;
		}
		if (valueOK) {
			replyOk();
		}
		else
			replyError(PARAM_WRONG);
	} else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

void cmdECHO() {
	bool paramsOK = true;
	
	char* param = 0;
	paramsOK = paramsOK && hostComm.sCmd.getParamString(param);
	
	if (paramsOK) {
		Serial.println(param);
		replyOk();
	}
	else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

extern void setInteractiveMode(bool on);
void cmdHelp() {
	setInteractiveMode(true);
	replyOk();
}

extern void setupArms();
void cmdSETUP() {
	setupArms();
	replyOk();
}

// This gets set as the default handler, and gets called when no other command matches.
void cmdUnrecognized(const char *command) {
	Serial.print(command);
	replyError(UNRECOGNIZED_CMD);
}

// default constructor
HostCommunication::HostCommunication()
{
} //HostCommunication



void HostCommunication::setup() {
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("LED",    cmdLED);         // Turns LED on
	sCmd.addCommand("ECHO",   cmdECHO);        // test, echo the passed string
	sCmd.addCommand("h",	  cmdHelp);        // test, echo the passed string
	sCmd.addCommand("SETUP",  cmdSETUP);       // test, echo the passed string

	sCmd.setDefaultHandler(cmdUnrecognized);      // Handler for command that isn't matched  (says "What?")
}

void HostCommunication::loop() {
	sCmd.readSerial();     // We don't do much, just process serial commands
}


