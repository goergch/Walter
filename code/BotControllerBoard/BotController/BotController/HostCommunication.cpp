/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "HostCommunication.h"
#include "Controller.h"

HostCommunication hostComm;
extern Controller controller;

enum errorCode { PARAM_WRONG, PARAM_NUMBER_WRONG, UNRECOGNIZED_CMD }; 
// Commands:
// LED (on|off|blink)				// switch LED
// ECHO "text"						// test
// h interactive mode				// switch to menu mode

extern void setLED(bool onOff);
extern void setLEDPattern();

void replyOk() {
	Serial.println(F("ok."));
}

void replyError(int errorCode) {
	Serial.print(F("nok("));
	Serial.print(errorCode);
	Serial.println(")");
}


void cmdLED(uint8_t checksum) {  
	char* param = 0;
	bool paramsOK = hostComm.sCmd.getParamString(param,checksum);
	paramsOK = paramsOK & hostComm.sCmd.checkCheckSum(checksum);

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

void cmdECHO(uint8_t checksum) {
	bool paramsOK = true;
	
	char* param = 0;
	paramsOK = paramsOK && hostComm.sCmd.getParamString(param, checksum);
	paramsOK = paramsOK & hostComm.sCmd.checkCheckSum(checksum);

	
	if (paramsOK) {
		Serial.println(param);
		replyOk();
	}
	else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

extern void setInteractiveMode(bool on);
void cmdHelp(uint8_t checksum) {
	bool paramsOK = hostComm.sCmd.checkCheckSum(checksum);
	if (paramsOK) {
		setInteractiveMode(true);
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

extern void setupArms();
void cmdSETUP(uint8_t checksum) {
	bool paramsOK = hostComm.sCmd.checkCheckSum(checksum);
	if (paramsOK) {
		setupArms();
		replyOk();		
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdENABLE(uint8_t checksum) {
	bool paramsOK = hostComm.sCmd.checkCheckSum(checksum);
	if (paramsOK) {
		controller.enable();
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdDISABLE(uint8_t checksum){
	bool paramsOK = hostComm.sCmd.checkCheckSum(checksum);
	if (paramsOK) {
		controller.disable();
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
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
	sCmd.addCommand("SETUP",  cmdSETUP);       // call setup
	sCmd.addCommand("ENABLE", cmdENABLE);      // enable all motors
	sCmd.addCommand("DISABLE", cmdDISABLE);    // disableall motors

	sCmd.setDefaultHandler(cmdUnrecognized);      // Handler for command that isn't matched  (says "What?")
}

void HostCommunication::loop() {
	sCmd.readSerial();     // We don't do much, just process serial commands
}


