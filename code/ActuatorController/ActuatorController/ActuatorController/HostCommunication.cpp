/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "HostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "CommDef.h"

HostCommunication hostComm;
extern Controller controller;
extern BotMemory botMemory;


enum errorCode { NO_ERROR = SerialCommand::NO_ERROR, CHECKSUM_EXPECTED = SerialCommand::CHECKSUM_EXPECTED, CHECKSUM_WRONG = SerialCommand::CHECKSUM_WRONG, 
				 PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5, CMD_ERROR = 6}; 
// Commands:
// LED (on|off|blink)				// switch LED
// ECHO "text"						// test
// h interactive mode				// switch to menu mode

extern void setLED(bool onOff);
extern void setLEDPattern();

void replyOk() {
	Serial.println(F(">ok"));
	Serial.print(F(">"));
}

void replyError(int errorCode) {
	int patchedErrorCode = errorCode;
	if (errorCode == PARAM_NUMBER_WRONG) {
		if (hostComm.sCmd.getErrorCode() != 0) {
			patchedErrorCode = hostComm.sCmd.getErrorCode();
		}
	}
	Serial.print(F(">nok("));
	Serial.print(patchedErrorCode);
	Serial.println(")");
	Serial.print(F(">"));
}


void cmdLOG() {
	char* logClass= NULL;
	char* onOff= NULL;

	bool paramsOK = hostComm.sCmd.getParamString(logClass);
	paramsOK = hostComm.sCmd.getParamString(onOff) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;
		bool onOffSet = false;
		bool onOffFlag = false;
		logger->print(F("log "));
		logger->print(logClass);
		logger->print(" ");
		logger->println(onOff);

		if (strncasecmp(onOff, "on", 2) == 0) {
			onOffFlag = true;
			onOffSet = true;
		}
		if (strncasecmp(onOff, "off", 3) == 0) {
			onOffFlag = false;
			onOffSet = true;
		}
		
		if (onOffSet && (strncasecmp(logClass, "setup", 5) == 0)) {
			memory.persMem.logSetup = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		
		if (onOffSet && (strncasecmp(logClass, "servo", 5) == 0)) {
			memory.persMem.logServo = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}

		if (onOffSet && (strncasecmp(logClass, "stepper", 5) == 0)) {
			memory.persMem.logStepper = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		if (onOffSet && (strncasecmp(logClass, "encoder", 5) == 0)) {
			memory.persMem.logEncoder = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		if (onOffSet && (strncasecmp(logClass, "test", 5) == 0)) {
			valueOK = true;
			replyOk();
			return;
		}


		if (valueOK) 
			replyOk();
		else
			replyError(PARAM_WRONG);
	} else {
		replyError(PARAM_NUMBER_WRONG);
	}
}


void cmdLED() {  
	char* param = 0;
	bool paramsOK = hostComm.sCmd.getParamString(param);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;
        if (strncasecmp(param, "on", 2) == 0) {
			setLED(true);
			valueOK = true;
		}
        if (strncasecmp(param, "off", 3) == 0) {
			setLED(false);
			valueOK = true;
		}		
        if (strncasecmp(param, "blink", 5) == 0) {
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

void cmdINFO() {
	bool paramsOK = true;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		if (controller.setuped())
			Serial.print(F(" setuped"));
		if (controller.isEnabled())
			Serial.print(F(" enabled"));
		else		
			Serial.print(F(" disabled"));

		if (controller.getCurrentActuator() != NULL) {
			Serial.print(F(" curr="));
			Serial.print(controller.getCurrentActuator()->getConfig().id);
		}

		Serial.print(F(" i2c=("));		
		bool first = true;
		for (int i = 0;i<127;i++) {
			byte error;
			bool yes = scanI2CAddress(i, error);			
			if (yes) {
				if (!first)
					Serial.print(F(","));
				Serial.print(F("0x"));
				if (i<16)
					Serial.print(F("0"));
				Serial.print(i,HEX);
				first = false;
			}
		}
		Serial.print(")");

		replyOk();
	} else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

// the only command that does not require a checksum
void cmdCHECKSUM() {
	char* onoff = 0;
	bool paramsOK = hostComm.sCmd.getParamString(onoff);
	// paramsOK = hostComm.sCmd.endOfParams() && paramsOK;

	if (paramsOK) {
		bool valueOK = false;
		if (strncasecmp(onoff, "on", 2) == 0) {
			hostComm.sCmd.useChecksum(true);
			valueOK = true;
		}
		if (strncasecmp(onoff, "off", 3) == 0) {
			hostComm.sCmd.useChecksum(false);
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
	paramsOK = hostComm.sCmd.getParamString(param) && paramsOK; 
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;

	
	if (paramsOK) {
		Serial.print(param);

		replyOk();
	}
	else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

extern void setInteractiveMode(bool on);

void cmdSETUP() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		bool ok = controller.setup();
		if (ok)
			replyOk();		
		else
			replyError(CMD_ERROR);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdENABLE() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		controller.enable();			// enable all actuators
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdDISABLE(){
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		controller.disable();			// disable first, in order to avoid ticks
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdPOWER(){
	char* param = 0;
	bool paramsOK = hostComm.sCmd.getParamString(param);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;

	if (paramsOK) {
		bool valueOK = false;
		if (strncasecmp(param, "on", 2) == 0) {
			controller.switchActuatorPowerSupply(true);
			delay(10); // short break to let voltage become stable
			valueOK = true;
		}
		if (strncasecmp(param, "off", 3) == 0) {
			controller.switchActuatorPowerSupply(false);
			delay(10); // short break to calm down
			valueOK = true;
		}
		if (valueOK)
			replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdKNOB() {
	char* feedbackParam = 0;
	int actuatorNo = 0;
	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);
	paramsOK = paramsOK && hostComm.sCmd.getParamString(feedbackParam);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=7));
		bool withNoFeedback = strncasecmp(feedbackParam, "nofeedback", 10) == 0;
		bool withFeedback = strncasecmp(feedbackParam, "feedback", 8) == 0;
 
		if ((valueOK) && (withNoFeedback || withFeedback)) {
			controller.selectActuator(actuatorNo);			
			if (withNoFeedback)
				controller.adjustMotor(ADJUST_MOTOR_BY_KNOB_WITHOUT_FEEDBACK);
			if (withFeedback)
				controller.adjustMotor(ADJUST_MOTOR_BY_KNOB_WITH_FEEDBACK);
			replyOk();
		}
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdMEM() {
	char* cmdParam = 0;
	bool paramsOK = hostComm.sCmd.getParamString(cmdParam);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;

		if (strncasecmp(cmdParam, "reset", 5) == 0) {
			BotMemory::setDefaults();
			memory.println();
			memory.save();
			valueOK = true;
		}

		if (strncasecmp(cmdParam, "list", 4) == 0) {
			controller.printConfiguration();
			valueOK = true;
		}
		
		if (valueOK)
			replyOk();
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdSTEP() {
	int actuatorNo = 0;
	float incr = 0;

	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);
	paramsOK = paramsOK && hostComm.sCmd.getParamFloat(incr);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=6));
		if (valueOK && (abs(incr) < 10)) {
			controller.selectActuator(actuatorNo);
			controller.changeAngle(incr, abs(incr)*50);
		} else
			valueOK = false;

		if (valueOK)
			replyOk();
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdSET() {
	int actuatorNo = 0;
	float minValue, maxValue, nullValue = 0;
	bool minValueSet, maxValueSet, nullValueSet = false;
	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);	
	paramsOK = hostComm.sCmd.getNamedParamFloat("min",minValue,minValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("max",maxValue,maxValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("null",nullValue,nullValueSet)&& paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=MAX_ACTUATORS));
		if (valueOK) {
			Actuator* actuator = controller.getActuator(actuatorNo);

			valueOK = false;
			if ((minValueSet) && (abs(minValue) < 180)) {
				actuator->setMinAngle(minValue);
				valueOK = true;
			} 

			if ((maxValueSet) && (abs(maxValue) < 180)) {
				actuator->setMaxAngle(maxValue);
				valueOK = true;
			}

			if ((nullValueSet) && (abs(nullValue) < 180)) {
				actuator->setNullAngle(nullValue);
				valueOK = true;
			}
		}
	
		if (valueOK)
			replyOk();
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdGET() {
	int actuatorNo = -1;
	bool isAll = false;
	char* actuatorStr= 0;
	bool paramsOK = hostComm.sCmd.getParamString(actuatorStr);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;

		if (actuatorStr) {
			isAll = (strncasecmp(actuatorStr, "all", 3) == 0);
			if (isAll)
				valueOK = true;
			else
				if ((actuatorStr != NULL) && (actuatorStr[0] >= '0') && (actuatorStr[0] <= '9')) {
					actuatorNo = atoi(actuatorStr);	
					valueOK = true;
				}
		}

		if (valueOK) {
			valueOK = false;
			if (controller.setupIsDone()) {
				if ((actuatorNo>=0) && (actuatorNo<MAX_ACTUATORS)) {
					Actuator* actuator = controller.getActuator(actuatorNo);
					Serial.print(F("n="));
					actuator->printName();
					Serial.print(F(" ang="));
					Serial.print(actuator->getCurrentAngle(),2);
					Serial.print(F(" min="));
					Serial.print(actuator->getMinAngle(),2);
					Serial.print(F(" max="));
					Serial.print(actuator->getMaxAngle(),2);
					Serial.print(F(" null="));
					Serial.print(actuator->getNullAngle(),2);
					valueOK = true;
				} 
				if (isAll) {
					for (int i = 0;i<MAX_ACTUATORS;i++) {
						Actuator* actuator = controller.getActuator(actuatorNo);
						Serial.print(F(" i="));
						Serial.print(i);
						Serial.print(F(" ang="));
						Serial.print(actuator->getCurrentAngle(),2);
						Serial.print(F(" min="));
						Serial.print(actuator->getMinAngle(),2);
						Serial.print(F(" max="));
						Serial.print(actuator->getMaxAngle(),2);
						Serial.print(F(" null="));
						Serial.print(actuator->getNullAngle(),2);
					}												
					valueOK = true;
				}
				if (valueOK)
					replyOk();
				else
					replyError(PARAM_WRONG);
			} else 
				replyError(CMD_ERROR);		
		} else 
			replyError(PARAM_WRONG);			
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdMOVETO() {
	float angle[7] = {0,0,0,0,0,0,0};
	bool paramsOK = true;
	int duration = 0; 
	for (int i = 0;i<7;i++) 
		paramsOK = hostComm.sCmd.getParamFloat(angle[i]) && (abs(angle[i]) <= 180.0) && paramsOK;

	paramsOK = hostComm.sCmd.getParamInt(duration) && (duration < 1000) && (duration>50) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		for (int i = 0;i<7;i++) 
			controller.getActuator(i)->setAngle(angle[i],duration);

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

void cmdHELP() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		Serial.println(F("usage:"));
		Serial.println(F("\tLED <on|off|blink>"));
		Serial.println(F("\tECHO \"hello\""));
		Serial.println(F("\tSETUP"));		
		Serial.println(F("\tENABLE"));
		Serial.println(F("\tDISABLE"));
		Serial.println(F("\tPOWER <on|off>"));
		Serial.println(F("\tKNOB <ActuatorNo> <feedback|nofeedback>"));
		Serial.println(F("\tSTEP <ActuatorNo> <incr>"));
		Serial.println(F("\tCHECKSUM <on|off>"));
		Serial.println(F("\tMEM (<reset>|<list>)"));
		Serial.println(F("\tSET <ActuatorNo> [min=<min>] [max=<max>] [null=<nullvalue>]"));
		Serial.println(F("\tGET <ActuatorNo> : n=<name> ang=<angle> min=<min> max=<max> null=<null>"));
		Serial.println(F("\tGET all : (i=<no> n=<name> ang=<angle> min=<min> max=<max> null=<null>)"));
		Serial.println(F("\tMOVETO <angle1> <angle2> ... <angle7> <durationMS>"));
		Serial.println(F("\tLOG <setup|servo|stepper|encoder> <on|off>"));
		Serial.println(F("\tINFO (setup) (enabled|disabled)"));

		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

// default constructor
HostCommunication::HostCommunication()
{
} //HostCommunication



void HostCommunication::setup() {
	// Setup callbacks for SerialCommand commands
	for (int i = 0;i<CommDefType::NumberOfCommands;i++) {
		
		sCmd.addCommand(commDef[i].name, commDef[i].cmdFunction);
	}
	sCmd.setDefaultHandler(cmdUnrecognized);   // Handler for command that isn't matched  (says "What?")

	sCmd.useChecksum(false);
}

void HostCommunication::loop() {
	sCmd.readSerial();     // We don't do much, just process serial commands
}


