/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "HostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"

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
		if (strncasecmp(onOff, "on", 2) == 0) {
			onOffFlag = true;
			onOffSet = true;
		}
		if (strncasecmp(onOff, "off", 3) == 0) {
			onOffFlag = false;
			onOffSet = true;
		}

		if (onOffSet && (strncasecmp(logClass, "setup", 5) == 0)) {
			logSetup = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		
		if (onOffSet && (strncasecmp(logClass, "servo", 5) == 0)) {
			logServo = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}

		if (onOffSet && (strncasecmp(logClass, "stepper", 5) == 0)) {
			logStepper = onOffFlag;
			valueOK = true;
			replyOk();
			return;
		}
		if (onOffSet && (strncasecmp(logClass, "encoder", 5) == 0)) {
			logEncoder = onOffFlag;
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
			Serial.print(" curr=");
			Serial.print(controller.getCurrentActuator()->getConfig().id);
		}

		Serial.print(" i2c=(");		
		bool first = true;
		for (int i = 0;i<127;i++) {
			byte error;
			bool yes = scanI2CAddress(i, error);			
			if (yes) {
				if (!first)
					Serial.print(",");
				Serial.print("0x");
				if (i<16)
					Serial.print("0");
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
	char* absrel = 0;
	int actuatorNo = 0;
	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);
	paramsOK = paramsOK && hostComm.sCmd.getParamString(absrel);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=7));
		bool isRel = strncasecmp(absrel, "rel", 3) == 0;
		bool isAbs = strncasecmp(absrel, "abs", 3) == 0;
 
		if ((valueOK) && (isRel || isAbs)) {
			controller.selectActuator(actuatorNo);			
			if (isRel)
				controller.adjustMotor(ADJUST_MOTOR_BY_KNOB);
			if (isAbs)
				controller.adjustMotor(ADJUST_MOTOR_ANGLE_ABS_BY_KNOB);
			replyOk();
		}
		else
			replyError(PARAM_WRONG);
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdMEMORY() {
	char* cmdParam = 0;
	bool paramsOK = hostComm.sCmd.getParamString(cmdParam);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = false;

		if (strncasecmp(cmdParam, "reset", 5) == 0) {
			BotMemory::setDefaults();
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

void cmdMOVE() {
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
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=7));
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
	int actuatorNo = 0;

	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=7));
		if (valueOK) {
			Actuator* actuator = controller.getActuator(actuatorNo);
			Serial.print(F("n="));
			actuator->printName();
			Serial.print(F(" a="));
			Serial.print(actuator->getCurrentAngle(),2);
			Serial.print(F(" min="));
			Serial.print(actuator->getMinAngle(),2);
			Serial.print(F(" max="));
			Serial.print(actuator->getMaxAngle(),2);
			Serial.print(F(" null="));
			Serial.print(actuator->getNullAngle(),2);
		}
		
		if (valueOK)
			replyOk();
		else
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
		paramsOK = paramsOK  && hostComm.sCmd.getParamFloat(angle[i]) && (abs(angle[i]) <= 180.0);
	
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
		Serial.println(F("\tKNOB <ActuatorNo> <rel|abs>"));
		Serial.println(F("\tMOVE <ActuatorNo> <incr>"));
		Serial.println(F("\tCHECKSUM <on|off>"));
		Serial.println(F("\tMEM (<reset>)"));
		Serial.println(F("\tSET <ActuatorNo> [min=<min>] [max=<max>] [null=<nullvalue>]"));
		Serial.println(F("\tGET <ActuatorNo> : n=<name> a=<angle> min=<min> max=<max> null=<null>"));
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
	sCmd.addCommand("HELP",    cmdHELP);        // show help
	sCmd.addCommand("h",	  cmdHELP);         // test, echo the passed string
	sCmd.addCommand("LED",    cmdLED);          // Turns LED on
	sCmd.addCommand("ECHO",   cmdECHO);         // test, echo the passed string
	sCmd.addCommand("SETUP",  cmdSETUP);        // setup arms
	sCmd.addCommand("ENABLE", cmdENABLE);       // enable all motors, switch on power supply
	sCmd.addCommand("DISABLE", cmdDISABLE);     // disable all motors, switch off power supply
	sCmd.addCommand("POWER", cmdPOWER);         // switch power of motors
	sCmd.addCommand("KNOB", cmdKNOB);           // use the knob
	sCmd.addCommand("MOVE", cmdMOVE);           // move actuator by degree
	sCmd.addCommand("CHECKSUM", cmdCHECKSUM);   // switch checksum on/off
	sCmd.addCommand("MEM", cmdMEMORY);          // show memory (setup and config)
	sCmd.addCommand("SET", cmdSET);				// set configuration information
	sCmd.addCommand("GET", cmdGET);				// get configuration information
	sCmd.addCommand("MOVETO", cmdMOVETO);		// main function to move each actuator to a specific angle
	sCmd.addCommand("LOG", cmdLOG);				// switch logging on/off
	sCmd.addCommand("INFO", cmdINFO);			// switch logging on/off
	
	
	sCmd.setDefaultHandler(cmdUnrecognized);   // Handler for command that isn't matched  (says "What?")

	sCmd.useChecksum(false);
}

void HostCommunication::loop() {
	sCmd.readSerial();     // We don't do much, just process serial commands
}


