/* 
* HostCommunication.cpp
*
* Created: 26.06.2016 22:16:52
* Author: JochenAlt
*/


#include "I2CPortScanner.h"
#include "HostCommunication.h"
#include "Controller.h"
#include "BotMemory.h"
#include "CommDef.h"
#include "utilities.h"
#include "core.h"

HostCommunication hostComm;
extern Controller controller;
extern BotMemory botMemory;


// Commands:
// LED (on|off|blink)				// switch LED
// ECHO "text"						// test
// h interactive mode				// switch to menu mode

extern void setLED(bool onOff);
extern void setLEDPattern();

void replyOk() {
	cmdSerial->println(F(">ok"));
	cmdSerial->print(F(">"));
}

void replyError(int errorCode) {
	int patchedErrorCode = errorCode;
	if (errorCode == PARAM_NUMBER_WRONG) {
		if (hostComm.sCmd.getErrorCode() != 0) {
			patchedErrorCode = hostComm.sCmd.getErrorCode();
		}
	}
	cmdSerial->print(F(">nok("));
	cmdSerial->print(patchedErrorCode);
	cmdSerial->println(")");
	cmdSerial->print(F(">"));
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
		logger->print(F(" "));
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
		if (onOffSet && (strncasecmp(logClass, "loop", 5) == 0)) {
			memory.persMem.logLoop = onOffFlag;

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
		if (controller.isPowered())
			cmdSerial->print(F(" powered"));
		if (controller.isSetup())
			cmdSerial->print(F(" setuped"));
		if (controller.isEnabled())
			cmdSerial->print(F(" enabled"));

		if (controller.getCurrentActuator() != NULL) {
			cmdSerial->print(F(" curr="));
			cmdSerial->print(controller.getCurrentActuator()->getConfig().id);
		}


		cmdSerial->print(F(" i2c0=("));
		bool first = true;
		int count = 0;
		for (int i = 1;i<127;i++) {
			byte error;
			bool yes = scanI2CAddress(Wires[0], i, error);
			if (yes) {
				count++;
				if (!first)
					cmdSerial->print(F(","));
				cmdSerial->print(F("0x"));
				if (i<16)
					cmdSerial->print(F("0"));
				cmdSerial->print(i,HEX);
				first = false;
			}
		}
		cmdSerial->print(")");

		cmdSerial->print(F(" i2c1=("));
		first = true;
		for (int i = 1;i<127;i++) {
			byte error;
			bool yes = scanI2CAddress(Wires[1], i, error);
			if (yes) {
				if (!first)
					cmdSerial->print(F(","));
				cmdSerial->print(F("0x"));
				if (i<16)
					cmdSerial->print(F("0"));
				cmdSerial->print(i,HEX);
				first = false;
			}
		}
		cmdSerial->print(")");

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
		cmdSerial->print(param);

		replyOk();
	}
	else {
		replyError(PARAM_NUMBER_WRONG);
	}
}

void cmdSETUP() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		
		bool ok = controller.setup();
		if (ok)
			replyOk();		
		else
			replyError(getLastError());
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdENABLE() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		if (controller.isPowered()) {
			controller.enable();			// enable all actuators
			replyOk();
		} else 
			replyError(getLastError());
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
		if (controller.isSetup()) {
			if (strncasecmp(param, "on", 2) == 0) {
				controller.switchActuatorPowerSupply(true);
				valueOK = true;
			}
		} else {
			setError(CORTEX_POWER_ON_WITHOUT_SETUP);
			valueOK = false;
		}
		if (strncasecmp(param, "off", 3) == 0) {
			controller.disable();
			controller.switchActuatorPowerSupply(false);
			valueOK = true;
		}
		if (valueOK)
			replyOk();
		else
			replyError(getLastError());
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

void cmdKNOB() {
	int16_t actuatorNo = 0;
	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=7));
 
		if (valueOK) {
			if ((actuatorNo>=0) && (actuatorNo<=4) && !controller.isPowered()) {
				controller.switchActuatorPowerSupply(true);					
			} 
			
			controller.selectActuator(actuatorNo);			
			controller.adjustMotor(ADJUST_MOTOR_BY_KNOB);
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
	int16_t actuatorNo = 0;
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
	int16_t actuatorNo = 0;
	float maxSpeed,maxAcc,P,D,minValue, maxValue, nullValue = 0;
	bool maxSpeedSet, maxAccSet, PSet,DSet, minValueSet, maxValueSet, nullValueSet = false;

	bool paramsOK = hostComm.sCmd.getParamInt(actuatorNo);	
	paramsOK = hostComm.sCmd.getNamedParamFloat("min",minValue,minValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("max",maxValue,maxValueSet) && paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("null",nullValue,nullValueSet)&& paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("speed",maxSpeed,maxSpeedSet)&& paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("acc",maxAcc,maxAccSet)&& paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("P",P,PSet)&& paramsOK;
	paramsOK = hostComm.sCmd.getNamedParamFloat("D",D,DSet)&& paramsOK;

	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		bool valueOK = ((actuatorNo>=0) && (actuatorNo<=MAX_ACTUATORS));
		if (valueOK) {
			Actuator* actuator = controller.getActuator(actuatorNo);

			valueOK = false;
			if ((minValueSet) && (abs(minValue) < 180)) {
				actuator->setMinAngle(minValue);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE) 
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.minAngle= minValue;
				else
					memory.persMem.armConfig[actuatorNo].config.servoArm.servo.minAngle= minValue;

				valueOK = true;
			} 

			if ((maxValueSet) && (abs(maxValue) < 180)) {
				actuator->setMaxAngle(maxValue);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAngle= maxValue;
				else
					memory.persMem.armConfig[actuatorNo].config.servoArm.servo.maxAngle= maxValue;

				valueOK = true;
			}

			if ((nullValueSet) && (abs(nullValue) < 180)) {
				actuator->setNullAngle(nullValue);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.encoder.nullAngle= nullValue;
				else
					memory.persMem.armConfig[actuatorNo].config.servoArm.servo.nullAngle= nullValue;
				valueOK = true;
			}

			if ((maxSpeedSet) && (abs(maxSpeed) < 9999)) {
				actuator->setMaxSpeed(maxSpeedSet);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxSpeed = maxSpeed;
				valueOK = true;
			}

			if ((maxAccSet) && (abs(maxAcc) < 99999)) {
				actuator->setMaxAcc(maxAccSet);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.maxAcc= maxAcc;

				valueOK = true;
			}

			if ((PSet) && (fabs(P) <= 1.0)) {
				actuator->setP(P);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.kP= P;
					
				valueOK = true;
			}

			if ((DSet) && (fabs(D) <= 100.0)) {
				actuator->setD(D);
				if (memory.persMem.armConfig[actuatorNo].actuatorType  == STEPPER_ENCODER_TYPE)
					memory.persMem.armConfig[actuatorNo].config.stepperArm.stepper.kD= D;
				valueOK = true;
			}
		}

	
		if (valueOK) {
			memory.delayedSave();
			replyOk();
		}
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
				if ((actuatorStr != NULL) && (actuatorStr[0] >= '0') && (actuatorStr[0] <= '6')) {
					actuatorNo = atoi(actuatorStr);	
					valueOK = true;
				} else
					setError(PARAM_WRONG);
		}

		if (valueOK) {
			valueOK = false;
			if (controller.isSetup()) {
				if ((actuatorNo>=0) && (actuatorNo<MAX_ACTUATORS)) {
					Actuator* actuator = controller.getActuator(actuatorNo);
					cmdSerial->print(F(" n="));
					cmdSerial->print(actuatorNo);
					cmdSerial->print(F(" ang="));
					cmdSerial->print(actuator->getCurrentAngle(),2);
					cmdSerial->print(F(" min="));
					cmdSerial->print(actuator->getMinAngle(),2);
					cmdSerial->print(F(" max="));
					cmdSerial->print(actuator->getMaxAngle(),2);
					cmdSerial->print(F(" null="));
					cmdSerial->print(actuator->getNullAngle(),2);
					valueOK = true;
				} 
				if (isAll) {
					for (int i = 0;i<MAX_ACTUATORS;i++) {
						Actuator* actuator = controller.getActuator(i);
						cmdSerial->print(F(" n="));
						cmdSerial->print(i);
						cmdSerial->print(F(" ang="));
						cmdSerial->print(actuator->getCurrentAngle(),2);
						cmdSerial->print(F(" min="));
						cmdSerial->print(actuator->getMinAngle(),2);
						cmdSerial->print(F(" max="));
						cmdSerial->print(actuator->getMaxAngle(),2);
						cmdSerial->print(F(" null="));
						cmdSerial->print(actuator->getNullAngle(),2);
			
					}												
					valueOK = true;
				}
				if (valueOK)
					replyOk();
				else
					replyError(PARAM_WRONG);
			} else {
				setError(CORTEX_SETUP_MISSING);
				replyError(getLastError());
			}
		} else 
			replyError(PARAM_WRONG);			
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}


void cmdMOVETO() {
	float angle[7] = {0,0,0,0,0,0,0};
	bool paramsOK = true;
	int16_t duration = 0;
	for (int i = 0;i<7;i++) 
		paramsOK = hostComm.sCmd.getParamFloat(angle[i]) && (abs(angle[i]) <= 360.0) && paramsOK;

	paramsOK = hostComm.sCmd.getParamInt(duration) && (duration <= 9999) && (duration>=20) && paramsOK;
	paramsOK = hostComm.sCmd.endOfParams() && paramsOK;
	
	if (paramsOK) {
		if (memory.persMem.logLoop) {
			logger->print(F("moveTo("));
		}
		for (int i = 0;i<7;i++) {
			controller.getActuator(i)->setAngle(angle[i],duration);
			if (memory.persMem.logLoop) {
				logger->print(i);
				logger->print(",");
				logger->print(angle[i]);
				logger->print(",");
				logger->print(duration);
				logger->print(";");
			}
		}
		if (memory.persMem.logLoop) {
			logger->println();
		}
		replyOk();
	}
	else
		replyError(PARAM_NUMBER_WRONG);
}

// This gets set as the default handler, and gets called when no other command matches.
void cmdUnrecognized(const char *command) {
	cmdSerial->print(command);
	replyError(UNRECOGNIZED_CMD);
}

void cmdHELP() {
	bool paramsOK = hostComm.sCmd.endOfParams();
	if (paramsOK) {
		cmdSerial->println(F("usage:"));
		cmdSerial->println(F("\tLED <on|off|blink>"));
		cmdSerial->println(F("\tECHO \"hello\""));
		cmdSerial->println(F("\tSETUP"));
		cmdSerial->println(F("\tENABLE"));
		cmdSerial->println(F("\tDISABLE"));
		cmdSerial->println(F("\tPOWER <on|off>"));
		cmdSerial->println(F("\tKNOB <ActuatorNo>"));
		cmdSerial->println(F("\tSTEP <ActuatorNo> <incr>"));
		cmdSerial->println(F("\tCHECKSUM <on|off>"));
		cmdSerial->println(F("\tMEM (<reset>|<list>)"));
		cmdSerial->println(F("\tSET <ActuatorNo> [min=<min>] [max=<max>] [null=<nullvalue>] [speed=x][acc=x] [P=x][D=x]"));
		cmdSerial->println(F("\tGET <ActuatorNo> : n=<name> ang=<angle> min=<min> max=<max> null=<null>"));
		cmdSerial->println(F("\tGET all : (i=<no> n=<name> ang=<angle> min=<min> max=<max> null=<null>)"));
		cmdSerial->println(F("\tMOVETO <angle1> <angle2> ... <angle7> <durationMS>"));
		cmdSerial->println(F("\tLOG <setup|servo|stepper|encoder|loop> <on|off>"));
		cmdSerial->println(F("\tINFO"));

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

void HostCommunication::loop(uint32_t now) {
	sCmd.readSerial();     // We don't do much, just process serial commands
}


