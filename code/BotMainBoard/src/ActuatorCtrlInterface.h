/*
 * MicroControllerInterface.h
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#ifndef MICROCONTROLLERINTERFACE_H_
#define MICROCONTROLLERINTERFACE_H_

#include <thread>
#include "string.h"

#include "setup.h"
#include "Util.h"
#include "SerialPort.h"

using namespace std;

#include "CommDef.h"

class ActuatorCtrlInterface {
public:
	static const int NumberOfActuators = 7;

	enum LEDState { LED_ON, LED_OFF, LED_BLINKS };


	ActuatorCtrlInterface() {
		errorCode = NO_ERROR_CODE;
		powerOn = false;
		ledState = LED_OFF;
		ledStatePending = true;
		logSuckingThread= NULL;
		withChecksum = false;
		logMCToConsole = false;
	}
	static ActuatorCtrlInterface& getInstance() {
		static ActuatorCtrlInterface instance;
		return instance;
	}

	// log everything from uC to cout. Used for directly access the uC
	void loguCToConsole() { logMCToConsole = true; };

	// initialize a safe communication. uC's setup is not called, bot remains silent
	bool setupCommunication();

	// send a direct command to uC
	void directAccess(string cmd, string& response, bool &okOrNOk);

	// okay, that's a gimmick only, but everyone likes a blinking LED
	void setLEDState(LEDState state);

	// setup the Bot, do not yet switch it on
	void setupBot();

	// return current angles
	void getAngles(ActuatorStateType actuatorState[]);

	// switch on/off the bot. Requires setupBot upfront
	void power(bool onOff);

	// most important method to transfer a trajectory point to uC
	// requires setupBot and power(true) upfront
	void move(rational angle[], int duration_ms);


	void loop();

private:
	enum ErrorCodeType:int {
		NO_ERROR_CODE = 0, CHECKSUM_EXPECTED = 1, CHECKSUM_WRONG = 2,
		PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5,
		CMD_ERROR = 6, NO_RESPONSE_CODE= 7};

	bool receive(string& str, int timeout_ms);
	bool checkReponseCode(string &s, string& plainResponse, bool &OkOrNOk);

	void sendString(string str);
	ErrorCodeType getError();
	bool isError();

	bool cmdLED(LEDState state);
	bool cmdECHO(string s);
	bool cmdCHECKSUM(bool onOff);
	bool cmdPOWER(bool onOff);
	bool cmdSETUP();
	bool cmdDISABLE();
	bool cmdENABLE();
	bool cmdMOVETO(rational angle[7], int duration_ms);
	bool cmdGET(int actuatorNo, ActuatorStateType actuatorState);
	bool cmdGETall(ActuatorStateType actuatorState[]);

	bool cmdSET(int ActuatorNo, rational minAngle, rational maxAngle, rational nullAngle);
	bool cmdSTEP(int actuatorID, rational incr);
	bool cmdMEMReset();
	bool cmdMEMList(string &result);
	bool cmdKNOB(bool useAbs);
	bool cmdLOGsetup(bool onOff);
	bool cmdLOGservos(bool onOff);
	bool cmdLOGstepper(bool onOff);
	bool cmdLOGencoder(bool onOff);
	bool cmdLOGtest(bool onOff);

	bool cmdINFO();

	void computeChecksum(string s,uint8_t& hash);
	void logFetcher();

	SerialPort serialCmd; 			// serial port to transfer commands
	SerialPort serialLog; 			// serial port to suck log output from uC

	ErrorCodeType errorCode;		// global error code. Not nice, but works fine
	LEDState ledState;	 			// current state of LED (not necessarily transfered)
	bool ledStatePending;			// true, if LED state needs to be transfered to uC
	bool powerOn;


	std::thread* logSuckingThread;	// thread that sucks in all uC logs and merges into our log
	int logSuckingThreadState;		// status of logging thread

	ActuatorStateType currActState[NumberOfActuators];
	bool withChecksum;
	bool logMCToConsole = false;
};

#endif /* MICROCONTROLLERINTERFACE_H_ */
