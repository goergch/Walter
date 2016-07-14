/*
 * MicroControllerInterface.h
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#ifndef MICROCONTROLLERINTERFACE_H_
#define MICROCONTROLLERINTERFACE_H_

#include "setup.h"
#include "SerialPort.h"
#include "string.h"

using namespace std;

#include "CommDef.h"

class ActuatorCtrlInterface {

public:
	enum LEDState { LED_ON, LED_OFF, LED_BLINKS };
	enum ErrorCodeType:int { NO_ERROR_CODE = 0, CHECKSUM_EXPECTED = 1, CHECKSUM_WRONG = 2,
					 PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5, CMD_ERROR = 6, NO_RESPONSE_CODE= 7};



	ActuatorCtrlInterface() {
		errorCode = NO_ERROR_CODE;
		powerOn = false;
		ledState = LED_OFF;
		ledStatePending = true;
	}
	static ActuatorCtrlInterface& getInstance() {
		static ActuatorCtrlInterface instance;
		return instance;
	}
	bool setup();

	void send();
	bool receive(string& str, int timeout_ms);
	bool checkReponseCode(string &s, string& plainResponse, bool &reponseCodeRead);

	void sendString(string str);
	ErrorCodeType getError();
	bool isError();

	void setLEDState(LEDState state);

private:
	bool cmdLED(LEDState state);
	bool cmdECHO(string s);
	bool cmdCHECKSUM(bool onOff);
	bool cmdPOWER(bool onOff);
	bool cmdDISABLE();
	bool cmdENABLE();
	bool cmdMOVETO(float angle[7], int duration_ms);
	bool cmdGET(int ActuatorNo, float& curr, float& min, float &max, float &nullAngle);
	bool cmdSET(int ActuatorNo, float minAngle, float maxAngle, float nullAngle);
	bool cmdSTEP(int actuatorID, float incr);
	bool cmdMEMReset();
	bool cmdMEMList(string &result);
	bool cmdKNOB(bool useAbs);
	bool cmdLOGsetup(bool onOff);
	bool cmdLOGservos(bool onOff);
	bool cmdLOGstepper(bool onOff);
	bool cmdLOGencoder(bool onOff);
	bool cmdINFO();



	void computeChecksum(string s,uint8_t& hash);
	bool powerOn;

	SerialPort serialPort;
	ErrorCodeType errorCode;
	LEDState ledState;
	bool ledStatePending;
};

#endif /* MICROCONTROLLERINTERFACE_H_ */
