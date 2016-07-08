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

class ActuatorCtrlInterface{
public:

	enum LEDState { LED_ON, LED_OFF, LED_BLINKS };

	ActuatorCtrlInterface() {
		ledStatePending = 0;
		ledState = LED_BLINKS;
	}
	static ActuatorCtrlInterface& getInstance() {
		static ActuatorCtrlInterface instance;
		return instance;
	}
	bool setup();

	void setLEDState(LEDState state);

	void send();
	bool receive(string& str);
	void sendString(string str);

	enum errorCode { NO_ERROR = 0, CHECKSUM_EXPECTED = 1, CHECKSUM_WRONG = 2,
					 PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5, CMD_ERROR = 6 };
private:
	void computeChecksum(string s,uint8_t& hash);
	bool dataIsPending;
	bool powerOn;

	LEDState ledState;
	bool ledStatePending;
	SerialPort serialPort;
	int errorCode;

};

#endif /* MICROCONTROLLERINTERFACE_H_ */
