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

class MicroControllerInterface {
public:

	enum LEDState { LED_ON, LED_OFF, LED_BLINKS };

	MicroControllerInterface() {
		ledStatePending = 0;
		ledState = LED_BLINKS;
	}
	static MicroControllerInterface& getInstance() {
		static MicroControllerInterface instance;
		return instance;
	}
	void setup();

	void setLEDState(LEDState state);


	void sendIfValueHasChanged() {
		if (dataIsPending)
			send();
	}

	void send();
	bool receive(string& str);
	void sendString(string str);

private:
	void computeChecksum(string s,uint8_t& hash);

	bool dataIsPending;
	bool powerOn;

	LEDState ledState;
	bool ledStatePending;
	SerialPort serialPort;

};

#endif /* MICROCONTROLLERINTERFACE_H_ */
