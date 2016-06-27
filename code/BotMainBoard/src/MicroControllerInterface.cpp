/*
 * MicroControllerInterface.cpp
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#include "MicroControllerInterface.h"

#include "unistd.h"
#include "setup.h"
#include "SerialPort.h"

using namespace std;

void MicroControllerInterface::setup() {
	serialPort.connect("COM1", 115200);
}

void MicroControllerInterface::setLEDState(LEDState state) {
	if (state != ledState)
		ledStatePending = true;
	ledState = state;
}

void MicroControllerInterface::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

void MicroControllerInterface::send() {
	string cmd = "";

	if (ledStatePending) {
		cmd.append("LED ");
		switch(ledState) {
			case LED_ON: 		cmd.append("on");break;
			case LED_OFF: 		cmd.append("off");break;
			case LED_BLINKS: 	cmd.append("blink");break;
			default:
				break;
		}

		sendString(cmd);
	}
}

void MicroControllerInterface::sendString(string str) {
	uint8_t hash = 0;
	computeChecksum(str, hash);
	serialPort.sendString(str);
}


bool MicroControllerInterface::receive(string& str) {
	int bytesRead = serialPort.receive(str);
	return bytesRead != 0;
}
