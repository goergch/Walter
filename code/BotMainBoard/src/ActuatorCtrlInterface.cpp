/*
 * MicroControllerInterface.cpp
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */

#include "ActuatorCtrlInterface.h"

#include "unistd.h"
#include "setup.h"
#include "SerialPort.h"
#include <sstream>
#include <iostream>
#include <string>
#include "Util.h"

using namespace std;

bool ActuatorCtrlInterface::setup() {
	int error = serialPort.connect(ACTUATOR_CTRL_SERIAL_PORT, ACTUATOR_CTRL_BAUD_RATE);
	if (error != 0) {
		cout << "connecting to " << ACTUATOR_CTRL_SERIAL_PORT << "(" << ACTUATOR_CTRL_BAUD_RATE << ") failed(" << error << ")" << endl;
		return false;
	}

	// try first communication
	int challenge = randomInt(0,99);
	string challengeStr;
	challengeStr += challenge;
	string testStr = "echo " + challengeStr;
	testStr += "\r\n";
	serialPort.sendString(testStr);
	string reponseStr;
	bool ok = receive(reponseStr);

	return true;
}

bool ActuatorCtrlInterface::responseOK(string &s) {
	if (s.substr(s.length()-responseOK.length(), s.length()) == responseOK)
		return true;
	errorCode = 99;
	int errorcodeIdx = s.findstr(reponseNOK);
	if (errorcodeIdx >= 0) {
		int errorcodeEndIdx = s.findstr(")");
		if (errorcodeEndIdx <= errorcodeIdx)
			errorcodeEndIdx = errorcodeIdx;
		string errorStr = s.substr(errorCodeIdx,errorcodeEndIdx-errorCodeIdx);
		errorCode = atoi(errorStr);
	}
	return false;
}


void ActuatorCtrlInterface::setLEDState(LEDState state) {
	if (state != ledState)
		ledStatePending = true;
	ledState = state;
}

void ActuatorCtrlInterface::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

void ActuatorCtrlInterface::send() {
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

void ActuatorCtrlInterface::sendString(string str) {
	uint8_t checksum = 0;
	computeChecksum(str, checksum);

	// add checksum to string (std::toString does not work with ming)
	std::ostringstream ss;
    ss << checksum;
	str.append(ss.str());

	serialPort.sendString(str);
}


bool ActuatorCtrlInterface::receive(string& str, int& errorCode) {
	const string reponseOK =".ok.";
	const string reponseNOK =".nok";

	string rawReponse;
	int bytesRead = serialPort.receive(rawReponse);

	if (rawReponse.substr(rawReponse.length()-responseOK.length(), rawReponse.length()) == responseOK) {
		str = rawReponse.substr(0,rawReponse.length()-responseOK.length());
		return true;
	}

	str = "";
	errorCode = 99;
	int errorcodeIdx = rawReponse.findstr(reponseNOK);
	if (errorcodeIdx >= 0) {
		int errorcodeEndIdx = rawReponse.findstr(")");
		if (errorcodeEndIdx <= errorcodeIdx)
			errorcodeEndIdx = errorcodeIdx;
		string errorStr = rawReponse.substr(errorCodeIdx,errorcodeEndIdx-errorCodeIdx);
		errorCode = atoi(errorStr);
	}
	return false;
}
