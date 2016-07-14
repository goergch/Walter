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
#include <unistd.h>

using namespace std;

const string reponseOKStr =">ok";
const string reponseNOKStr =">nok(";
const string newlineStr = "\r";
const int receiveTimeOut = 100;


ActuatorCtrlInterface::ErrorCodeType ActuatorCtrlInterface::getError() {
	return errorCode;
}

bool ActuatorCtrlInterface::isError() {
	return errorCode != NO_ERROR_CODE;
}

bool ActuatorCtrlInterface::setup() {
	int error = serialPort.connect(ACTUATOR_CTRL_SERIAL_PORT, ACTUATOR_CTRL_BAUD_RATE);
	if (error != 0) {
		cout << "connecting to " << ACTUATOR_CTRL_SERIAL_PORT << "(" << ACTUATOR_CTRL_BAUD_RATE << ") failed(" << error << ")" << endl;
		return false;
	}

	// try first communication
	int challenge = randomInt(10,99);
	string challengeStr= ITOS(challenge);
	string testStr = "echo " + challengeStr;
	sendString(testStr);
	awaitCommandExecution(TEST_CMD);
	string reponseStr;
	bool ok = receive(reponseStr, receiveTimeOut);

	if (!ok)
		return false;

	// switch checksum on
	sendString("checksum on");
	awaitCommandExecution(CHECKSUM_CMD);

	ok = receive(reponseStr, receiveTimeOut);
	if (!ok)
		return false;

	return ok;
}

bool ActuatorCtrlInterface::checkReponseCode(string &s, string &plainReponse, bool &reponseCodeRead) {
	reponseCodeRead = false; // are we able to read ok or nok ?
	if (s.compare(s.length()-reponseOKStr.length(), reponseOKStr.length(), reponseOKStr) == 0) {
		plainReponse = s.substr(0,s.length()-reponseOKStr.length());
		reponseCodeRead = true;
		return true;
	}

	errorCode = ActuatorCtrlInterface::NO_RESPONSE_CODE;
	int start = s.length()-reponseNOKStr.length()-4;
	if (start < 0)
		start = 0;
	int errorcodeIdx = s.find(reponseNOKStr,start);
	if (errorcodeIdx >= 0) {
		reponseCodeRead = true;
		int errorcodeEndIdx = s.find(")",errorcodeIdx+1);
		if (errorcodeEndIdx <= errorcodeIdx)
			errorcodeEndIdx = errorcodeIdx;
		string errorStr = s.substr(errorcodeIdx+reponseNOKStr.length(),errorcodeEndIdx-errorcodeIdx-1);
		int code = atoi(errorStr.c_str());
		errorCode = (ActuatorCtrlInterface::ErrorCodeType)code;
		plainReponse = s.substr(0,s.length()-errorcodeIdx);

	}
	return false;
}

void ActuatorCtrlInterface::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

void ActuatorCtrlInterface::setLEDState(LEDState state) {
	if (state != ledState)
		ledStatePending = true;
	ledState = state;
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
		awaitCommandExecution(ActuatorCtrlInterface::LED_CMD);

	}
}

void ActuatorCtrlInterface::sendString(string str) {
	uint8_t checksum = 0;
	computeChecksum(str, checksum);

	// add checksum to string
	str += ITOS(checksum);
	str += newlineStr;

	serialPort.sendString(str);

}

void ActuatorCtrlInterface::awaitCommandExecution(cmdType cmd) {
	int waitDuringExecution = cmdExecutionTime[cmd];
	delay(waitDuringExecution);
}

bool ActuatorCtrlInterface::receive(string& str, int timeout_ms) {


	string response;
	string reponsePayload;
	long startTime = millis();
	int bytesRead;
	bool ok = false;
	bool okOrNOK = false;

	// read from serial until "ok" or "nok" has been read or timeout occurs
	do {
		string rawResponse;
		bytesRead = serialPort.receive(rawResponse);
		if (bytesRead > 0) {
			response += rawResponse;
			ok = checkReponseCode(rawResponse, reponsePayload,okOrNOK);
		}
	}
	while ((millis() - startTime < timeout_ms) && (!okOrNOK));

	if (okOrNOK)
		str = reponsePayload;

	return ok;
}
