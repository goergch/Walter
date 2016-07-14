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
#include "CommDef.h"
#include "easylogging++.h"
#include <iomanip>
using namespace std;

const string reponseOKStr =">ok";
const string reponseNOKStr =">nok(";
const string newlineStr = "\r";
const int receiveTimeOut = 100;

void cmdHELP() {};
void cmdLED() {};
void cmdPOWER(){};
void cmdECHO(){};
void cmdMOVETO(){};
void cmdDISABLE(){};
void cmdENABLE(){};
void cmdGET(){};
void cmdSET(){};
void cmdMOVE(){};
void cmdMEM(){};
void cmdCHECKSUM(){};
void cmdKNOB(){};
void cmdLOG(){};
void cmdHELP();
void cmdINFO(){};

bool ActuatorCtrlInterface::cmdLED(LEDState state) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LED_CMD);

	cmd.append(comm->name);
	switch(ledState) {
		case LED_ON: 		cmd.append("on");break;
		case LED_OFF: 		cmd.append("off");break;
		case LED_BLINKS: 	cmd.append("blink");break;
		default:
			break;
	}

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdECHO(string s) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::ECHO_CMD);
	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(s);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdMEMReset() {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::MEM_CMD);
	cmd.append(comm->name);
	cmd.append(" reset");
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdMEMList(string& mem) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::MEM_CMD);
	cmd.append(comm->name);
	cmd.append(" list");
	sendString(cmd);
	bool ok = receive(mem, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdPOWER(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::POWER_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" on");
	else
		cmd.append(" off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdCHECKSUM(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::CHECKSUM_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" on");
	else
		cmd.append(" off");

	serialPort.sendString(cmd); // send without checksum
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdDISABLE() {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::DISABLE_CMD);

	cmd.append(comm->name);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdENABLE() {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::ENABLE_CMD);

	cmd.append(comm->name);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}


bool ActuatorCtrlInterface::cmdMOVETO(float angle[7], int duration_ms) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::MOVETO_CMD);

	cmd.append(comm->name);
	for (int i = 0;i<7;i++) {
	    cmd.append(static_cast< std::ostringstream & >(( std::ostringstream() << " " << std::setprecision(2) <<  angle[i] ) ).str());

	}
	cmd.append(" ");
	cmd.append(std::to_string(duration_ms));

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdSTEP(int actuatorID, float incr) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::STEP_CMD);

	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(std::to_string(actuatorID));
	cmd.append(static_cast< std::ostringstream & >(( std::ostringstream() << " " << std::setprecision(incr) <<  incr) ).str());

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdSET(int ActuatorNo, float minAngle, float maxAngle, float nullAngle) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::SET_CMD);

	cmd.append(comm->name);
    cmd.append(static_cast< std::ostringstream & >(( std::ostringstream() << " " << std::setprecision(2) <<  minAngle ) ).str());
    cmd.append(static_cast< std::ostringstream & >(( std::ostringstream() << " " << std::setprecision(2) <<  maxAngle ) ).str());
    cmd.append(static_cast< std::ostringstream & >(( std::ostringstream() << " " << std::setprecision(2) <<  nullAngle ) ).str());

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdGET(int ActuatorNo, float &curr, float &min, float &max, float &nullAngle) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::GET_CMD);

	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(std::to_string(ActuatorNo));


	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);

	// format: 	a=1.0 min=1.0 max=1.0 null=1.0
	std::istringstream is;
	string token;
	is.str(reponseStr);
	std::getline(is, token, '=');
	is >> curr;
	std::getline(is, token, '=');
	is >> min;
	std::getline(is, token, '=');
	is >> max;
	std::getline(is, token, '=');
	is >> nullAngle;

	return ok;
}

bool ActuatorCtrlInterface::cmdLOGsetup(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" setup on");
	else
		cmd.append(" setup off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;

}
bool ActuatorCtrlInterface::cmdLOGservos(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" servo on");
	else
		cmd.append(" servo off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdLOGstepper(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" stepper on");
	else
		cmd.append(" stepper off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}
bool ActuatorCtrlInterface::cmdLOGencoder(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" encoder on");
	else
		cmd.append(" encoder off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdINFO() {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::INFO_CMD);

	cmd.append(comm->name);

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);

	return ok;
}



ActuatorCtrlInterface::ErrorCodeType ActuatorCtrlInterface::getError() {
	return errorCode;
}

bool ActuatorCtrlInterface::isError() {
	return errorCode != NO_ERROR_CODE;
}

bool ActuatorCtrlInterface::setup() {
	LOG(DEBUG) << "entering ActuatorCtrlInterface::setup";

	int error = serialPort.connect(ACTUATOR_CTRL_SERIAL_PORT, ACTUATOR_CTRL_BAUD_RATE);
	if (error != 0) {
		LOG(ERROR) << "connecting to " << ACTUATOR_CTRL_SERIAL_PORT << "(" << ACTUATOR_CTRL_BAUD_RATE << ") failed(" << error << ")" << endl;
		return false;
	}

	// try first communication
	int challenge = randomInt(10,99);
	string challengeStr= ITOS(challenge);
	bool ok = cmdECHO(challengeStr);
	if (!ok)
		return false;

	// switch checksum on
	ok = cmdCHECKSUM(true);
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
	bool ok = true;
	if (ledStatePending) {
		ok = cmdLED(ledState);
	}
	if (!ok) {
		cerr << "sending failed (" << getError() << ")" << endl;
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

	LOG_IF(ok, DEBUG) << "reponse \"" << response << "\" -> " << (okOrNOK?"OK":"NOK") <<  endl;

	if (okOrNOK)
		str = reponsePayload;

	return ok;
}
