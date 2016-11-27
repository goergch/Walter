/*
 * MicroControllerInterface.cpp
 *
 *  Created on: 02.03.2015
 *      Author: JochenAlt
 */


#include <iostream>
#include <thread>
#include <chrono>
#include "unistd.h"
#include "setup.h"
#include <sstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <iomanip>

#include "ActuatorCtrlInterface.h"

#include "SerialPort.h"
#include "CommDef.h"
#include "Util.h"

using namespace std;

const string reponseOKStr =">ok\r\n>";	 // reponse code from uC: >ok or >nok(errornumber)
const string reponseNOKStr =">nok(";

// the following functions are dummys, real functions are used in the uC. Purpose is to have
// one communication interface header between uC and host containing all commands. uC uses a
// library that works with function pointers to parse the commands, here we use regular method calls,
// since in most cases we send in fire-and-forget style
void cmdHELP() {};
void cmdSTEP() {};
void cmdLED() {};
void cmdPOWER(){};
void cmdECHO(){};
void cmdMOVETO(){};
void cmdDISABLE(){};
void cmdSETUP(){};
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


bool ActuatorCtrlInterface::microControllerPresent(string cmd) {
	if (!microControllerOk) {
		LOG(ERROR) << "microController not present, " << cmd << " failed";
		return false;
	}
	return true;
}
bool ActuatorCtrlInterface::cmdLED(LEDState state) {
	if (!microControllerPresent("cmdLED"))
		return false;

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
	if (!microControllerPresent("cmdMEMReset"))
		return false;

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
	if (!microControllerPresent("cmdMEMList"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::MEM_CMD);
	cmd.append(comm->name);
	cmd.append(" list");
	sendString(cmd);
	bool ok = receive(mem, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdPOWER(bool onOff) {
	if (!microControllerPresent("cmdPOWER"))
		return false;

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

	serialCmd.sendString(cmd); // send without checksum
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	if (ok)
		withChecksum = onOff;
	return ok;
}

bool ActuatorCtrlInterface::cmdSETUP() {
	if (!microControllerPresent("cmdSETUP"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::SETUP_CMD);

	cmd.append(comm->name);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}


bool ActuatorCtrlInterface::cmdDISABLE() {
	if (!microControllerPresent("cmdDISABLE"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::DISABLE_CMD);

	cmd.append(comm->name);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdENABLE() {
	if (!microControllerPresent("cmdENABLE"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::ENABLE_CMD);

	cmd.append(comm->name);
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}


bool ActuatorCtrlInterface::cmdMOVETO(JointAngles angle_rad, int duration_ms) {
	if (!microControllerPresent("cmdMOVETO"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::MOVETO_CMD);

	cmd.append(comm->name);
	for (int i = 0;i<7;i++) {
	    cmd.append(" ");
	    rational angle_deg = degrees(angle_rad[i]);
		cmd.append(to_string(angle_deg,2));
	}
	cmd.append(" ");
	cmd.append(std::to_string(duration_ms));

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdSTEP(int actuatorID, rational incr_rad) {
	if (!microControllerPresent("cmdSTEP"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::STEP_CMD);

	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(std::to_string(actuatorID));
	cmd.append(" ");
	cmd.append(to_string(degrees(incr_rad),2));

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdSET(int ActuatorNo, rational minAngle, rational maxAngle, rational nullAngle) {
	if (!microControllerPresent("cmdSET"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::SET_CMD);

	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(to_string(degrees(minAngle),2));
	cmd.append(" ");
	cmd.append(to_string(degrees(maxAngle),2));
	cmd.append(" ");
	cmd.append(to_string(degrees(nullAngle),2));
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdGETall(ActuatorStateType actuatorState[]) {
	if (!microControllerPresent("cmdGETall"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::GET_CMD);

	cmd.append(comm->name);
	cmd.append(" all");
	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	std::istringstream is(reponseStr);
	string token;
	std::stringstream ta;
	ta.precision(2);

	std::string out = "";
	out += ta.str() + "\n";

	// format: 	ang=1.0 min=1.0 max=1.0 null=1.0
	for (int i = 0;i<NumberOfActuators;i++) {
		int idx;
		std::getline(is, token, '=');
		is >> idx;
		if (idx != i) {
			LOG(ERROR) << "get all reply inconsistent (" << i << " != " << idx << ")";
			return false;
		}
		std::getline(is, token, '=');
		float currentAngle, minAngle, maxAngle,nullAngle;
		is >> currentAngle;
		std::getline(is, token, '=');
		is >> minAngle;
		std::getline(is, token, '=');
		is >> maxAngle;
		std::getline(is, token, '=');
		is >> nullAngle;

		// convert to radian
		actuatorState[i].currentAngle = radians(currentAngle);
		actuatorState[i].minAngle = radians(minAngle);
		actuatorState[i].maxAngle = radians(maxAngle);
		actuatorState[i].nullAngle = radians(nullAngle);

	}
	return ok;

}

bool ActuatorCtrlInterface::cmdGET(int actuatorNo, ActuatorStateType actuatorState) {
	if (!microControllerPresent("cmdGET"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::GET_CMD);

	cmd.append(comm->name);
	cmd.append(" ");
	cmd.append(std::to_string(actuatorNo));


	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);

	// format: 	ang=1.0 min=1.0 max=1.0 null=1.0
	std::istringstream is(reponseStr);
	string token;
	is.str(reponseStr);
	std::getline(is, token, '=');
	is >> actuatorState.currentAngle;
	std::getline(is, token, '=');
	is >> actuatorState.minAngle;
	std::getline(is, token, '=');
	is >> actuatorState.maxAngle;
	std::getline(is, token, '=');
	is >> actuatorState.nullAngle;

	actuatorState.currentAngle = radians(actuatorState.currentAngle);
	actuatorState.minAngle = radians(actuatorState.minAngle);
	actuatorState.maxAngle = radians(actuatorState.maxAngle);
	actuatorState.nullAngle = radians(actuatorState.nullAngle);

	return ok;
}

bool ActuatorCtrlInterface::cmdLOGsetup(bool onOff) {
	if (!microControllerPresent("cmdLOGsetup"))
		return false;

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

bool ActuatorCtrlInterface::cmdLOGtest(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	if (onOff)
		cmd.append(" test on");
	else
		cmd.append(" test off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdLOGservos(bool onOff) {
	if (!microControllerPresent("cmdLOGservos"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	cmd.append(onOff?" servo on":" servo off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdLOGstepper(bool onOff) {
	if (!microControllerPresent("cmdLOGstepper"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	cmd.append(onOff?" stepper on":" stepper off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}
bool ActuatorCtrlInterface::cmdLOGencoder(bool onOff) {
	if (!microControllerPresent("cmdLOGencoder"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

	cmd.append(comm->name);
	cmd.append(onOff?" encoder on":" encoder off");

	sendString(cmd);
	string reponseStr;
	bool ok = receive(reponseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool ActuatorCtrlInterface::cmdINFO(bool &powered, bool& setuped, bool &enabled) {
	if (!microControllerPresent("cmdINFO"))
		return false;

	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::INFO_CMD);

	cmd.append(comm->name);

	sendString(cmd);
	string responseStr;
	bool ok = receive(responseStr, comm->expectedExecutionTime_ms);

	powered = (responseStr.find("powered")!=std::string::npos);
	setuped = (responseStr.find("setuped")!=std::string::npos);
	enabled = (responseStr.find("enabled")!=std::string::npos);
	return ok;
}



ActuatorCtrlInterface::ErrorCodeType ActuatorCtrlInterface::getError() {
	return errorCode;
}

bool ActuatorCtrlInterface::isError() {
	return errorCode != NO_ERROR_CODE;
}

void ActuatorCtrlInterface::logFetcher() {

	string currentLine;
	string str;

	while (true) {
		int bytesRead = serialLog.receive(str);

		if (bytesRead > 0) {
			currentLine.append(str);
			// log full lines only
			int endOfLineIdx = currentLine.find("\r", 0);
			if (endOfLineIdx > 0) {
				string line = currentLine.substr(0,endOfLineIdx);
				currentLine = currentLine.substr(endOfLineIdx+1);

				if (line[0] == '\r')
					line = line.substr(1);
				if (line[0] == '\n')
					line = line.substr(1);

				replaceWhiteSpace(line);

 				if (logMCToConsole)
					cout << "log>" << line << endl;

 				LOG(TRACE) << line;
				logSuckingThreadState = 1; // a log line has been detected, state success!
			}
		}
		delay(10);
	}
}

bool ActuatorCtrlInterface::setupCommunication() {
	LOG(DEBUG) << "entering ActuatorCtrlInterface::setup";

	LOG(DEBUG) << "log sucking thread started";
	logSuckingThreadState = 0;

	int error = serialLog.connect(ACTUATOR_CTRL_LOGGER_PORT, ACTUATOR_CTRL_LOGGER_BAUD_RATE);
	if (error != 0) {
		LOG(ERROR) << "connecting to " << ACTUATOR_CTRL_LOGGER_PORT << "(" << ACTUATOR_CTRL_LOGGER_BAUD_RATE << ") failed(" << error << ")" << endl;
		return false;
	}

	// now start command interface
	error = serialCmd.connect(ACTUATOR_CTRL_SERIAL_PORT , ACTUATOR_CTRL_BAUD_RATE);
	if (error != 0) {
		LOG(ERROR) << "connecting to " << ACTUATOR_CTRL_SERIAL_PORT << "(" << ACTUATOR_CTRL_BAUD_RATE << ") failed(" << error << ")" << endl;
		return false;
	}

	// start log fetching thread
	logSuckingThread = new std::thread(&ActuatorCtrlInterface::logFetcher, this);
	delay(1);

	bool ok = cmdLOGtest(true); // writes a log entry
	if (!ok) {
		if (errorCode == CHECKSUM_EXPECTED) {
			// try with checksum, uC must have been started earlier with checksum set
			withChecksum= true;
			ok = cmdLOGtest(true);
		}
	}
	if (!ok) {
		LOG(ERROR) << "switch on uC logger failed(" << errorCode << ")";
		return false;
	}
	// wait at most 100ms for log entry
	unsigned long startTime  = millis();
	do { delay(1); }
	while ((millis() - startTime < 1000) && (logSuckingThreadState != 1));

	if (logSuckingThreadState != 1) {
		LOG(ERROR) << "log thread failed";
		return false;
	}


	// try first communication and check correct reponse
	int challenge = randomInt(10,99);
	string challengeStr= std::to_string(challenge);
	ok = cmdECHO(challengeStr);
	if (!ok) {
		LOG(ERROR) << "challenge/reponse to uC failed";
		return false;
	}

	// switch checksum on
	ok = cmdCHECKSUM(true);
	if (!ok) {
		LOG(ERROR) << "switching on checksum to uC failed";
		return false;
	}

	// try second communication with checksum
	challenge = randomInt(10,99);
	challengeStr= std::to_string(challenge);
	ok = cmdECHO(challengeStr);
	if (!ok) {
		LOG(ERROR) << "challenge/reponse with checksum to uC failed";
		return false;
	}

	if (logSuckingThreadState != 1) {
		LOG(ERROR) << "logging interface could not be established";
		return false;
	}

	if (ok)
		microControllerOk = true;
	return ok;
}

// reponse code of uC is >ok or >nok(error). Parse this, return true if ok or nok has been parsed,
// extract the remaining payload (plainReponse) and return a flag weather ok or nok has been parsed
bool ActuatorCtrlInterface::checkReponseCode(string &s, string &plainReponse, bool &OkOrNOk) {
	OkOrNOk = false; // are we able to read ok or nok ?
	int startOkSearchIdx = s.length()-reponseOKStr.length();
	if ((startOkSearchIdx >= 0) && (s.compare(startOkSearchIdx, reponseOKStr.length(), reponseOKStr) == 0)) {
		plainReponse = s.substr(0,s.length()-reponseOKStr.length());
		OkOrNOk = true;
		return true;
	}

	errorCode = ActuatorCtrlInterface::NO_RESPONSE_CODE;
	int start = s.length()-reponseNOKStr.length()-5;
	if (start < 0)
		start = 0;
	int errorcodeIdx = s.find(reponseNOKStr,start);
	if (errorcodeIdx >= 0) {
		OkOrNOk = false;
		int errorcodeEndIdx = s.find(")",errorcodeIdx+1);
		if (errorcodeEndIdx <= errorcodeIdx)
			errorcodeEndIdx = errorcodeIdx;
		string errorStr = s.substr(errorcodeIdx+reponseNOKStr.length(),errorcodeEndIdx-errorcodeIdx-reponseNOKStr.length());
		int code = atoi(errorStr.c_str());
		errorCode = (ActuatorCtrlInterface::ErrorCodeType)code;
		plainReponse = s.substr(0,errorcodeIdx);
		return true;
	}
	return false;
}

void ActuatorCtrlInterface::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		if (c != ' ')
			hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

void ActuatorCtrlInterface::setLEDState(LEDState state) {
	if (state != ledState)
		ledStatePending = true;
	ledState = state;
}

bool  ActuatorCtrlInterface::setupBot() {
	LOG(INFO) << "setup bot";
	return cmdSETUP();
}

bool ActuatorCtrlInterface::enableBot() {
	LOG(INFO) << "enable bot";
	return cmdENABLE();
}

bool ActuatorCtrlInterface::disableBot() {
	LOG(INFO) << "disable bot";
	return cmdDISABLE();
}


bool ActuatorCtrlInterface::info(bool &powered, bool& setuped, bool &enabled) {
	LOG(INFO) << "get info";
	return cmdINFO(powered,setuped, enabled);
}


bool ActuatorCtrlInterface::getAngles(ActuatorStateType actuatorState[]) {
	LOG(INFO) << "get angles";

	bool ok = cmdGETall(currActState);

	LOG(DEBUG) << "angles =(" << setprecision(2) <<
			currActState[0].currentAngle << " " <<
			currActState[1].currentAngle << " " <<
			currActState[2].currentAngle << " " <<
			currActState[3].currentAngle << " " <<
			currActState[4].currentAngle << " " <<
			currActState[5].currentAngle << " " <<
			currActState[6].currentAngle << ")";

	for (int i = 0;i<7;i++)
		actuatorState[i] = currActState[i];
	return ok;
}

bool ActuatorCtrlInterface::power(bool onOff) {
	LOG(INFO) << "power (" << onOff << ")";

	bool ok;
	if (onOff) {
		ok = cmdPOWER(true);
		ok = ok && cmdENABLE();
	} else {
		ok = cmdDISABLE();
		ok = ok && cmdPOWER(false);
		botIsUpAndRunning = false;
	}
	return ok;
}

bool ActuatorCtrlInterface::move(JointAngles angle_rad, int duration_ms) {
	LOG(INFO) << "move to " << setprecision(1) << angle_rad;
	return cmdMOVETO(angle_rad, duration_ms);
}

void ActuatorCtrlInterface::directAccess(string cmd, string& response, bool &okOrNOk) {
	sendString(cmd);
	okOrNOk = receive(response, 3000);
}

void ActuatorCtrlInterface::loop() {
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

	if (withChecksum) {
		// add checksum to string
		str +=" chk=";
		str += std::to_string(checksum); // ITOS(checksum);
	}

	serialCmd.sendString(str);
}



bool ActuatorCtrlInterface::receive(string& str, int timeout_ms) {
	string response="";
	string reponsePayload="";
	unsigned long startTime = millis();
	int bytesRead;
	bool replyIsOk = false; // does not necessarily mean that the answer is "ok", we only received a reply which we can read
	bool okOrNOK = false;

	// read from serial until "ok" or "nok" has been read or timeout occurs
	int count= 0;// check two times at least (makes debugging easier)
	string rawResponse ="";
	do {
		bytesRead = serialCmd.receive(rawResponse);
		if (bytesRead > 0) {
			response += rawResponse;
			replyIsOk = checkReponseCode(response, reponsePayload,okOrNOK);
		} else
			delay(1);
	}
	while (((count++ < 2) || (millis() - startTime < (unsigned long)timeout_ms)) && (!replyIsOk));

	LOG_IF(!replyIsOk, ERROR) << "response \"" << rawResponse << "|" << response << "|" << reponsePayload << "\" could not be parsed";

	LOG_IF(replyIsOk, DEBUG) << "response \""
			<< replaceWhiteSpace(reponsePayload)
			<< "\" & " << (okOrNOK?"OK":"NOK(" + int_to_string(errorCode) + ")");

	if (okOrNOK)
		str = reponsePayload;
	else
		str = "";

	return okOrNOK;
}
