/*
 * CortexController.cpp
 *
 *
 * Author: JochenAlt
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

#include "core.h"
#include "CommDef.h"

#include "CortexController.h"
#include "CmdDispatcher.h"

#include "SerialPort.h"
#include "logger.h"
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
void cmdDISABLE(){};
void cmdSETUP(){};
void cmdENABLE(){};
void cmdGET(){};
void cmdSET(){};
void cmdMOVETO(){};
void cmdMEM(){};
void cmdCHECKSUM(){};
void cmdKNOB(){};
void cmdLOG(){};
void cmdHELP();
void cmdINFO(){};
void cmdPRINT(){};
void cmdPRINTLN(){};


bool CortexController::microControllerPresent(string cmd) {
	resetError();
	if (!microControllerOk) {
		setError(CORTEX_CONNECTION_FAILED);
		LOG(ERROR) << "microController not present, " << cmd << " failed";
	}
	return !isError();
}
bool CortexController::cmdLED(LEDState state) {
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

	string responseStr;
	bool ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	return ok;
}

bool CortexController::cmdECHO(string s) {
	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::ECHO_CMD);
		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(s);
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdMEMReset() {
	if (!microControllerPresent("cmdMEMReset"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::MEM_CMD);
		cmd.append(comm->name);
		cmd.append(" reset");
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdMEMList(string& mem) {
	if (!microControllerPresent("cmdMEMList"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::MEM_CMD);
		cmd.append(comm->name);
		cmd.append(" list");
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}

bool CortexController::cmdPOWER(bool onOff) {
	if (!microControllerPresent("cmdPOWER"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::POWER_CMD);

		cmd.append(comm->name);
		if (onOff)
			cmd.append(" on");
		else
			cmd.append(" off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
		if (ok)
			powered = onOff;
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdCHECKSUM(bool onOff) {
	string cmd = "";
	CommDefType* comm = CommDefType::get(CommDefType::CommandType::CHECKSUM_CMD);

	bool ok = false;
	do {
		cmd.append(comm->name);
		if (onOff)
			cmd.append(" on");
		else
			cmd.append(" off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
		if (ok)
			withChecksum = onOff;
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdSETUP() {
	if (!microControllerPresent("cmdSETUP"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::SETUP_CMD);

		cmd.append(comm->name);
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
		setup = ok;
	} while (retry(ok));

	return ok;
}


bool CortexController::cmdDISABLE() {
	if (!microControllerPresent("cmdDISABLE"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::DISABLE_CMD);

		cmd.append(comm->name);
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
		enabled = false;
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdENABLE() {
	if (!microControllerPresent("cmdENABLE"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::ENABLE_CMD);

		cmd.append(comm->name);
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
		enabled = ok;
	} while (retry(ok));

	return ok;
}


bool CortexController::cmdMOVETO(JointAngles angle_rad, int duration_ms) {
	if (!microControllerPresent("cmdMOVETO"))
		return false;
	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::MOVETO_CMD);

		cmd.append(comm->name);
		for (int i = 0;i<7;i++) {
			cmd.append(" ");
			rational angle_deg = degrees(angle_rad[i]);
			string angleStr = string_format("%.2f",angle_deg);
			cmd.append(angleStr);
		}
		cmd.append(" ");
		cmd.append(std::to_string(duration_ms));

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}

bool CortexController::retry(bool replyOk) {
	if ((!replyOk) && (communicationFailureCounter>=3))
		LOG(ERROR) << "3.th failed retry. quitting";

	return ((!replyOk) && (communicationFailureCounter>0) && (communicationFailureCounter<5));
}

bool CortexController::cmdSTEP(int actuatorID, rational incr_rad) {
	if (!microControllerPresent("cmdSTEP"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::STEP_CMD);

		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(std::to_string(actuatorID));
		cmd.append(" ");
		cmd.append(to_string(degrees(incr_rad),2));

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}

bool CortexController::cmdSET(int ActuatorNo, rational minAngle, rational maxAngle, rational nullAngle) {
	if (!microControllerPresent("cmdSET"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::SET_CMD);

		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(to_string(degrees(minAngle),2));
		cmd.append(" ");
		cmd.append(to_string(degrees(maxAngle),2));
		cmd.append(" ");
		cmd.append(to_string(degrees(nullAngle),2));
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}

bool CortexController::cmdGETall(ActuatorStateType actuatorState[]) {
	if (!microControllerPresent("cmdGETall"))
		return false;

	bool ok = false;
	string responseStr;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::GET_CMD);

		cmd.append(comm->name);
		cmd.append(" all");
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	std::istringstream is(responseStr);
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

bool CortexController::cmdGET(int actuatorNo, ActuatorStateType actuatorState) {
	if (!microControllerPresent("cmdGET"))
		return false;

	bool ok = false;
	string reponseStr;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::GET_CMD);

		cmd.append(comm->name);
		cmd.append(" ");
		cmd.append(std::to_string(actuatorNo));

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

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

bool CortexController::cmdLOGsetup(bool onOff) {
	if (!microControllerPresent("cmdLOGsetup"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

		cmd.append(comm->name);
		if (onOff)
			cmd.append(" setup on");
		else
			cmd.append(" setup off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdLOGtest(bool onOff) {
	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

		cmd.append(comm->name);
		if (onOff)
			cmd.append(" test on");
		else
			cmd.append(" test off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdLOGservos(bool onOff) {
	if (!microControllerPresent("cmdLOGservos"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

		cmd.append(comm->name);
		cmd.append(onOff?" servo on":" servo off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}

bool CortexController::cmdLOGstepper(bool onOff) {
	if (!microControllerPresent("cmdLOGstepper"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

		cmd.append(comm->name);
		cmd.append(onOff?" stepper on":" stepper off");

		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));
	return ok;
}
bool CortexController::cmdLOGencoder(bool onOff) {
	if (!microControllerPresent("cmdLOGencoder"))
		return false;

	bool ok = false;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::LOG_CMD);

		cmd.append(comm->name);
		cmd.append(onOff?" encoder on":" encoder off");
		string responseStr;
		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	return ok;
}

bool CortexController::cmdINFO(bool &powered, bool& setuped, bool &enabled) {
	if (!microControllerPresent("cmdINFO"))
		return false;

	bool ok = false;
	string responseStr;
	do {
		string cmd = "";
		CommDefType* comm = CommDefType::get(CommDefType::CommandType::INFO_CMD);

		cmd.append(comm->name);

		ok = callMicroController(cmd, responseStr, comm->expectedExecutionTime_ms);
	} while (retry(ok));

	LOG(INFO) << "cmdINFO:" << responseStr;

	powered = (responseStr.find("powered")!=std::string::npos);
	setuped = (responseStr.find("setuped")!=std::string::npos);
	enabled = (responseStr.find("enabled")!=std::string::npos);
	return ok;
}

void CortexController::logFetcher() {

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

 				// push log message to cmd dispatcher
 				CommandDispatcher::getInstance().addLogLine(line);

 				LOG(TRACE) << line;
				logSuckingThreadState = 1; // a log line has been detected, state success!
			}
		} else
			delay(1);
	}
}

bool CortexController::setupCommunication() {
	LOG(DEBUG) << "entering ActuatorCtrlInterface::setup";

	LOG(DEBUG) << "log sucking thread started";
	logSuckingThreadState = 0;
	serialLog.disconnect();
	bool ok= serialLog.connect(CORTEX_LOGGER_SERIAL_PORT, CORTEX_LOGGER_BAUD_RATE);
	if (!ok) {
		LOG(ERROR) << "connecting to " << CORTEX_LOGGER_SERIAL_PORT << "(" << CORTEX_LOGGER_BAUD_RATE << ") failed";
		setError(CORTEX_LOG_COM_FAILED);
		return false;
	}

	// now start command interface
	serialCmd.disconnect();
	ok = serialCmd.connect(CORTEX_COMMAND_SERIAL_PORT , CORTEX_COMMAND_BAUD_RATE);
	if (!ok) {
		LOG(ERROR) << "connecting to " << CORTEX_COMMAND_SERIAL_PORT << "(" << CORTEX_COMMAND_BAUD_RATE << ") failed";
		setError(CORTEX_COM_FAILED);

		return false;
	}

	// start log fetching thread
	if (logSuckingThread == NULL)
		logSuckingThread = new std::thread(&CortexController::logFetcher, this);
	delay(1);

	 ok = cmdLOGtest(true); // writes a log entry
	if (!ok) {
		if (getLastError() == CHECKSUM_EXPECTED) {
			// try with checksum, uC must have been started earlier with checksum set
			withChecksum= true;
			ok = cmdLOGtest(true);
		}
	}
	if (!ok) {
		LOG(ERROR) << "logger test failed(" << getLastError() << ")";
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
		LOG(ERROR) << "challenge/reponse check failed";
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

	if (ok) {
		microControllerOk = true;
		communicationFailureCounter = 0;
	}
	return ok;
}

bool CortexController::communicationOk() {
	return (microControllerOk && (communicationFailureCounter < 5));
}

// reponse code of uC is >ok or >nok(error). Parse this, return true if ok or nok has been parsed,
// extract the remaining payload (plainReponse) and return a flag weather ok or nok has been parsed
bool CortexController::checkReponseCode(string &s, string &plainReponse, bool &OkOrNOk) {
	resetError();

	OkOrNOk = false; // are we able to read ok or nok ?
	int startOkSearchIdx = s.length()-reponseOKStr.length();
	if ((startOkSearchIdx >= 0) && (s.compare(startOkSearchIdx, reponseOKStr.length(), reponseOKStr) == 0)) {
		plainReponse = s.substr(0,s.length()-reponseOKStr.length());
		OkOrNOk = true;
		return true;
	}

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
		setError((ErrorCodeType)code);
		plainReponse = s.substr(0,errorcodeIdx);
		return true;
	}
	return false;
}

void CortexController::computeChecksum(string s,uint8_t& hash) {
	int c;

	int i = 0;
	while ((c = s[i++])) {
		if (c != ' ')
			hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
	}
}

void CortexController::setLEDState(LEDState state) {
	if (state != ledState)
		ledStatePending = true;
	ledState = state;
}

bool  CortexController::setupBot() {
	LOG(INFO) << "setup bot";
	return cmdSETUP();
}

bool CortexController::enableBot() {
	LOG(INFO) << "enable bot";
	return cmdENABLE();
}

bool CortexController::disableBot() {
	LOG(INFO) << "disable bot";
	return cmdDISABLE();
}


bool CortexController::info(bool &powered, bool& setuped, bool &enabled) {
	LOG(INFO) << "get info";
	return cmdINFO(powered,setuped, enabled);
}


bool CortexController::getAngles(ActuatorStateType actuatorState[]) {
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

bool CortexController::power(bool onOff) {
	LOG(INFO) << "power (" << onOff << ")";

	bool ok;
	if (onOff) {
		ok = cmdPOWER(true);
		ok = ok && cmdENABLE();
	} else {
		ok = cmdDISABLE();
		ok = ok && cmdPOWER(false);
	}
	return ok;
}

bool CortexController::move(JointAngles angle_rad, int duration_ms) {
	duration_ms = max(20, duration_ms); // minimum time is 20ms
	return cmdMOVETO(angle_rad, min(9999,duration_ms));
}

void CortexController::directAccess(string cmd, string& response, bool &okOrNOk) {
	okOrNOk = callMicroController(cmd, response, 5000);
}

void CortexController::loop() {
	string cmd = "";
	bool ok = true;
	if (ledStatePending) {
		ok = cmdLED(ledState);
	}
	if (!ok) {
		cerr << "sending failed (" << getLastError() << ")";
	}
}

void CortexController::sendString(string str) {
	uint8_t checksum = 0;
	computeChecksum(str, checksum);

	if (withChecksum) {
		// add checksum to string
		str +=" chk=";
		str += std::to_string(checksum); // ITOS(checksum);
	}

	serialCmd.sendString(str);
}


bool CortexController::callMicroController(string& cmd, string& response, int timeout_ms) {
	resetError();

	CommandDispatcher::getInstance().addCmdLine(cmd);
	// check command to identify timeout
	for (int i = 0;i<CommDefType::NumberOfCommands;i++) {
		string cmdStr = string(commDef[i].name);
		if (hasPrefix(cmd,cmdStr)) {
			timeout_ms = commDef[i].expectedExecutionTime_ms;
			break;
		}
	}
	sendString(cmd);
	delay(5);
	bool ok = receive(response, timeout_ms-5);
	replace (response.begin(), response.end(), '\r' , ' ');
	// replace (response.begin(), response.end(), '\n' , 'N');

	if (ok) {
		CommandDispatcher::getInstance().addCmdLine(response);
		CommandDispatcher::getInstance().updateHeartbeat();
	}
	else {
		if (getLastError() != ABSOLUTELY_NO_ERROR)
			CommandDispatcher::getInstance().addCmdLine(getLastErrorMessage());
		else
			CommandDispatcher::getInstance().addCmdLine("unknown error");
	}
	LOG(DEBUG) << "send -> \"" << cmd << " timeout=" << timeout_ms << "-> \"" << response << "\" ok=" << string(ok?"true":"false") << " (" << getLastError() << ")";
	return ok;
}


bool CortexController::receive(string& str, int timeout_ms) {

	string response="";
	string reponsePayload="";
	unsigned long startTime = millis();
	int bytesRead;
	bool replyIsOk = false; // does not necessarily mean that the answer is "ok", we only received a reply which we can read
	bool okOrNOK = false;

	// read from serial until "ok" or "nok" has been read or timeout occurs
	int retryCount= 3;// check at least three times to receive an response with a certain delay in between
	string rawResponse ="";
	bool isTimeout = false;
	do {
		bytesRead = serialCmd.receive(rawResponse);
		if (bytesRead > 0) {
			response += rawResponse;
			replyIsOk = checkReponseCode(response, reponsePayload,okOrNOK);
		} else
			delay(1); // enough to transfer 64 byte
		retryCount--;
		isTimeout = (millis() - startTime > (unsigned long)timeout_ms);
	}
	while (((retryCount > 0) || !isTimeout) && (!replyIsOk));


	if (replyIsOk) {
		communicationFailureCounter = 0; // communication was ok, reset any previous failure
		str = reponsePayload;
		if (okOrNOK) {
			LOG(DEBUG) << "response \""
				<< replaceWhiteSpace(reponsePayload)
				<< "\" & " << (okOrNOK?"OK(":"NOK(") << getLastError() <<  ")";
		} else {
			LOG(WARNING) << "response \""
				<< replaceWhiteSpace(reponsePayload)
				<< "\" & " << (okOrNOK?"OK(":"NOK(") << getLastError() <<  ")";
		}
	} else {
		if (isTimeout) {
			LOG(WARNING) << "no response";
			setError(CORTEX_NO_RESPONSE);
			okOrNOK = false;
		} else {
			str = "";
			LOG(WARNING) << "response-error \"" << replaceWhiteSpace(response) << "|" << replaceWhiteSpace(reponsePayload) << "\" not parsed";
		}
		// communication received no parsable string, reset any remains in serial buffer
		serialCmd.clear();
		delay(10);
		serialCmd.clear();
		communicationFailureCounter++;
	}

	return okOrNOK;
}
