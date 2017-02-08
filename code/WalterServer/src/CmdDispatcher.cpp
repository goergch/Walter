
#include "CommDef.h"
#include "core.h"

#include "TrajectoryExecution.h"
#include "CmdDispatcher.h"
#include "Util.h"

#include "setup.h"
#include <vector>
#include "logger.h"


CommandDispatcher commandDispatcher;


bool getURLParameter(vector<string> names, vector<string> values, string key, string &value) {
	for (int i = 0;i<(int)names.size();i++) {
		if (names[i].compare(key) == 0) {
			value = values[i];
			return true;
		}
	}
	return false;
}


void compileURLParameter(string uri, vector<string> &names, vector<string> &values) {
	names.clear();
	values.clear();

	std::istringstream iss(uri);
	std::string token;
	while (std::getline(iss, token, '&'))
	{
		// extract name and value of parameter
		int equalsIdx = token.find("=");
		if (equalsIdx > 0) {
			string name = token.substr(0,equalsIdx);
			string value = token.substr(equalsIdx+1);
			names.insert(names.end(), name);
			values.insert(values.end(), urlDecode(value));
		};
	}
}

CommandDispatcher::CommandDispatcher() {
	logLineCounter = 0;
	cmdLineCounter = 0;
	addCmdLine("<no command>");
	addLogLine("start logging");
}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}

// central dispatcher of all url requests arriving at the webserver
// returns true, if request has been dispatched within dispatch. Otherwise the caller
// should assume that static content is to be displayed.
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {

	response = "";
	string urlPath = getPath(uri);

	vector<string> urlParamName;
	vector<string> urlParamValue;

	compileURLParameter(query,urlParamName,urlParamValue);

	// check if direct cortex command defined via URL parameter
	// example: /cortex/LED?blink
	if (hasPrefix(uri, "/cortex")) {
		LOG(DEBUG) << uri << " " << query;

		string cmd = uri.substr(string("/cortex/").length());
		for (int i = 0;i<CommDefType::NumberOfCommands;i++) {
			string cortexCmdStr = string(commDef[i].name);
			if (hasPrefix(cmd, cortexCmdStr)) {
				string command = string(commDef[i].name);
				// are there any parameters ?
				if (query.length() > 0) {
					std::istringstream iss(query);
					std::string token;
					while (std::getline(iss, token, '&'))
					{
						// extract name and value of parameter
						int equalsIdx = token.find("=");
						if (equalsIdx > 0) {
							string name = token.substr(0,equalsIdx);
							string value = token.substr(equalsIdx+1);
							command += " " + name + "=" + value;
						} else {
							command += " " + token;
						}
					}
				};
				LOG(DEBUG) << "calling cortex with \"" << command << "\"";
				string cmdReply;
				TrajectoryExecution::getInstance().directAccess(command, cmdReply, okOrNOk);

				if (cmdReply.length() > 0)
					response += cmdReply + "";
				if (okOrNOk)
					response += "ok";
				else
					response += "failed";
				return true;
			}
		}
	}

	// check, if cortex is called via one command string
	// example /direct?param=LED+blink
	if (hasPrefix(uri, "/direct")) {
		if (hasPrefix(query, "param=")) {
			LOG(DEBUG) << uri << " " << query;

			string cmd = urlDecode(query.substr(string("param=").length()));
			LOG(DEBUG) << "calling cortex with \"" << cmd << "\"";
			string cmdReply;

			TrajectoryExecution::getInstance().directAccess(cmd, cmdReply, okOrNOk);
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response += s.str();
			return true;
		}
	}

	// check, if TransactionExecutor is called with orchestrated calls
	if (hasPrefix(uri, "/executor/")) {
		string executorPath = uri.substr(string("/executor/").length());

		if (hasPrefix(executorPath, "startupbot")) {
			LOG(DEBUG) << uri << " " << query;


			okOrNOk =  TrajectoryExecution::getInstance().startupBot();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
		}
		else if (hasPrefix(executorPath, "teardownbot")) {
			LOG(DEBUG) << uri << " " << query;


			okOrNOk = TrajectoryExecution::getInstance().teardownBot();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
			return true;
		}
		else if (hasPrefix(executorPath, "nullpositionbot")) {
			LOG(DEBUG) << uri << " " << query;


			okOrNOk = TrajectoryExecution::getInstance().moveToNullPosition();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
			return true;
		}
		else if (hasPrefix(executorPath, "isupandrunning")) {
			response = "";
			bool result = TrajectoryExecution::getInstance().isBotUpAndReady();
			okOrNOk = true;
			response = result?"true":"false";
			return true;
		}
		else if (hasPrefix(executorPath, "emergencystop")) {
			LOG(DEBUG) << uri << " " << query;


			response = "";
			bool result = TrajectoryExecution::getInstance().emergencyStopBot();
			okOrNOk = true;
			response = result?"true":"false";
			return true;
		}
		else if (hasPrefix(executorPath, "setangles")) {
			LOG(DEBUG) << uri << " " << query;


			string param = urlDecode(query.substr(string("param=").length()));
			okOrNOk = TrajectoryExecution::getInstance().setAnglesAsString(param);
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
			return true;
		}
		else if (hasPrefix(executorPath, "getangles")) {
			LOG(DEBUG) << uri << " " << query;

			int indent = 0;
			response  = TrajectoryExecution::getInstance().currentTrajectoryNodeToString(indent);
			okOrNOk = !isError();
			return true;
		}
		else if (hasPrefix(executorPath, "settrajectory")) {
			LOG(DEBUG) << uri << " " << query;

			string param = urlDecode(body);
			LOG(DEBUG) << "body:" << param;

			TrajectoryExecution::getInstance().runTrajectory(param);
			okOrNOk = !isError();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response += s.str();
			return true;
		}
		else if (hasPrefix(executorPath, "stoptrajectory")) {
			LOG(DEBUG) << uri << " " << query;


			TrajectoryExecution::getInstance().stopTrajectory();
			okOrNOk = !isError();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
			return true;
		}

	}

	if (hasPrefix(uri, "/web")) {
		string keyValue;
		if (getURLParameter(urlParamName, urlParamValue, "key", keyValue)) {
			if (keyValue.compare(string("cortexcmd")) == 0) {
				if (getURLParameter(urlParamName, urlParamValue, "from", keyValue)) {
					int from = string_to_int(keyValue);
					if (from >=0 )
						response = getCmdLineJson(from+1);
					else
						response = getCmdLineJson(0);
				} else
					response = getCmdLineJson(0);
				okOrNOk = true;
				return true;
			} else {
				if (keyValue.compare(string("cortexlog")) == 0) {
					if (getURLParameter(urlParamName, urlParamValue, "from", keyValue)) {
						int from = string_to_int(keyValue);
						if (from >=0)
							response = getLogLineJson(from+1);
						else
							response = getLogLineJson(0);
					} else
						response = getLogLineJson(0);
					okOrNOk = true;
					return true;
				} else {
					if (keyValue.compare(string("alert")) == 0) {
						if (getURLParameter(urlParamName, urlParamValue, "from", keyValue)) {
							int from = string_to_int(keyValue);
							if (from >=0) {
								response = getAlertLineJson(from);
								okOrNOk = true;
								return true;
							}
						} else {
							response = int_to_string(alertCounter);
							okOrNOk = true;
							return true;
						}
					}
					else {
						if (keyValue.compare(string("heartbeat")) == 0) {
							response = getHeartbeatJson();
							okOrNOk = true;
							return true;
						}
					}
				}
			}
		} else {
			if (getURLParameter(urlParamName, urlParamValue, "action", keyValue)) {
				if (keyValue.compare("savecmd") == 0) {
					LOG(DEBUG) << uri << " " << query;


					string value;
					if (getURLParameter(urlParamName, urlParamValue, "value", value)) {
						string cmdReply;
						TrajectoryExecution::getInstance().directAccess(value, cmdReply, okOrNOk);
						response = cmdReply;
						return true;
					}
				}
			}
		}
	}

	okOrNOk = false;
	return false;
}

string CommandDispatcher::getHeartbeatJson() {
	if (millis() - lastHeartbeat < 1000)
		return "true";
	return "";
}

void CommandDispatcher::updateHeartbeat() {
	lastHeartbeat = millis();
}

string  CommandDispatcher::getCmdLineJson(int fromId) {
	string fromIdStr = string("{\"id\":") + int_to_string(fromId);
	string cmdJson;
	int idx = cortexCmdJson.find(fromIdStr);
	if (idx>=0){
		cmdJson = cortexCmdJson.substr(idx);
	} else
		cmdJson = "";

	string result = "[";
	result += cmdJson + string("]");
	return result;
}

string CommandDispatcher::getAlertLineJson(int fromId) {
	string fromIdStr = string("{\"id\":") + int_to_string(fromId);
	string shortAlertJson;
	int idx = alertJson.find(fromIdStr);
	if (idx>=0){
		int start = alertJson.find("\"line\":", idx);
		int end = alertJson.find("}", start);

		shortAlertJson = alertJson.substr(start+string("\"line\":\"").length(),end-start-string("\"line\":\"").length()-1);
	} else
		shortAlertJson = "";

	return shortAlertJson;
}

string CommandDispatcher::getLogLineJson(int fromId) {
	string fromIdStr = string("{\"id\":") + int_to_string(fromId);
	string logJson;
	int idx = cortexLogJson.find(fromIdStr);
	if (idx>=0){
		logJson = cortexLogJson.substr(idx);
	} else
		logJson = "";

	string result = "[ ";
	result += logJson + " ]";
	return result;
}

void CommandDispatcher::setOneTimeTrajectoryNodeName(string name) {
	oneTimeTrajectoryName = name;
}


void CommandDispatcher::addCmdLine(string line) {
	int CRLNRidx = 0;
	do {
		CRLNRidx = line.find("\n");
		string s;
		if (CRLNRidx >=0) {
			s = line.substr(0,CRLNRidx);
			line = line.substr(CRLNRidx+1);
		} else {
			s = line;
		}

		if (line.compare("") != 0) {
			int idx = cortexCmdJson.find("\"line\"");
			if (idx >= 0)
				cortexCmdJson += ", ";

			string time = currentTimeToString();

			cortexCmdJson += "{\"id\":" + int_to_string(cmdLineCounter++) +
					", \"time\":\"" + htmlEncode(time) + "\"" +
					", \"traj\":\"" + htmlEncode(oneTimeTrajectoryName) + "\"" +
					", \"line\":\"" + htmlEncode(s) + "\"" +
					"}";
			oneTimeTrajectoryName = "";
		}
	}
	while ((CRLNRidx >= 0));

}

void CommandDispatcher::addAlert(string line) {
	int idx = alertJson.find("\"line\"");
	if (idx >= 0)
		alertJson += ", ";

	alertJson += "{\"id\":" + int_to_string(alertCounter++) + ", \"line\":\"" + htmlEncode(line) + "\"}";
}



void CommandDispatcher::addLogLine(string line) {
	string time;

	if ((line.length() > 24) && (line[13] == ':') && (line[16] == ':')) {
		time = line.substr(11,12);
		line = line.substr(24);
	} else
		time = currentTimeToString();


	int idx = cortexLogJson.find("\"line\"");

	if (idx >= 0)
		cortexLogJson += ", ";
	cortexLogJson += "{\"id\":" + int_to_string(logLineCounter++) +
			", \"time\":\"" + htmlEncode(time) + "\"" +
			", \"line\":\"" + htmlEncode(line) + "\"}";


	// remove staff from beginning if log gets loo long to be displayed
	while (cortexLogJson.length() > LOGVIEW_MAXSIZE) {
		int idx = cortexLogJson.find("{\"id\"");
		if (idx >= 0) {
			idx = cortexLogJson.find("{\"id\"", idx+1);
			if (idx>= 0) {
				cortexLogJson = cortexLogJson.substr(idx);
			}
		}
	}
}

