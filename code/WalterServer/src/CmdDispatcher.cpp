/*
 * WebserverAPI.cpp
 *
 *  Created on: 26.12.2016
 *      Author: JochenAlt
 */

#include "CommDef.h"
#include "core.h"

#include "TrajectoryExecution.h"
#include "CmdDispatcher.h"
#include "logger.h"
#include "util.h"

#include "setup.h"
#include <vector>

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
	addLogLine("dummy zeile");
	addAlert("1. Alert");
	addAlert("2. Alert");

}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}

// returns true, if request has been dispatched
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {
	response = "";
	string urlPath = getPath(uri);

	vector<string> urlParamName;
	vector<string> urlParamValue;

	compileURLParameter(query,urlParamName,urlParamValue);

	// check if cortex command
	if (hasPrefix(uri, "/cortex")) {
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
					response += "ok.\r\n";
				else
					response += "failed.\r\n";
				return true;
			}
		}
	}

	if (hasPrefix(uri, "/direct")) {

		if (hasPrefix(query, "param=")) {
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

	if (hasPrefix(uri, "/executor/")) {
		string executorPath = uri.substr(string("/executor/").length());
		LOG(DEBUG) << uri;

		if (hasPrefix(executorPath, "startupbot")) {
			okOrNOk =  TrajectoryExecution::getInstance().startupBot();
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response = s.str();
			return true;
		}
		else if (hasPrefix(executorPath, "teardownbot")) {
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
		else if (hasPrefix(executorPath, "isupandrunning")) {
			response = "";
			bool result = TrajectoryExecution::getInstance().isBotUpAndReady();
			okOrNOk = true;
			response = result?"true":"false";
			return true;
		}
		else if (hasPrefix(executorPath, "setangles")) {
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
			response  = TrajectoryExecution::getInstance().currentTrajectoryNodeToString();
			okOrNOk = !isError();
			return true;
		}
		else if (hasPrefix(executorPath, "settrajectory")) {
			string param = urlDecode(body);
			TrajectoryExecution::getInstance().runTrajectory(param);
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
		else if (hasPrefix(executorPath, "stoptrajectory")) {
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
					okOrNOk = true;
					return true;
				}
			} else {
				if (keyValue.compare(string("cortexlog")) == 0) {
					if (getURLParameter(urlParamName, urlParamValue, "from", keyValue)) {
						int from = string_to_int(keyValue);
						if (from >=0)
							response = getLogLineJson(from+1);
						else
							response = getLogLineJson(0);
						okOrNOk = true;
						return true;
					}
				} else
				{
					if (keyValue.compare(string("alert")) == 0) {
						if (getURLParameter(urlParamName, urlParamValue, "from", keyValue)) {
							int from = string_to_int(keyValue);
							if (from >=0) {
								response = getAlertLineJson(from);
								okOrNOk = true;
								return true;
							}
						} else
						{
							response = int_to_string(alertCounter);
							okOrNOk = true;
							return true;
						}
					}
				}
			}
		} else {
			if (getURLParameter(urlParamName, urlParamValue, "action", keyValue)) {
				if (keyValue.compare("savecmd") == 0) {
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

string  CommandDispatcher::getCmdLineJson(int fromId) {
	string fromIdStr = string("{\"id\":") + int_to_string(fromId);
	string cmdJson;
	int idx = cortexCmdJson.find(fromIdStr);
	if (idx>=0){
		cmdJson = cortexCmdJson.substr(idx);
	} else
		cmdJson = "";


	string result = "[";
	result += cmdJson + string(", ") + "{\"id\":" + int_to_string(cmdLineCounter) + ", \"line\":\"&gt\"}" + "]";
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

void CommandDispatcher::addCmdLine(string line) {
	int idx = cortexCmdJson.find("\"line\"");
	if (idx >= 0)
		cortexCmdJson += ", ";

	cortexCmdJson += "{\"id\":" + int_to_string(cmdLineCounter++) + ", \"line\":\"" + htmlEncode(line) + "\"}";
}

void CommandDispatcher::addAlert(string line) {
	int idx = alertJson.find("\"line\"");
	if (idx >= 0)
		alertJson += ", ";

	alertJson += "{\"id\":" + int_to_string(alertCounter++) + ", \"line\":\"" + htmlEncode(line) + "\"}";
}

void CommandDispatcher::addLogLine(string line) {
	int idx = cortexLogJson.find("\"line\"");

	if (idx >= 0)
		cortexLogJson += ", ";
	cortexLogJson += "{\"id\":" + int_to_string(logLineCounter++) + ", \"line\":\"" + htmlEncode(line) + "\"}";

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

