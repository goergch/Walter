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

#include "setup.h"

CommandDispatcher commandDispatcher;

CommandDispatcher::CommandDispatcher() {
	addCmdLine("&gt;");
	addLogLine("start logging");
}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}

// returns true, if request has been dispatched
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {
	response = "";
	string urlPath = getPath(uri);

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

			cortexreply += ">" + cmd + "\r\n";
			TrajectoryExecution::getInstance().directAccess(cmd, cmdReply, okOrNOk);

			if (cmdReply.length() > 0) {
				response = cmdReply;
				cortexreply += cmdReply + " ";
			} else {
				response = "";
			}
			std::ostringstream s;
			if (okOrNOk) {
				s << "OK";
			} else {
				s << "NOK(" << getLastError() << ") " << getErrorMessage(getLastError());
			}
			response += s.str();
			cortexreply += s.str();
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
		if (hasPrefix(query, "key=")) {
			string name = query.substr(string("key=").length());
			response = getVariableJson(name, okOrNOk);
			return okOrNOk;
		} else
		if (hasPrefix(query, "action=")) {
			string action = query.substr(string("action=").length());
			action = action.substr(0,action.find("&"));
			string value = query.substr(string("action=").length() + action.length()+1 + string("value=").length());
			if (action.compare("savecmd") == 0) {
				string cmdReply;
				TrajectoryExecution::getInstance().directAccess(value, cmdReply, okOrNOk);
			}
			return okOrNOk;
		}
	}
	// check for input fields or variables
	if (hasPrefix(uri, "/")) {
		// variable used in html text?
		if (hasPrefix(query, "var")) {
			string name;
			if (query.length() > string("key=").length())
				name = query.substr(string("key=").length());
			response = getVariable(name, okOrNOk);
			return okOrNOk;
		}
		// input in the cortext input field
		if (hasPrefix(query, "cortexinputfield")) {
			string cmd = urlDecode(query.substr(string("cortexinputfield=").length()));
			LOG(DEBUG) << "calling cortex with \"" << cmd << "\"";
			string cmdReply;

			cortexreply += ">" + cmd + "\r\n";
			TrajectoryExecution::getInstance().directAccess(cmd, cmdReply, okOrNOk);

			if (cmdReply.length() > 0)
				cortexreply += cmdReply + " ";
			if (okOrNOk)
				cortexreply += "ok.\r\n";
			else
				cortexreply += "(communication failure)\r\n";
			return false; // serve with static content
		}
	}

	okOrNOk = false;
	return false;
}

string CommandDispatcher::getVariable(string name, bool &ok) {
	ok = true;
	if (name.compare(string("cortexreply")) == 0)
		return cortexreply;

	if (name.compare(string("cortexlog")) == 0)
		return cortexlog;

	if (name.compare(string("port")) == 0)
		return int_to_string(SERVER_PORT);
	ok = false;
	return string_format("variable named %s not found", name.c_str());
}

string CommandDispatcher::getVariableJson(string name, bool &ok) {
	ok = true;
	if (name.compare(string("cortexcmd")) == 0)
		return getCmdLineJson();;

	if (name.compare(string("cortexlog")) == 0)
		return getLogLineJson();

	if (name.compare(string("port")) == 0)
		return int_to_string(SERVER_PORT);
	ok = false;
	return string_format("variable named %s not found", name.c_str());
}


void CommandDispatcher::addCortexLogLine(string logline) {
	cortexlog += logline + "\r\n";
	while (cortexlog.length() > LOGVIEW_MAXSIZE) {
		int idx = cortexlog.find("\r\n");
		if (idx >= 0)
			cortexlog.erase(0,idx+2);
		else {
			cortexlog.erase(0,cortexlog.length() - LOGVIEW_MAXSIZE);
		}
	}
}

string  CommandDispatcher::getCmdLineJson() {
	string result = "[";
	result += cortexCmdJson + string(", ") + "{ \"line\":\"&gt\"}" + "]";
	return result;
}

string CommandDispatcher::getLogLineJson() {
	string result = "[ ";
	result += cortexLogJson + " ]";
	return result;
}

void CommandDispatcher::addCmdLine(string line) {
	int idx = cortexCmdJson.find("\"line\"");
	if (idx >= 0)
		cortexCmdJson += ", ";

	cortexCmdJson += "{ \"line\":\"" + line + "\"}";
}

void CommandDispatcher::addLogLine(string line) {
	int idx = cortexLogJson.find("\"line\"");

	if (idx >= 0)
		cortexLogJson += ", ";
	cortexLogJson += "{\"line\":\"" + line + "\"}";

	// remove staff from beginning if log gets loo long to be displayed
	while (cortexLogJson.length() > LOGVIEW_MAXSIZE) {
		int idx = cortexLogJson.find("\"line\"");
		if (idx >= 0) {
			idx = cortexLogJson.find("\"line\"", idx+1);
			if (idx>= 0) {
				cortexLogJson = cortexLogJson.substr(idx);
			}
		}
	}
}

