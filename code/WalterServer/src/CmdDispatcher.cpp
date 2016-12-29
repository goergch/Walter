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
}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}



// returns true, if request has been dispatched
bool  CommandDispatcher::dispatch(string uri, string query, string body, string &response, bool &okOrNOk) {
	LOG(DEBUG) << " uri=" << uri << " query=" << query;

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

	// check for input fields or variables
	if (hasPrefix(uri, "/")) {
		// variable used in html text?
		if (hasPrefix(query, "var")) {
			string name;
			if (query.length() > string("key=").length())
				name = query.substr(string("key=").length());
			response = getVariable(name, okOrNOk);
			return true;
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


void CommandDispatcher::addCortexLogLine(string logline) {
	cortexlog += logline + "\r\n";
	while (cortexlog.length() > LOGVIEW_MAXSIZE) {
		int idx = cortexlog.find("\r\n");
		if (idx > 0)
			cortexlog = cortexlog.substr(idx+2);
		else
			cortexlog = cortexlog.substr(LOGVIEW_MAXSIZE - cortexlog.length());
	}
}
