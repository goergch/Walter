/*
 * WebserverAPI.cpp
 *
 *  Created on: 26.12.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecution.h"
#include "CmdDispatcher.h"
#include "CommDef.h"

CommandDispatcher commandDispatcher;



CommandDispatcher::CommandDispatcher() {
}

bool CommandDispatcher::setup() {
	return TrajectoryExecution::getInstance().setup();
}

CommandDispatcher& CommandDispatcher::getInstance() {
	return commandDispatcher;
}



// returns true, if request has been dispatched
bool  CommandDispatcher::dispatch(string uri, string query, string &response, bool &okOrNOk) {
	LOG(DEBUG) << " uri=" << uri << " query=" << query;

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

				response = "";
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

	// check if url command
	if (hasPrefix(uri, "/direct")) {
		if (hasPrefix(query, "/cmd")) {
			string cmd = urlDecode(query.substr(string("cmd=").length()));
			LOG(DEBUG) << "calling cortex with \"" << cmd << "\"";
			string cmdReply;

			TrajectoryExecution::getInstance().directAccess(cmd, cmdReply, okOrNOk);
			cortexreply += ">" + cmd + " -> ";

			if (cmdReply.length() > 0)
				cortexreply += cmdReply + " ";
			if (okOrNOk)
				cortexreply += "ok<br>";
			else
				cortexreply += "failed.<br>";
			return true;
		}
	}

	// check for input fields or variables
	if (hasPrefix(uri, "/")) {
		if (hasPrefix(query, "var")) {
			string name;
			if (query.length() > string("key=").length())
				name = query.substr(string("key=").length());
			response = getVariable(name, okOrNOk);
			return true;
		}
		if (hasPrefix(query, "cortexinputfield")) {
			string cmd = urlDecode(query.substr(string("cortexinputfield=").length()));
			LOG(DEBUG) << "calling cortex with \"" << cmd << "\"";
			string cmdReply;

			TrajectoryExecution::getInstance().directAccess(cmd, cmdReply, okOrNOk);
			cortexreply += ">" + cmd + " -> ";

			if (cmdReply.length() > 0)
				cortexreply += cmdReply + " ";
			if (okOrNOk)
				cortexreply += "ok.\r\n";
			else
				cortexreply += "failed.\r\n";
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

	if (name.compare(string("port")) == 0)
		return "8000";
	ok = false;
	return string_format("variable named %s not found", name.c_str());
}
