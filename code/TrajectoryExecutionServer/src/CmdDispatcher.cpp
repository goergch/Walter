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


void CommandDispatcher::dispatch(string method, string query, string &response, bool &okOrNOk) {
	LOG(DEBUG) << " method=" << method << " query=" << query;

	// check if cortex command
	for (int i = 0;i<CommDefType::NumberOfCommands;i++) {
		string cortexCmdStr = upcase(string(commDef[i].name));
		if (upcase(method).compare(0,cortexCmdStr.length(), cortexCmdStr) == 0) {
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
			    	}
			    }
			};
			LOG(DEBUG) << "calling direct access with " << command;
			TrajectoryExecution::getInstance().directAccess(command, response, okOrNOk);
			return;
		}
	}
	response = "antwort";
	okOrNOk = true;
}
