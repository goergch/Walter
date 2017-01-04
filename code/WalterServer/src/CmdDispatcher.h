/*
 * WebserverAPI.h
 *
 *  Created on: 26.12.2016
 *      Author: JochenAlt
 */

#ifndef WEBSERVERAPI_H_
#define WEBSERVERAPI_H_

#include "TrajectoryExecution.h"
#include <vector>

class CommandDispatcher {
public:
	CommandDispatcher();

	bool dispatch(string uri, string query, string body, string &response, bool &okOrNOk);
	static CommandDispatcher& getInstance();
	string getVariableJson(string name, bool &ok);

	string getCmdLineJson();
	string getLogLineJson();
	void addCmdLine(string line);
	void addLogLine(string line);

private:

	string cortexCmdJson;
	string cortexLogJson;

};


#endif /* WEBSERVERAPI_H_ */
