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

	string getCmdLineJson(int fromIdx);
	string getLogLineJson(int fromIdx);
	void addCmdLine(string line);
	void addLogLine(string line);
	string getIncrLogLineJSon(string line);

private:

	string cortexCmdJson;
	string cortexLogJson;
	int logLineCounter;
	int cmdLineCounter;
};


#endif /* WEBSERVERAPI_H_ */
