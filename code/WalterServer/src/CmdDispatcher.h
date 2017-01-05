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
	string getAlertLineJson(int fromIdx);
	string getHeartbeatJson();
	void updateHeartbeat();

	void addCmdLine(string line);
	void addLogLine(string line);
	void addAlert(string line);

	string getIncrLogLineJSon(string line);

private:

	string cortexCmdJson;
	string cortexLogJson;
	string alertJson;

	int logLineCounter;
	int cmdLineCounter;
	int alertCounter;

	uint32_t lastHeartbeat = 0;
};


#endif /* WEBSERVERAPI_H_ */
