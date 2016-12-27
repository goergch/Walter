/*
 * WebserverAPI.h
 *
 *  Created on: 26.12.2016
 *      Author: JochenAlt
 */

#ifndef WEBSERVERAPI_H_
#define WEBSERVERAPI_H_

#include "TrajectoryExecution.h"

class CommandDispatcher {
public:
	CommandDispatcher();

	bool dispatch(string uri, string query, string &response, bool &okOrNOk);
	bool setup();
	static CommandDispatcher& getInstance();
	string getVariable(string name, bool &ok);
private:
	string cortexreply;
};


#endif /* WEBSERVERAPI_H_ */
