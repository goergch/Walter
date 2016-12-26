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

	void dispatch(string method, string query, string &response, bool &okOrNOk);
	bool setup();
	static CommandDispatcher& getInstance();
};


#endif /* WEBSERVERAPI_H_ */
