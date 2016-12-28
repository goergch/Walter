/*
 * ExecutionInvoker.h
 *
 *  Created on: 27.12.2016
 *      Author: JochenAlt
 */

#ifndef EXECUTIONINVOKER_H_
#define EXECUTIONINVOKER_H_

#include <string>

class ExecutionInvoker {
public:
	ExecutionInvoker();
	static ExecutionInvoker& getInstance();

	bool startupBot();
	bool teardownBot();

	void setHost(std::string host, int port);

private:
	std::string host;
	int port;
};

#endif /* EXECUTIONINVOKER_H_ */
