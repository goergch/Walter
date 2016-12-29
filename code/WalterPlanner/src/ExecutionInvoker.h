/*
 * ExecutionInvoker.h
 *
 *  Created on: 27.12.2016
 *      Author: JochenAlt
 */

#ifndef EXECUTIONINVOKER_H_
#define EXECUTIONINVOKER_H_

#include <string.h>
#include "spatial.h"
#include "Trajectory.h"

using namespace std;

class ExecutionInvoker {
public:
	ExecutionInvoker();
	static ExecutionInvoker& getInstance();

	bool startupBot();
	bool teardownBot();
	bool isBotUpAndRunning();
	bool setAngles(JointAngles angles);
	TrajectoryNode getAngles();
	bool runTrajectory(Trajectory traj);
	bool stopTrajectory();
	string directAccess(string directCommand,string &reponse);

	void setHost(string host, int port);

private:
	bool httpGET(string path, string &responsestr, int timeout_ms);
	bool httpPOST(string path, string body, string &responsestr, int timeout_ms);

	std::string host;
	int port;
};

#endif /* EXECUTIONINVOKER_H_ */
