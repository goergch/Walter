/*
 * ExecutionInvoker.h
 *
 * Class to call the webserver to transfer trajectoryies.
 *
 * Author: JochenAlt
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

	// call webserver to startup the bot
	bool startupBot();
	// call webserver to teardown the bot
	bool teardownBot();
	// true if bot is up and running
	bool isBotUpAndRunning();
	// send passed angles to webserver
	bool setAngles(JointAngles angles);
	// fetch current angles of bot
	TrajectoryNode getAngles();
	// pass a full trajectory to webserver. Start it immediately.
	bool runTrajectory(Trajectory traj);
	// stop currently running trajectory
	bool stopTrajectory();
	// pass a direct command to webserver
	string directAccess(string directCommand,string &reponse);
	// define url of webserver
	void setHost(string host, int port);
private:
	bool httpGET(string path, string &responsestr, int timeout_ms);
	bool httpPOST(string path, string body, string &responsestr, int timeout_ms);

	std::string host;
	int port;
};

#endif /* EXECUTIONINVOKER_H_ */
