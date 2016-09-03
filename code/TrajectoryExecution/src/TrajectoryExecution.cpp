/*
 * TrajectoryExecController.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecution.h"
#include "ActuatorCtrlInterface.h"

TrajectoryExecution::TrajectoryExecution() {

}

TrajectoryExecution& TrajectoryExecution::getInstance() {
	static TrajectoryExecution instance;
	return instance;
}


bool TrajectoryExecution::setup() {
	bool ok = ActuatorCtrlInterface::getInstance().setupCommunication();
	if (!ok)
    	LOG(ERROR) << "uC not present";

	TrajectoryPlayer::setup();

	return ok;
}

// send a direct command to uC
void TrajectoryExecution::directAccess(string cmd, string& response, bool &okOrNOk) {
	ActuatorCtrlInterface::getInstance().directAccess(cmd, response, okOrNOk);
}

void TrajectoryExecution::loguCToConsole() {
	ActuatorCtrlInterface::getInstance().loguCToConsole();
}


string TrajectoryExecution::currentPoseToString() {
	Pose pose = getCurrentPose();
	return pose.toString();
}

string TrajectoryExecution::currentTrajectoryNodeToString() {
	TrajectoryNode node = getCurrentTrajectoryNode();
	return node.toString();
}

void TrajectoryExecution::runTrajectory(const string& trajectoryStr) {
	Trajectory& traj = getTrajectory();
	int idx = 0;
	bool ok = traj.fromString(trajectoryStr, idx);
	if (!ok)
		LOG(ERROR) << "parse error trajectory";

	playTrajectory();
}

void TrajectoryExecution::setPose(const string& poseStr) {
	Pose pose;
	int idx = 0;
	bool ok = pose.fromString(poseStr, idx);
	if (!ok)
		LOG(ERROR) << "parse error trajectory";
	TrajectoryPlayer::setPose(pose);
}




