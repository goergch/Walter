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
	traj.compile();

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


void TrajectoryExecution::setAnglesAsString(string anglesAsString) {
	JointAngles angles;
	int idx = 0;
	bool ok = angles.fromString(anglesAsString, idx);
	if (!ok)
		LOG(ERROR) << "parse error angles";
	TrajectoryPlayer::setAngles(angles);
}

void TrajectoryExecution::loop() {
	// take current time, compute IK and store pose and angles every TrajectorySampleRate. When a new pose is computed, notifyNewPose is called
	TrajectoryPlayer::loop();
}


void TrajectoryExecution::notifyNewPose(const Pose& pPose) {
	rational angles[NumberOfActuators] = {pPose.angles[0], pPose.angles[1], pPose.angles[2], pPose.angles[3], pPose.angles[4], pPose.angles[5], pPose.angles[6] };

	// move the bot to the passed position within the next TrajectorySampleRate ms.
	ActuatorCtrlInterface::getInstance().move(angles, TrajectorySampleRate);
}

