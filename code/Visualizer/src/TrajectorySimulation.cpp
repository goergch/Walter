/*
 * BotController.cpp
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include "TrajectorySimulation.h"
#include "Util.h"
#include "BotWindowCtrl.h"
#include "Kinematics.h"
#include "TrajectoryExecution.h"

void TrajectorySimulation::notifyNewPose(const Pose& pPose) {
	if (BotWindowCtrl::getInstance().isReady()) {
		BotWindowCtrl::getInstance().notifyNewBotData();
	}
}

bool poseInputCallback(const Pose& pose) {
	return TrajectorySimulation::getInstance().setPose(pose);
}

// called when angles have been changed in ui and kinematics need to be recomputed
void anglesInputCallback(const JointAngleType& pAngles) {
	TrajectorySimulation::getInstance().setAngles(pAngles);
}

TrajectorySimulation::TrajectorySimulation() {
	resetTrajectory();
}

void TrajectorySimulation::setup() {
	connectToTrajExecution = false;
	TrajectoryPlayer::setup();

	// callbacks from UI: inform me when any data  has changed
	BotWindowCtrl::getInstance().setTcpInputCallback(poseInputCallback);
	BotWindowCtrl::getInstance().setAnglesCallback(anglesInputCallback);
	Pose pose;
	Kinematics::getInstance().computeForwardKinematics(getCurrentAngles(), pose);

	// carry out inverse kinematics to get alternative solutions
	setPose(pose);
}

void TrajectorySimulation::loop() {
	TrajectoryPlayer::loop();
	if (connectToTrajExecution) {
		string nodeAsString = TrajectoryExecution::getInstance().currentTrajectoryNodeToString();
		TrajectoryNode node;
		int idx;
		bool ok = node.fromString(nodeAsString, idx);
		if (!ok)
			LOG(ERROR) << "parse error node";

		// set pose of bot to current node
		TrajectorySimulation::getInstance().setAngles(node.angles);
		TrajectorySimulation::getInstance().setPose(node.pose);
	}
}

void TrajectorySimulation::connectToExecution(bool yesOrNo) {
	connectToTrajExecution = yesOrNo;
}


