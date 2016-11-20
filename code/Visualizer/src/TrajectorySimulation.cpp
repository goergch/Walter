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
void anglesInputCallback(const JointAngles& pAngles) {
	TrajectorySimulation::getInstance().setAngles(pAngles);
}

TrajectorySimulation::TrajectorySimulation() {
	resetTrajectory();
}

void TrajectorySimulation::setup() {
	retrieveFromRealBotFlag = false;
	TrajectoryPlayer::setup();

	// callbacks from UI: inform me when any data  has changed
	BotWindowCtrl::getInstance().setTcpInputCallback(poseInputCallback);
	BotWindowCtrl::getInstance().setAnglesCallback(anglesInputCallback);
	Pose pose;
	pose.angles = getCurrentAngles();
	Kinematics::getInstance().computeForwardKinematics( pose);

	// carry out inverse kinematics to get alternative solutions
	setPose(pose);
}

void TrajectorySimulation::loop() {
	TrajectoryPlayer::loop();

	// if the bot is moving, fetch its current position and send it to the UI via Trajectory Simulation
	if (retrieveFromRealBotFlag) {
		string nodeAsString = TrajectoryExecution::getInstance().currentTrajectoryNodeToString();
		LOG(DEBUG) << nodeAsString;

		int idx = 0;
		TrajectoryNode node;
		bool ok = node.fromString(nodeAsString, idx);

		if (!ok)
			LOG(ERROR) << "parse error node";
		// set pose of bot to current node and send to UI
		TrajectorySimulation::getInstance().setAngles(node.pose.angles);
	}

	// we need to send the simulation pose to the bot
	if (sendToRealBotFlag) {
		JointAngles currentAngles = TrajectorySimulation::getInstance().getCurrentAngles();
		string anglesAsString = currentAngles.toString();
		TrajectoryExecution::getInstance().setAnglesAsString(anglesAsString);
	}

}

void TrajectorySimulation::receiveFromRealBot(bool yesOrNo) {
	retrieveFromRealBotFlag = yesOrNo;
}

void TrajectorySimulation::sendToRealBot(bool yesOrNo) {
	sendToRealBotFlag = yesOrNo;
}

