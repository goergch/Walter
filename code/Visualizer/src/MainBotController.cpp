/*
 * BotController.cpp
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include "MainBotController.h"
#include "Util.h"
#include "BotWindowCtrl.h"
#include "Kinematics.h"


void MainBotController::notifyNewPose(const Pose& pPose) {
	if (BotWindowCtrl::getInstance().isReady()) {
		BotWindowCtrl::getInstance().notifyNewBotData();
	}
}

bool poseInputCallback(const Pose& pose) {
	return MainBotController::getInstance().setPose(pose);
}

// called when angles have been changed in ui and kinematics need to be recomputed
void anglesInputCallback(const JointAngleType& pAngles) {
	MainBotController::getInstance().setAngles(pAngles);
}

MainBotController::MainBotController() {
	resetTrajectory();
}

void MainBotController::setup() {
	TrajectoryPlayer::setup();

	// callbacks from UI: inform me when any data  has changed
	BotWindowCtrl::getInstance().setTcpInputCallback(poseInputCallback);
	BotWindowCtrl::getInstance().setAnglesCallback(anglesInputCallback);
	Pose pose;
	Kinematics::getInstance().computeForwardKinematics(getCurrentAngles(), pose);

	// carry out inverse kinematics to get alternative solutions
	setPose(pose);
}

