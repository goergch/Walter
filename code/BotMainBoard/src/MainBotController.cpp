/*
 * BotController.cpp
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include <MainBotController.h>
#include "Util.h"
#include "ui/BotWindowCtrl.h"
#include "Kinematics.h"



bool MainBotController::setPose(const Pose& pPose) {
	KinematicsSolutionType solution;
	std::vector<KinematicsSolutionType> validSolutions;
	JointAngleType currentAngles = MainBotController::getInstance().getCurrentAngles();

	bool ok = Kinematics::getInstance().computeInverseKinematics(actuatorLimits, currentAngles, pPose, solution,validSolutions);
	if (ok) {
		MainBotController::getInstance().setAnglesImpl(solution.angles);
		MainBotController::getInstance().setPoseImpl(pPose);
		MainBotController::getInstance().setConfigurationImpl(solution.config);
		MainBotController::getInstance().setPossibleSolutionsImpl(validSolutions);

		if (BotWindowCtrl::getInstance().isReady())
			BotWindowCtrl::getInstance().notifyNewBotData();
	}
	return ok;
}

bool poseInputCallback(const Pose& pose) {
	return MainBotController::getInstance().setPose(pose);
}


// called, when angles have been changed in ui and kinematics need to be recomputed
void anglesInputCallback(const JointAngleType& pAngles) {
	Pose pose;
	KinematicConfigurationType config;
	Kinematics::getInstance().computeForwardKinematics(pAngles, pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	MainBotController::getInstance().setAnglesImpl(pAngles);
	MainBotController::getInstance().setPoseImpl(pose);
	MainBotController::getInstance().setConfigurationImpl(config);

	// compute inverse Kinematics to get alternative solutions
	poseInputCallback(MainBotController::getInstance().getCurrentPose());
}



MainBotController::MainBotController() {
}

void MainBotController::setup() {
	currJointAngles = {0,0,0,0,0,0,radians(35.0)};
	BotWindowCtrl::getInstance().setTcpInputCallback(poseInputCallback);
	BotWindowCtrl::getInstance().setAnglesCallback(anglesInputCallback);
	Kinematics::getInstance().computeForwardKinematics(currJointAngles, currPose);

	// carry out inverse kinematics to get alternative solutions
	poseInputCallback(currPose);
}

void MainBotController::loop() {
	delay(10);
}


