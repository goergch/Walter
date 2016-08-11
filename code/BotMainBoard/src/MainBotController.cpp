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


bool poseInputCallback(const Pose& pose) {
	KinematicsSolutionType solution;
	std::vector<KinematicsSolutionType> validSolutions;
	JointAngleType currentAngles = MainBotController::getInstance().getCurrentAngles();

	bool ok = Kinematics::getInstance().computeInverseKinematics(actuatorLimits, currentAngles, pose, solution,validSolutions);
	if (ok) {
		MainBotController::getInstance().setAngles(solution.angles);
		MainBotController::getInstance().setPose(pose);
		MainBotController::getInstance().setConfiguration(solution.config);
		MainBotController::getInstance().setPossibleSolutions(validSolutions);
	}
	return ok;
}


// called, when angles have been changed in ui and kinematics need to be recomputed
void anglesInputCallback(const JointAngleType& pAngles) {
	Pose pose;
	KinematicConfigurationType config;
	Kinematics::getInstance().computeForwardKinematics(pAngles, pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	MainBotController::getInstance().setAngles(pAngles);
	MainBotController::getInstance().setPose(pose);
	MainBotController::getInstance().setConfiguration(config);

	// compute inverse Kinematics to get alternative solutions
	poseInputCallback(MainBotController::getInstance().getCurrentPose());
}


MainBotController::MainBotController() {
}

void MainBotController::setup() {
	currJointAngles = {0,0,0,0,0,0,radians(35.0)};
	botWindowCtrl.setTcpInputCallback(poseInputCallback);
	botWindowCtrl.setAnglesCallback(anglesInputCallback);
	Kinematics::getInstance().computeForwardKinematics(currJointAngles, currPose);

	// carry out inverse kinematics to get alternative solutions
	poseInputCallback(currPose);
}

void MainBotController::loop() {
	delay(10);
}


