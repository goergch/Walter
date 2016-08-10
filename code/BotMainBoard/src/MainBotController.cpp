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


// called, when angles have been changed in ui and kinematics need to be recomputed
void anglesCallback(JointAngleType pAngles) {
	Pose pose;
	KinematicConfigurationType config;
	Kinematics::getInstance().computeForwardKinematics(pAngles, pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	MainBotController::getInstance().setAngles(pAngles);
	MainBotController::getInstance().setPose(pose);
	MainBotController::getInstance().setConfiguration(config);

}

bool TCPInputCallback(const Pose& pose) {
	KinematicsSolutionType solution;
	std::vector<KinematicConfigurationType> validConfigurations;
	JointAngleType currentAngles = MainBotController::getInstance().getCurrentAngles();

	bool ok = Kinematics::getInstance().computeInverseKinematics(actuatorLimits, currentAngles, pose, solution,validConfigurations);
	if (ok) {
		MainBotController::getInstance().setAngles(solution.angles);
		MainBotController::getInstance().setPose(pose);
		MainBotController::getInstance().setConfiguration(solution.config);
		MainBotController::getInstance().setPossibleConfigurations(validConfigurations);
	}
	return ok;
}

MainBotController::MainBotController() {
}

void MainBotController::setup() {
	currJointAngles = {0,0,0,0,0,0,radians(35.0)};
	botWindowCtrl.setTcpInputCallback(TCPInputCallback);
	botWindowCtrl.setAnglesCallback(anglesCallback);
	Kinematics::getInstance().computeForwardKinematics(currJointAngles, currTCP);
}

void MainBotController::loop() {
	delay(10);
}

void MainBotController::computeAngles(const Pose& tcp, const JointAngleType& currAngles, KinematicsSolutionType& angles) {
}

void MainBotController::computePose(const JointAngleType& currAngles, Pose& tcp) {

}
