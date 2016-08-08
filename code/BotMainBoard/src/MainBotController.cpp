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
void anglesCallback(JointAngleType pAngles, Pose &pose, KinematicConfigurationType &config) {
	Kinematics::getInstance().computeForwardKinematics(pAngles, pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	MainBotController::getInstance().setCurrentAngles(pAngles);

}

void TCPInputCallback(Pose pose, KinematicConfigurationType &config, JointAngleType &angles, std::vector<KinematicConfigurationType>& validConfigurations) {
	KinematicsSolutionType solutions;
	JointAngleType currentAngles = MainBotController::getInstance().getCurrentAngles();
	Kinematics::getInstance().computeInverseKinematics(actuatorLimits, currentAngles, pose, solutions,validConfigurations);
	angles = solutions.angles;
	config = solutions.config;
	MainBotController::getInstance().setCurrentAngles(angles);
}

MainBotController::MainBotController() {
	currJointAngles = {0,0,0,0,0,0,30.0};
}

void MainBotController::setup() {
	botWindowCtrl.setTcpInputCallback(TCPInputCallback);
	botWindowCtrl.setAnglesCallback(anglesCallback);
}

void MainBotController::loop() {
	delay(10);
}

void MainBotController::computeAngles(const Pose& tcp, const JointAngleType& currAngles, KinematicsSolutionType& angles) {
}

void MainBotController::computePose(const JointAngleType& currAngles, Pose& tcp) {

}
