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
#include "Trajectory.h"


const int trajectoryPlayerSampleRate_ms = 100;

bool MainBotController::setPose(const Pose& pPose) {
	KinematicsSolutionType solution;
	std::vector<KinematicsSolutionType> validSolutions;
	JointAngleType currentAngles = MainBotController::getInstance().getCurrentAngles();

	bool ok = Kinematics::getInstance().computeInverseKinematics(currentAngles, pPose, solution,validSolutions);
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
	trajectoryPlayerOn  = false;
	resetTrajectory();
}

void MainBotController::setup() {
	currJointAngles = {0,0,0,0,0,0,radians(35.0)};

	// callbacks from UI: inform me when any data  has changed
	BotWindowCtrl::getInstance().setTcpInputCallback(poseInputCallback);
	BotWindowCtrl::getInstance().setAnglesCallback(anglesInputCallback);
	Kinematics::getInstance().computeForwardKinematics(currJointAngles, currPose);

	// carry out inverse kinematics to get alternative solutions
	poseInputCallback(currPose);
}

void MainBotController::loop() {
	if (trajectoryPlayerOn) {
		if ((millis() > trajectoryPlayerStartTime+trajectoryPlayerTime_ms+1000) /* && BotWindowCtrl::getInstance().readyForControllerEvent() */) {
			Trajectory& trajectory = Trajectory::getInstance();
			if (trajectoryPlayerTime_ms > trajectory.duration_ms())
				stopTrajectory();
			else {
				TrajectoryNode node = trajectory.getTrajectoryNodeByTime(trajectoryPlayerTime_ms);
				if (!node.isNull())
					setPose(node.pose);
			}
			trajectoryPlayerTime_ms += 1000;
		}
	}
	delay(10);
}


void MainBotController::playTrajectory() {
	resetTrajectory();
	Trajectory& trajectory = Trajectory::getInstance();
	if (trajectory.getTrajectory().size() > 1) {

		TrajectoryNode startNode = trajectory.getTrajectory()[0];
		setAnglesImpl(startNode.angles);
		// setPose(startNode.pose);
		trajectoryPlayerTime_ms = 0;
		trajectoryPlayerStartTime = millis();
		trajectoryPlayerOn = true;
	}
}
void MainBotController::stopTrajectory() {
	trajectoryPlayerOn = false;
}
void MainBotController::resetTrajectory() {
	trajectoryPlayerTime_ms = 0;
	trajectoryPlayerStartTime = millis();
	trajectoryPlayerOn = false;
}

void MainBotController::forwardTrajectory() {

}
void MainBotController::backTrajectory() {

}
