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
#include "TrajectoryMgr.h"


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
	// if kinematics cannot find a solution, nothing has been set, old pose remains
	return ok;
}

bool poseInputCallback(const Pose& pose) {
	return MainBotController::getInstance().setPose(pose);
}

// called when angles have been changed in ui and kinematics need to be recomputed
void anglesInputCallback(const JointAngleType& pAngles) {
	Pose pose;
	PoseConfigurationType config;
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
		uint32_t currentTime = millis()-trajectoryPlayerStartTime;
		if ((currentTime  > trajectoryPlayerTime_ms+TrajectoryPlayerSampleRate_ms) && BotWindowCtrl::getInstance().readyForControllerEvent() ) {
			Trajectory& trajectory = TrajectoryMgr::getInstance().getTrajectory();
			if (trajectoryPlayerTime_ms > trajectory.getDurationMS()) {
				TrajectoryNode node = trajectory.getNodeByTime(trajectoryPlayerTime_ms, true);
				if (!node.isNull()) {
					setPose(node.pose);
				}

				stopTrajectory();
			}
			else {
				TrajectoryNode node = trajectory.getNodeByTime(trajectoryPlayerTime_ms, true);
				if (!node.isNull()) {
					setPose(node.pose);
				}
			}
			trajectoryPlayerTime_ms += TrajectoryPlayerSampleRate_ms;
		}
	}
	delay(10); // for avoiding huge cpu load
}


void MainBotController::playTrajectory() {
	Trajectory& trajectory = TrajectoryMgr::getInstance().getTrajectory();
	if (trajectory.size() > 1) {

		int idx = trajectory.selected();
		if ((idx == -1) || (idx == (int)trajectory.size() -1))
			idx = 0;
		trajectory.select(idx);

		TrajectoryNode startNode = trajectory.get(idx);
		setAnglesImpl(startNode.angles);
		// setPose(startNode.pose);
		trajectoryPlayerTime_ms = startNode.time_ms;
		trajectoryPlayerStartTime = millis() - trajectoryPlayerTime_ms;
		trajectoryPlayerOn = true;
	}
}
void MainBotController::stopTrajectory() {
	trajectoryPlayerOn = false;
}
void MainBotController::resetTrajectory() {
	trajectoryPlayerOn = false;
	trajectoryPlayerTime_ms = 0;
	trajectoryPlayerStartTime = millis();
	// reset selected node to the beginning
	if (TrajectoryMgr::getInstance().getTrajectory().size() > 0)
		TrajectoryMgr::getInstance().getTrajectory().select(0);
}

