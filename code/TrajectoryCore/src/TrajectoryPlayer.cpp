/*
 * TrajectoryPlayer.cpp
 *
 *  Created on: 31.08.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include "Util.h"
#include "TrajectoryPlayer.h"

bool TrajectoryPlayer::setPose(const Pose& pPose) {
	KinematicsSolutionType solution;
	std::vector<KinematicsSolutionType> validSolutions;

	bool ok = Kinematics::getInstance().computeInverseKinematics(currentAngles, pPose, solution,validSolutions);
	if (ok) {
		currNode.angles = solution.angles.getLegacy();
		currNode.pose = pPose;
		currConfiguration = solution.config;
		possibleSolutions.clear();
		possibleSolutions = validSolutions;
		currentAngles = solution.angles.getLegacy();

		notifyNewPose(currNode.pose); // inform the subclass
	}
	// if kinematics cannot find a solution, nothing has been set, old pose remains
	return ok;
}



// called when angles have been changed in ui and kinematics need to be recomputed
void TrajectoryPlayer::setAngles(const JointAngleType& pAngles) {
	Pose pose;
	PoseConfigurationType config;
	currentAngles = pAngles;
	pose.angles = pAngles;
	Kinematics::getInstance().computeForwardKinematics(pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	setPose(pose);
}

TrajectoryPlayer::TrajectoryPlayer() {
	trajectoryPlayerOn  = false;
	resetTrajectory();
}

void TrajectoryPlayer::setup() {
	currentAngles = Kinematics::getDefaultAngles().getLegacy();
	currNode.angles = currentAngles;
	Kinematics::getInstance().computeForwardKinematics(currNode.pose);
}

void TrajectoryPlayer::step() {
	trajectoryPlayerTime_ms += TrajectoryPlayerSampleRate_ms;
	stopped = false;
}

void TrajectoryPlayer::setPlayerPosition(int time_ms) {
	trajectoryPlayerTime_ms = time_ms;
}

void TrajectoryPlayer::loop() {
	if (trajectoryPlayerOn) {
		uint32_t currentTime = millis()-trajectoryPlayerStartTime;
		if ((currentTime  > trajectoryPlayerTime_ms+TrajectoryPlayerSampleRate_ms)) {
			if (!stopped) {
				if (trajectoryPlayerTime_ms > trajectory.getDurationMS()) {
					currNode = trajectory.getCurveNodeByTime(trajectory.getDurationMS(), true);
					if (!currNode.isNull()) {
						setPose(currNode.pose);
					}

					stopTrajectory();
				}
				else {
					currNode = trajectory.getCurveNodeByTime(trajectoryPlayerTime_ms, true);
					if (!currNode.isNull()) {
						setPose(currNode.pose);
					}
				}
				if (stopAfterStep)
					stopped = true;
				else
					step();
			}
		}
	}
}

void TrajectoryPlayer::playTrajectory() {
	if (trajectory.size() > 1) {

		int idx = trajectory.selected();
		if ((idx == -1) || (idx == (int)trajectory.size() -1))
			idx = 0;
		TrajectoryNode startNode = trajectory.select(idx);
		currentAngles = startNode.angles;
		currNode.angles = currentAngles;
		currNode.time_ms = startNode.time_ms;
		currNode.interpolationType = startNode.interpolationType;

		trajectoryPlayerTime_ms = startNode.time_ms;
		trajectoryPlayerStartTime = millis() - trajectoryPlayerTime_ms;
		trajectoryPlayerOn = true;
		stopAfterStep = false;
		stopped = false;
	}
}

void TrajectoryPlayer::stepTrajectory() {
	if (trajectory.size() > 1) {
		playTrajectory();
		stopAfterStep = true;
	}
}
void TrajectoryPlayer::stopTrajectory() {
	trajectoryPlayerOn = false;
}
void TrajectoryPlayer::resetTrajectory() {
	trajectoryPlayerOn = false;
	trajectoryPlayerTime_ms = 0;
	trajectoryPlayerStartTime = millis();
	// reset selected node to the beginning
	if (trajectory.size() > 0)
		trajectory.select(0);
}

