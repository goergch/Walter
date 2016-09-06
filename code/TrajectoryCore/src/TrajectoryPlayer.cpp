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

	Pose pose = pPose;
	bool ok = Kinematics::getInstance().computeInverseKinematics(pose, solution,validSolutions);
	if (ok) {
		currNode.angles = solution.angles;
		currNode.pose = pPose;
		currentAngles = solution.angles;
		possibleSolutions.clear();
		possibleSolutions = validSolutions;

		notifyNewPose(currNode.pose); // inform the subclass
	}
	// if kinematics cannot find a solution, nothing has been set, old pose remains
	return ok;
}



// called when angles have been changed in ui and kinematics need to be recomputed
void TrajectoryPlayer::setAngles(const JointAngles& pAngles) {
	Pose pose;
	pose.angles = pAngles;
	currentAngles = pAngles;

	Kinematics::getInstance().computeForwardKinematics(pose);
	setPose(pose);
}

TrajectoryPlayer::TrajectoryPlayer() {
	trajectoryPlayerOn  = false;
	resetTrajectory();
}

void TrajectoryPlayer::setup() {
	currNode.angles = Kinematics::getNullPositionAngles();
	Kinematics::getInstance().computeForwardKinematics(currNode.pose);
}

void TrajectoryPlayer::step() {
	trajectoryPlayerTime_ms += TrajectoryPlayerSampleRate;
	playerStopped = false;
}

void TrajectoryPlayer::setPlayerPosition(int time_ms) {
	trajectoryPlayerTime_ms = time_ms;
}

void TrajectoryPlayer::loop() {
	if (trajectoryPlayerOn) {
		milliseconds currentTime = millis()-startTime;
		if ((currentTime  > trajectoryPlayerTime_ms+TrajectoryPlayerSampleRate)) {
			if (!playerStopped) {
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
				if (singleStepMode)
					playerStopped = true;
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
		currNode.angles = startNode.angles;
		currNode.time = startNode.time;
		currNode.interpolationType = startNode.interpolationType;

		trajectoryPlayerTime_ms = startNode.time;
		startTime = millis() - trajectoryPlayerTime_ms;
		trajectoryPlayerOn = true;
		singleStepMode = false;
		playerStopped = false;
	}
}

void TrajectoryPlayer::stepTrajectory() {
	if (trajectory.size() > 1) {
		playTrajectory();
		singleStepMode = true;
	}
}
void TrajectoryPlayer::stopTrajectory() {
	trajectoryPlayerOn = false;
}
void TrajectoryPlayer::resetTrajectory() {
	trajectoryPlayerOn = false;
	trajectoryPlayerTime_ms = 0;
	startTime = millis();
	// reset selected node to the beginning
	if (trajectory.size() > 0)
		trajectory.select(0);
}

