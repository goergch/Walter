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
		currNode.pose = pPose;
		currNode.pose.angles = solution.angles;
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

void TrajectoryPlayer::setup(int pSampleRate_ms) {
	sampleRate = pSampleRate_ms;
	currNode.pose.angles = Kinematics::getNullPositionAngles();
	Kinematics::getInstance().computeForwardKinematics(currNode.pose);
}

void TrajectoryPlayer::step() {
	trajectoryPlayerTime_ms += sampleRate;
	playerStopped = false;
}

void TrajectoryPlayer::setPlayerPosition(int time_ms) {
	trajectoryPlayerTime_ms = time_ms;
}

void TrajectoryPlayer::loop() {
	if (trajectoryPlayerOn) {
		milliseconds currentTime = millis()-startTime;
		if ((currentTime  >= trajectoryPlayerTime_ms+sampleRate)) {
			if (!playerStopped) {
				if (trajectoryPlayerTime_ms > trajectory.getDuration()) {
					currNode = trajectory.getCompiledNodeByTime(trajectory.getDuration(), true);
					if (!currNode.isNull()) {
						setPose(currNode.pose);
					}

					stopTrajectory();
				}
				else {
					currNode = trajectory.getCompiledNodeByTime(trajectoryPlayerTime_ms, true);
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

// start playing of the set trajectory by setting the node that corresponds to the current time
void TrajectoryPlayer::playTrajectory() {
	if (trajectory.size() > 1) {

		int idx = trajectory.selected();
		if ((idx == -1) || (idx == (int)trajectory.size() -1))
			idx = 0;
		TrajectoryNode startNode = trajectory.select(idx);
		currNode.pose.angles = startNode.pose.angles;
		currNode.time = startNode.time;
		currNode.interpolationTypeDef = startNode.interpolationTypeDef;

		if (trajectoryPlayerTime_ms >= trajectory.getDuration()) {
			trajectoryPlayerTime_ms = startNode.time;
		}

		// go to start position
		if (!currNode.isNull())
			setPose(currNode.pose);

		// start time has to be a multiple of TrajectorySampleRate
		// first node is displayed here, next is done in ::loop
		trajectoryPlayerTime_ms = (trajectoryPlayerTime_ms/sampleRate)*sampleRate + sampleRate;

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
	currNode.null();
}
void TrajectoryPlayer::resetTrajectory() {
	trajectoryPlayerOn = false;
	trajectoryPlayerTime_ms = 0;
	startTime = millis();
	singleStepMode = false;
	// reset selected node to the beginning
	if (trajectory.size() > 0)
		trajectory.select(0);
}

