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

	bool ok = Kinematics::getInstance().computeInverseKinematics(currJointAngles, pPose, solution,validSolutions);
	if (ok) {
		currJointAngles = solution.angles;
		currPose = pPose;
		currConfiguration = solution.config;
		possibleSolutions = validSolutions;

		notifyNewPose(currPose); // inform the subclass
	}
	// if kinematics cannot find a solution, nothing has been set, old pose remains
	return ok;
}



// called when angles have been changed in ui and kinematics need to be recomputed
void TrajectoryPlayer::setAngles(const JointAngleType& pAngles) {
	Pose pose;
	PoseConfigurationType config;
	Kinematics::getInstance().computeForwardKinematics(pAngles, pose);
	Kinematics::getInstance().computeConfiguration(pAngles, config);
	setPose(pose);
}

TrajectoryPlayer::TrajectoryPlayer() {
	trajectoryPlayerOn  = false;
	resetTrajectory();
}

void TrajectoryPlayer::setup() {
	currJointAngles = {0,0,0,0,0,0,radians(35.0)};
	Kinematics::getInstance().computeForwardKinematics(currJointAngles, currPose);
}

void TrajectoryPlayer::loop() {
	if (trajectoryPlayerOn) {
		uint32_t currentTime = millis()-trajectoryPlayerStartTime;
		if ((currentTime  > trajectoryPlayerTime_ms+TrajectoryPlayerSampleRate_ms)) {
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
}

void TrajectoryPlayer::playTrajectory() {
	if (trajectory.size() > 1) {

		int idx = trajectory.selected();
		if ((idx == -1) || (idx == (int)trajectory.size() -1))
			idx = 0;
		trajectory.select(idx);

		TrajectoryNode startNode = trajectory.get(idx);
		currJointAngles = startNode.angles;
		trajectoryPlayerTime_ms = startNode.time_ms;
		trajectoryPlayerStartTime = millis() - trajectoryPlayerTime_ms;
		trajectoryPlayerOn = true;
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

