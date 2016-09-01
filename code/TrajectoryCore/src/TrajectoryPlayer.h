/*
 * TrajectoryPlayer.h
 *
 *  Created on: 31.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef TRAJECTORYPLAYER_H_
#define TRAJECTORYPLAYER_H_

#include "spatial.h"
#include "Kinematics.h"
#include "Trajectory.h"

class TrajectoryPlayer {
public:
	TrajectoryPlayer();

	// call me upfront before doing anything
	void setup();
	// call this often. Does the trajectory computation
	void loop();

	const TrajectoryNode& getCurrentTrajectoryNode() { return currNode; };
	const Pose& getCurrentPose() { return currNode.pose; };
	const JointAngleType& getCurrentAngles() { return currentAngles; };
	const PoseConfigurationType& getCurrentConfiguration() { return currConfiguration; };
	void selectConfiguration(const PoseConfigurationType& config ) { currConfiguration = config;};
	const std::vector<KinematicsSolutionType>& getPossibleSolutions() { return possibleSolutions;}

	// set new pose, compute kinematics and send notification
	void setAngles(const JointAngleType& pAngles);
	bool setPose(const Pose& pPose);

	Trajectory& getTrajectory() { return trajectory; };
	// trajectory player
	void playTrajectory();
	void stopTrajectory();
	void resetTrajectory();

	// to be derived
	virtual void notifyNewPose(const Pose& pose) {};
private:
	TrajectoryNode currNode;
	PoseConfigurationType currConfiguration;
	JointAngleType currentAngles;
	// Pose currPose;
	std::vector<KinematicsSolutionType> possibleSolutions;
	uint32_t trajectoryPlayerTime_ms;
	bool trajectoryPlayerOn;
	uint32_t trajectoryPlayerStartTime;
	Trajectory trajectory;
};



#endif /* TRAJECTORYPLAYER_H_ */
