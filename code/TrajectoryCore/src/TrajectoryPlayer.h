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

	// get currently computed trajectory node
	const TrajectoryNode& getCurrentTrajectoryNode() { return currNode; };

	// get currently computed pose
	const Pose& getCurrentPose() { return currNode.pose; };

	PoseConfigurationType getCurrentConfiguration() { return Kinematics::computeConfiguration(currNode.angles); };
	// get current angles
	const JointAngles& getCurrentAngles() { return currNode.angles; };

	// retrieve all valid solutions of the latest given pose
	const std::vector<KinematicsSolutionType>& getPossibleSolutions() { return possibleSolutions;}

	// set new angles compute kinematics and send notification
	void setAngles(const JointAngles& pAngles);

	// set new pose, compute kinematics and send notification
	bool setPose(const Pose& pPose);

	// current trajectory
	Trajectory& getTrajectory() { return trajectory; };

	// start to run the current trajectory
	void playTrajectory();

	// start to stepwise run the current trajectory
	void stepTrajectory();

	// execute one step
	void step(); // perform one step

	// true if player is runnin (complete or stepwise)
	bool isOn() { return trajectoryPlayerOn; }

	// set player position to a certain point in time
	void setPlayerPosition(int time_ms);

	// stop it
	void stopTrajectory();

	// reset pose to first position of trajectory
	void resetTrajectory();

	// to be derived. Notification if a new pose has been computed
	virtual void notifyNewPose(const Pose& pose) {};
private:
	TrajectoryNode currNode;
	JointAngles currentAngles;
	std::vector<KinematicsSolutionType> possibleSolutions;

	uint32_t trajectoryPlayerTime_ms;
	bool trajectoryPlayerOn;
	bool stopAfterStep;
	bool stopped;
	uint32_t trajectoryPlayerStartTime;
	Trajectory trajectory;
};



#endif /* TRAJECTORYPLAYER_H_ */
