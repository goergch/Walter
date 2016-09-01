/*
 * TrajectoryMgr.h
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORYMGR_H_
#define TRAJECTORYMGR_H_

#include "TrajectoryPlayer.h"

class TrajectoryExecution : public TrajectoryPlayer {
public:
	TrajectoryExecution();
	static TrajectoryExecution& getInstance();

	// call me upfront before doing anything
	bool setup();

	// send a direct command to uC
	void directAccess(string cmd, string& response, bool &okOrNOk);

	// log everything from uC to cout. Used for directly access the uC
	void loguCToConsole();

	// get the current pose of the robot as string
	string currentPoseToString();

	// set the current trajectory to be player
	void setTrajectory(const string& trajectory);


private:
};


#endif /* TRAJECTORYMGR_H_ */
