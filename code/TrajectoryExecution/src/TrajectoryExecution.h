/*
 * TrajectoryMgr.h
 *
 * Class that moves the real bot by playing the trajectory and calling the uC with single move commands
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

	// call as often as possible. Runs the trajectory by computing a support point every TrajectorySampleRate
	// and call notifyNewPose where communication with uC happens
	void loop();

	// send a direct command to uC. Used in console only
	void directAccess(string cmd, string& response, bool &okOrNOk);

	// log everything from uC to cout. Used when directly access the uC
	void loguCToConsole();

	// get the current trajectory node of the robot as string. Used to display it in the UI
	string currentTrajectoryNodeToString();

	// set the current angles in stringified form
	void setAnglesAsString(string angles);

	// set the current trajectory to be player
	void runTrajectory(const string& trajectory);

	// set the current pose to the bot
	void setPose(const string& pose);

	// is called by TrajectoryPlayer whenever a new pose is computed
	void notifyNewPose(const Pose& pPose);

	// switch on power and move bot into default position
	bool startupBot();

	// move into default position and power down
	bool teardownBot();

private:
	uint32_t lastLoopInvocation;
};


#endif /* TRAJECTORYMGR_H_ */
