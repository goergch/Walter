/*
 * MainBotController.h
 *
 * Main controller for everything. Coordinates UI with kinematics and trajectory control.
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#ifndef MAIN_BOTCONTROLLER_H_
#define MAIN_BOTCONTROLLER_H_

#include "spatial.h"
#include "Kinematics.h"
#include "Trajectory.h"
#include "TrajectoryPlayer.h"

class TrajectorySimulation : public TrajectoryPlayer {
public:
	TrajectorySimulation();
	static TrajectorySimulation& getInstance() {
			static TrajectorySimulation instance;
			return instance;
	}

	void setup(int pSampleRate /* ms */);

	// called by TrajectoryPlayer whenever a new pose is
	// computed. Sends an event to UI to update the pose
	virtual void notifyNewPose(const Pose& pose);

	// to be called often
	void loop();

	// define if we want to receive pose of bot via cortex
	void receiveFromRealBot(bool yesOrNo);

	// define if we want to send the current UI pose directly to the bot via cortex
	void sendToRealBot(bool yesOrNo);

	// true if bot is up and running
	bool botIsUpAndRunning();

	// initiate bot startup procedure, like switch on everything and move to default position
	void setupBot();

	// switch off in a controlled manner
	void teardownBot();

	// true, if a request for a heartbeat has been sent. Returns only one "true" per heartbeat.
	bool heartBeatSendOp();

	// true, if a heartbeat has been received. Returns only one "true" per heartbeat.
	bool heartBeatReceiveOp();

private:
	bool retrieveFromRealBotFlag;
	bool sendToRealBotFlag;
	bool sendOp = false;
	bool receiveOp = false;

	milliseconds lastLoopTime = 0;
};



#endif /* BOTCONTROLLER_H_ */
