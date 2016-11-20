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

	// call me upfront before doing anything
	void setup();
	virtual void notifyNewPose(const Pose& pose);
	void loop();

	void receiveFromRealBot(bool yesOrNo);
	void sendToRealBot(bool yesOrNo);

private:
	bool retrieveFromRealBotFlag;
	bool sendToRealBotFlag;

};



#endif /* BOTCONTROLLER_H_ */
