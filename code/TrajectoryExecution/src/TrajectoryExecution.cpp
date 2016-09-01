/*
 * TrajectoryExecController.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecution.h"
#include "ActuatorCtrlInterface.h"

TrajectoryExecution::TrajectoryExecution() {

}

TrajectoryExecution& TrajectoryExecution::getInstance() {
	static TrajectoryExecution instance;
	return instance;
}


bool TrajectoryExecution::setup() {
	bool ok = ActuatorCtrlInterface::getInstance().setupCommunication();

	if (!ok) {
    	LOG(ERROR) << "uC not present";
	}

	TrajectoryPlayer::setup();

	return ok;
}


// send a direct command to uC
void TrajectoryExecution::directAccess(string cmd, string& response, bool &okOrNOk) {
	ActuatorCtrlInterface::getInstance().directAccess(cmd, response, okOrNOk);
}

void TrajectoryExecution::loguCToConsole() {
	ActuatorCtrlInterface::getInstance().loguCToConsole();
}





