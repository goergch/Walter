/*
 * TrajectoryMgr.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecController.h"

TrajectoryExecController::TrajectoryExecController() {

}

TrajectoryExecController& TrajectoryExecController::getInstance() {
	static TrajectoryExecController instance;
	return instance;
}

