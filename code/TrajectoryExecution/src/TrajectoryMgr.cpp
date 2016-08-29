/*
 * TrajectoryMgr.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryMgr.h"

TrajectoryMgr::TrajectoryMgr() {

}

TrajectoryMgr& TrajectoryMgr::getInstance() {
	static TrajectoryMgr instance;
	return instance;
}
