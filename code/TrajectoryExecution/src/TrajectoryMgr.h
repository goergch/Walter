/*
 * TrajectoryMgr.h
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORYMGR_H_
#define TRAJECTORYMGR_H_

#include "Trajectory.h"

class TrajectoryMgr {
public:
	TrajectoryMgr();
	static TrajectoryMgr& getInstance();
	Trajectory& getTrajectory() { return trajectory; };
private:
	Trajectory trajectory;
};


#endif /* TRAJECTORYMGR_H_ */
