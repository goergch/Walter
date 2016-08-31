/*
 * TrajectoryMgr.h
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORYMGR_H_
#define TRAJECTORYMGR_H_

#include "Trajectory.h"

class TrajectoryExecController {
public:
	TrajectoryExecController();
	static TrajectoryExecController& getInstance();
	Trajectory& getTrajectory() { return trajectory; };


private:

	void computeIK();
	Trajectory trajectory;

};


#endif /* TRAJECTORYMGR_H_ */
