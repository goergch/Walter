/*
 * Trajectory.h
 *
 *  Created on: 13.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "spatial.h"
#include "BezierCurve.h"

using namespace std;

class Trajectory {
public:
	Trajectory();

	static Trajectory& getInstance() {
		static Trajectory instance;
		return instance;
	}

	void compile();
	vector<TrajectoryNode>& getTrajectory() { return trajectory; };
	TrajectoryNode& getTrajectoryNode(int idx) { return trajectory[idx]; };
	TrajectoryNode getTrajectoryNodeByTime(int time_ms);
	unsigned int duration_ms();

private:
	vector<TrajectoryNode> trajectory;
	vector<BezierCurve> interpolation;
};

#endif /* TRAJECTORY_H_ */
