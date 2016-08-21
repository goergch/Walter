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
	TrajectoryNode& getTrajectoryNode(int idx);
	TrajectoryNode& selectNode(int idx);
	int  selectedNode();
	int nodes() { return trajectory.size(); };

	TrajectoryNode getTrajectoryNodeByTime(int time_ms, bool select);
	unsigned int duration_ms();

	void save();
	void load(string filename);
	void merge(string filename);

private:
	vector<TrajectoryNode> trajectory;
	vector<BezierCurve> interpolation;
	int currentTrajectoryNode;
};

#endif /* TRAJECTORY_H_ */
