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
	Trajectory(const Trajectory& t);
	void operator=(const Trajectory& t);
	void compile();
	vector<TrajectoryNode>& getSupportNodes() { return trajectory; };
	TrajectoryNode& get(int idx);
	TrajectoryNode& select(int idx);
	int  selected();
	int size() { return trajectory.size(); };

	TrajectoryNode computeNodeByTime(milliseconds time, bool select);
	TrajectoryNode getCompiledNodeByTime(milliseconds time, bool select);

	milliseconds getDuration();

	string marshal(const Trajectory& t);
	Trajectory unmarshal(string s);

	string toString() const;
	bool fromString(const string& str, int &idx);


	void save(string filename);
	void load(string filename);
	void merge(string filename);

private:

	TrajectoryNode getCurvePoint(int time);
	bool isCurveAvailable(int time);

	void setCurvePoint(int time, const TrajectoryNode& node);
	void clearCurve();

	vector<TrajectoryNode> trajectory; 	// support nodes
	vector<BezierCurve> interpolation; 	// bezier curves between support nodes
	vector<TrajectoryNode> curve; 		// interpolated points including kinematics, used to compute kinematics only once

	int currentTrajectoryNode;
};



#endif /* TRAJECTORY_H_ */
