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
	vector<TrajectoryNode>& getList() { return trajectory; };
	TrajectoryNode& get(int idx);
	TrajectoryNode& select(int idx);
	int  selected();
	int size() { return trajectory.size(); };

	TrajectoryNode getNodeByTime(int time_ms, bool select);

	unsigned int getDurationMS();

	string marshal(const Trajectory& t);
	Trajectory unmarshal(string s);

	string toString() const;
	bool fromString(const string& str, int &idx);


	void save(string filename);
	void load(string filename);
	void merge(string filename);

private:

	TrajectoryNode getCached(int time);
	bool isCached(int time);

	void setCache(int time, const TrajectoryNode& node);
	void clearCache();

	vector<TrajectoryNode> trajectory; // support nodes
	vector<BezierCurve> interpolation; // bezier curves between support nodes
	vector<TrajectoryNode> cache; // interpolated points including kinematics, used to compute kinematics only once

	int currentTrajectoryNode;
};



#endif /* TRAJECTORY_H_ */
