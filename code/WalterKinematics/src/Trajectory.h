/*
 * Trajectory.h
 *
 * Manages a trajectory of spatial support points and certain interpolation attributes
 *
 * Author: JochenAlt
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

	// compute speed profile and interpolation points out of given trajectory
	void compile();

	// returns the trajectory node vector. Supposed to be used for adding new nodes
	vector<TrajectoryNode>& getSupportNodes() { return trajectory; };

	// get one certain trajectory node
	TrajectoryNode& get(int idx);

	// selecting one node is just an attribute, returned by selected(), no functional meaning here (used as cursor)
	TrajectoryNode& select(int idx);

	// return the index of the node selected by select(int)
	int  selected();

	// number of trajectory nodes.
	int size() { return trajectory.size(); };

	// return an interpolated node by time.
	TrajectoryNode getCompiledNodeByTime(milliseconds time);

	// returns duration of entire trajectory
	milliseconds getDuration();

	// get a string representation of the trajectory and vice versa
	static string marshal(const Trajectory& t);
	static Trajectory unmarshal(string s);

	// get a string out of the applied trajectory
	string toString() const;

	// assign the trajectory from the passed string at index idx
	bool fromString(const string& str, int &idx);

	// save trajectory to a file
	void save(string filename);

	// load trajectory from file. Existing trajectory is deleted.
	void load(string filename);

	// merge trajectory to existing trajectory
	void merge(string filename);
private:
	TrajectoryNode computeNodeByTime(milliseconds time, bool select);
	TrajectoryNode getCurvePoint(int time);
	bool isCurveAvailable(int time);

	void setCurvePoint(int time, const TrajectoryNode& node);
	void clearCurve();

	vector<TrajectoryNode> trajectory; 		// defined support nodes
	vector<BezierCurve> interpolation; 		// bezier curves between support nodes
	vector<SpeedProfile> speedProfile; 		// speed profile between support nodes

	vector<TrajectoryNode> compiledCurve; 	// compiled interpolated points including kinematics.

	int currentTrajectoryNode;
};



#endif /* TRAJECTORY_H_ */
