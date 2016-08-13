/*
 * Trajectory.h
 *
 *  Created on: 13.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "spatial.h"
#include "MainBotController.h"

class TrajectoryNode {
public:

	TrajectoryNode() {
		smooth = 0;
		duration = 0;
		angles = { 0,0,0,0,0,0,0 };
		interpolationType = CUBIC_BEZIER;
		time_ms = 0;
		pose.null();
	}
	TrajectoryNode(const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
		interpolationType = par.interpolationType;
		time_ms = par.time_ms;
	}
	void operator= (const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
		interpolationType = par.interpolationType;
		time_ms = par.time_ms;
	}


	string getText() {

		int par[7];
		int j=0;
		for (int i = 0;i<3;i++)
			par[j++] = pose.position[i];

		for (int i = 0;i<3;i++)
			par[j++] = pose.orientation[i];

		par[j++] = pose.gripperAngle;

		string text = string_format("%s (%i,%i,%i)(%i,%i,%i)(%i)",
							name.c_str(),
							par[0],par[1],par[2],
							par[3],par[4],par[5],
							par[6]);
		return text;
	}
	bool isNull() {
		return pose.isNull();
	}
	void null() {
		pose.null();
	}
	Pose pose;
	JointAngleType angles;
	bool smooth;
	float duration;
	string name;
	InterpolationType interpolationType;
	int time_ms;
};

class Trajectory {
public:
	Trajectory();

	static Trajectory& getInstance() {
				static Trajectory instance;
				return instance;
	}

	void setTrajectoryTiming();
	vector<TrajectoryNode>& getTrajectory() { return trajectory; };
	TrajectoryNode& getTrajectoryNode(int idx) { return trajectory[idx]; };
private:
	vector<TrajectoryNode> trajectory;
};

#endif /* TRAJECTORY_H_ */
