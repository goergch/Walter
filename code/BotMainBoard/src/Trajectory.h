/*
 * Trajectory.h
 *
 *  Created on: 13.08.2016
 *      Author: JochenAlt
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "MainBotController.h"

class TrajectoryNode {
public:
	TrajectoryNode() {
		smooth = 0;
		duration = 0;
		angles = { 0,0,0,0,0,0,0 };
	}
	TrajectoryNode(const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
	}
	void operator= (const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
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
	Pose pose;
	JointAngleType angles;
	bool smooth;
	float duration;
	string name;
};

class Trajectory {
public:
	Trajectory();

	static Trajectory& getInstance() {
				static Trajectory instance;
				return instance;
	}

	vector<TrajectoryNode>& getTrajectory() { return trajectory; };
	TrajectoryNode& getTrajectoryNode(int idx) { return trajectory[idx]; };
private:
	vector<TrajectoryNode> trajectory;
};

#endif /* TRAJECTORY_H_ */
