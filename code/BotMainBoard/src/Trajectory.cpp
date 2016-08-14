/*
 * Trajectory.cpp
 *
 *  Created on: 13.08.2016
 *      Author: SuperJochenAlt
 */

#include <Trajectory.h>

const int TrajectorySampleTime_ms = 100;
Trajectory::Trajectory() {
}

void Trajectory::compile() {
	// update starting times per node
	int currTime_ms = 0;
	interpolation.clear();

	if (trajectory.size() > 1) {
		// set interpolation
		interpolation.resize(trajectory.size()-1);

		for (unsigned int i = 0;i<trajectory.size();i++) {
			trajectory[i].time_ms = currTime_ms;
			currTime_ms += trajectory[i].duration_ms;
			TrajectoryNode& curr = trajectory[i];
			if (i+1 < trajectory.size()) {
				TrajectoryNode next = trajectory[i+1];
				TrajectoryNode prev;
				TrajectoryNode nextnext;
				if (i>0)
					prev = trajectory[i-1];
				if (i+2 < trajectory.size())
					nextnext = trajectory[i+2];
				interpolation[i].set(prev, curr,next, nextnext);
			}
		}
	}
}


TrajectoryNode Trajectory::getTrajectoryNodeByTime(int time_ms) {
	// find node that starts right before time_ms
	unsigned int idx = 0;
	while ((idx < trajectory.size()) && (trajectory[idx].time_ms + trajectory[idx].duration_ms< time_ms)) {
		idx++;
	}
	if ((trajectory.size() > 0) && (trajectory[idx].time_ms <= time_ms)) {
		TrajectoryNode startNode= trajectory[idx];
		BezierCurve bezier = interpolation[idx];
		float t = ((float)time_ms-startNode.time_ms) / ((float)startNode.duration_ms);
		TrajectoryNode node = bezier.getCurrent(t);
		return node;
	}
	return TrajectoryNode(); // return null value
}

unsigned int Trajectory::duration_ms() {
	int sum_ms = 0;
	for (unsigned i = 0;i<trajectory.size()-1;i++) {
		sum_ms += trajectory[i].duration_ms;
	}
	return sum_ms;
}
