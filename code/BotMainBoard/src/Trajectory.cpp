/*
 * Trajectory.cpp
 *
 *  Created on: 13.08.2016
 *      Author: SuperJochenAlt
 */

#include <Trajectory.h>

Trajectory::Trajectory() {
}

void Trajectory::setTrajectoryTiming() {
	int currTime = 0;
	for (unsigned int i = 0;i<trajectory.size();i++) {
		trajectory[i].time_ms = currTime;
		currTime += int(trajectory[i].duration/1000.0);
	}
}
