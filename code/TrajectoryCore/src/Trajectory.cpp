/*
 * Trajectory.cpp
 *
 *  Created on: 13.08.2016
 *      Author: JochenAlt
 */

#include "Trajectory.h"

const int TrajectorySampleTime_ms = 100;

Trajectory::Trajectory(const Trajectory& t) {
	trajectory = t.trajectory;
	interpolation = t.interpolation;
	currentTrajectoryNode = t.currentTrajectoryNode;
}
void Trajectory::operator=(const Trajectory& t) {
	trajectory = t.trajectory;
	interpolation = t.interpolation;
	currentTrajectoryNode = t.currentTrajectoryNode;
}

Trajectory::Trajectory() {
	currentTrajectoryNode = -1;// no currently selected node
}

void Trajectory::compile() {
	// update starting times per node
	int currTime_ms = 0;
	interpolation.clear();

	if (trajectory.size() > 1) {
		// set interpolation
		interpolation.resize(trajectory.size()-1);

		// compute timing first, since this is required for interpolation
		for (unsigned int i = 0;i<trajectory.size();i++) {
			trajectory[i].time_ms = currTime_ms;
			currTime_ms += trajectory[i].duration_ms;
		}

		for (unsigned int i = 0;i<trajectory.size();i++) {
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
	if (currentTrajectoryNode >= (int)trajectory.size())
		currentTrajectoryNode = (int)trajectory.size() -1;
}

TrajectoryNode& Trajectory::get(int idx) {
	return trajectory[idx];
};

TrajectoryNode& Trajectory::select(int idx) {
	currentTrajectoryNode = idx;
	return get(idx);
};

int  Trajectory::selected() {
	return currentTrajectoryNode;
}

TrajectoryNode Trajectory::getNodeByTime(int time_ms, bool select) {
	// find node that starts right before time_ms
	unsigned int idx = 0;
	while ((idx < trajectory.size()) && (trajectory[idx].time_ms + trajectory[idx].duration_ms< time_ms)) {
		idx++;
	}
	if ((trajectory.size() > 0) && (trajectory[idx].time_ms <= time_ms)) {
		TrajectoryNode startNode= trajectory[idx];
		if (select)
			currentTrajectoryNode = idx;
		if (idx < trajectory.size()-1) {
			BezierCurve bezier = interpolation[idx];
			float t = ((float)time_ms-startNode.time_ms) / ((float)startNode.duration_ms);
			TrajectoryNode node = bezier.getCurrent(t);
			return node;
		} else {
			return trajectory[trajectory.size()-1];
		}
	}
	return TrajectoryNode();
}


unsigned int Trajectory::getDurationMS() {
	int sum_ms = 0;
	for (unsigned i = 0;i<trajectory.size()-1;i++) {
		sum_ms += trajectory[i].duration_ms;
	}
	return sum_ms;
}

void Trajectory::save(string filename) {

	ofstream f(filename);
	string str = marshal(*this);
	f << str;
	f.close();
}

string Trajectory::marshal(const Trajectory& t) {
	stringstream str;
	str.precision(3);

	for (unsigned i = 0;i<t.trajectory.size();i++) {
		TrajectoryNode node = t.trajectory[i];
		str << fixed << "id=" << i << endl;
		str << "name=" << node.name << endl;
		str << "duration=" << node.duration_ms << endl;
		str << "position.x=" << node.pose.position.x << endl;
		str << "position.y=" << node.pose.position.y << endl;
		str << "position.z=" << node.pose.position.z << endl;
		str << "orientation.x=" << node.pose.orientation.x << endl;
		str << "orientation.y=" << node.pose.orientation.y << endl;
		str << "orientation.z=" << node.pose.orientation.z << endl;
		str << "gripper=" << node.pose.gripperAngle << endl;
		str << "interpolation=" << (int)node.interpolationType<< endl;
	}

	return str.str();
}

Trajectory  Trajectory::unmarshal(string str) {
	stringstream f(str);
	TrajectoryNode node;
	Trajectory result;
	bool nodePending = false;
	string line;
	while (getline(f, line)) {
		if (string_starts_with(line, "id=")) {
			if (nodePending) {
				result.trajectory.insert(result.trajectory.end(), node);
				nodePending = false;
			}
		} else {
			nodePending = true;
		}
		if (string_starts_with(line, "name=")) {
			char buffer[256];
			buffer[0] = 0;
            sscanf(line.c_str(),"name=%s", &buffer[0]);
            node.name = buffer;
		}
        sscanf(line.c_str(),"duration=%i", &node.duration_ms);
        sscanf(line.c_str(),"position.x=%lf", &node.pose.position.x);
        sscanf(line.c_str(),"position.y=%lf", &node.pose.position.y);
        sscanf(line.c_str(),"position.z=%lf", &node.pose.position.z);
        sscanf(line.c_str(),"orientation.x=%lf", &node.pose.orientation.x);
        sscanf(line.c_str(),"orientation.y=%lf", &node.pose.orientation.y);
        sscanf(line.c_str(),"orientation.z=%lf", &node.pose.orientation.z);
        sscanf(line.c_str(),"gripper=%lf", &node.pose.gripperAngle);
        sscanf(line.c_str(),"interpolation=%i", (int*)&node.interpolationType);
	}
	if (nodePending) {
		result.trajectory.insert(result.trajectory.end(), node);
		nodePending = false;
	}
	return result;
}

void Trajectory::load(string filename) {
	trajectory.clear();
	interpolation.clear();
	currentTrajectoryNode= -1;
	merge(filename);
}

void Trajectory::merge(string filename) {
	ifstream f(filename);
	TrajectoryNode node;
	string str;
	while(!f.eof())
	{
		string line;
		getline(f, line);
		str += line;
		str += LFCR;
	}
	f.close();
	Trajectory toBeMerged = unmarshal(str);
	for (unsigned i = 0;i<toBeMerged.trajectory.size();i++) {
		trajectory.insert(trajectory.end(), toBeMerged.trajectory[i]);
	}

}

bool Trajectory::fromString(const string& str, int &idx) {
	int noOfEntries = 0;
    int noOfItems = sscanf(&(str.c_str()[idx]),"{trajectory{n=%i;%n",&noOfEntries, &idx);
    for (int i = 0;i<noOfEntries;i++) {
    	TrajectoryNode& node = trajectory[i];
        bool ok = node.fromString(str,idx);
        if (!ok) {
        	LOG(ERROR) << "trajectory parse error";
        }
    }
    noOfItems += sscanf(&(str.c_str()[idx]),"}%n", &idx);

    return (noOfItems == 3);
}


string Trajectory::toString() const {

	stringstream str;
	str.precision(3);
	str << "{trajectory{n=" << trajectory.size() + ";";
	for (unsigned i = 0;i< trajectory.size();i++) {
		str << trajectory[i].toString();
	}
	str << "}";

	return str.str();
}
