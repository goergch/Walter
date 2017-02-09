#include "Trajectory.h"
#include "Kinematics.h"
#include "logger.h"
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
	interpolation.clear();
	speedProfile.clear();

	if (trajectory.size() > 1) {
        // clear cache of trajectory nodes
		clearCurve();

		// resize interpolation and profile arrays
		interpolation.resize(trajectory.size()-1);
		speedProfile.resize(trajectory.size()-1);

		// initialize first node
		trajectory[0].time = 0;
		trajectory[0].startSpeed= 0.0;
		trajectory[0].endSpeed= 0.0;
		trajectory[0].distance= 0.0;
		trajectory[0].duration= 0.0;

		float fullDuration= 0;
		float fullDistance= 0;

		for (unsigned int i = 0;i<trajectory.size();i++) {
			TrajectoryNode& curr = trajectory[i];

			// in case there is no user defined name, give it a number
			if (curr.name.empty())
				curr.name = int_to_string(i);

			// depending on the interpolation type, choose the right kinematics computation (forward or inverse)
			if (curr.isPoseInterpolation()) {
				Kinematics::getInstance().computeInverseKinematics(curr.pose);
			} else {
				Kinematics::getInstance().computeForwardKinematics(curr.pose);
			}

			if (i+1 < trajectory.size()) {
				TrajectoryNode& next = trajectory[i+1];

				TrajectoryNode prev(curr);
				TrajectoryNode nextnext(next);
				if (i>0)
					prev = trajectory[i-1];
				if (i+2 < trajectory.size())
					nextnext = trajectory[i+2];

				// compute the bezier curve between this and next point
				interpolation[i].set(prev, curr,next, nextnext);

				// aproximate the distance via the bezier curve
				curr.distance = interpolation[i].curveLength();

				// duration is either user defined, or computed via the average speed
				if (curr.durationDef != 0)
					curr.duration = curr.durationDef;
				else
					curr.duration = milliseconds(curr.distance / curr.averageSpeedDef);

				if (i == 0) {
					// first node, start with speed of 0
					if (i == trajectory.size() - 2) {
						// just two nodes, we are on the first
						curr.startSpeed = 0;
						next.startSpeed = 0;
						next.distance = 0.0;
						bool possibleWithoutAmendments = speedProfile[i].computeSpeedProfile(curr.startSpeed, next.startSpeed, curr.distance, curr.duration);
						if (!possibleWithoutAmendments)
							curr.averageSpeedDef = curr.distance / curr.duration;
					} else {
						// first node, and we have at least three nodes, accelerate to average speed
						curr.startSpeed = 0;
						next.startSpeed = next.averageSpeedDef;
						if (!curr.continouslyDef)
							next.startSpeed = 0;
						bool endSpeedFine = true;
						if (curr.startSpeed != 0) {
							rational computedDuration;
							endSpeedFine = SpeedProfile::getRampProfileDuration(curr.startSpeed, next.startSpeed, curr.distance, computedDuration);
							if (computedDuration>curr.duration)
								curr.duration = computedDuration;
						}
						/* bool possibleWithoutAmendments = */ speedProfile[i].computeSpeedProfile(curr.startSpeed, next.startSpeed, curr.distance, curr.duration);
						if (!endSpeedFine)
							curr.averageSpeedDef = next.startSpeed;
					}
				} else {
					if (i == trajectory.size()-2) {
						if (i == 0) {
							// we have two nodes and are on the last one
							// Dont compute speed profile again
							next.startSpeed = 0;
							next.distance = 0.0;
							next.duration = 0.0;
						} else {
							// next is last node, and we have more than two nodes, we end up with speed of 0
							next.startSpeed = 0;
							bool endSpeedFine = true;
							if (curr.startSpeed != 0) {
								rational computedDuration;
								endSpeedFine = SpeedProfile::getRampProfileDuration(curr.startSpeed, next.startSpeed, curr.distance, computedDuration);
								if (computedDuration>curr.duration)
									curr.duration = computedDuration;
							}
							/*bool possibleWithoutAmendments = */speedProfile[i].computeSpeedProfile(curr.startSpeed, next.startSpeed, curr.distance, curr.duration);
							curr.averageSpeedDef = curr.startSpeed;

							if (!endSpeedFine) {
								// todo: backtracking, end speed not null.
							}
							next.distance = 0.0;
							next.duration = 0.0;
						}
					} else {
						// neither first nor last node, somewhere in the middle.
						if (i == trajectory.size()-3)
							next.startSpeed = nextnext.averageSpeedDef;
						else
							next.startSpeed = next.averageSpeedDef;

						if (!curr.continouslyDef)
							next.startSpeed = 0;

						bool possibleWithoutAmendments = speedProfile[i].computeSpeedProfile(curr.startSpeed, next.startSpeed, curr.distance, curr.duration);
						if (!possibleWithoutAmendments)
							curr.averageSpeedDef = curr.distance / curr.duration;
					}
				}


				next.time = curr.time + curr.duration;

				interpolation[i].getStart() = curr; // assign the computed values into bezier curve
				interpolation[i].getEnd() = next;

				fullDuration += curr.duration;
				fullDistance += curr.distance;
				curr.endSpeed = next.startSpeed;
			}
		}

		// compute compiled curve depending on time slots
		milliseconds startTime = trajectory[0].time;
		milliseconds endTime = fullDuration;
		milliseconds time = startTime;
		TrajectoryNode prev;
		while (time < endTime+UITrajectorySampleRate) {
			TrajectoryNode node = computeNodeByTime(time, false);

			// depending on the interpolation type, choose the right kinematics computation (forward or inverse)
			if (node.isPoseInterpolation())
				Kinematics::getInstance().computeInverseKinematics(node.pose);
			else
				Kinematics::getInstance().computeForwardKinematics(node.pose);

			// change timing from support points to finegrained interpolation
			node.time = time;
			node.duration = UITrajectorySampleRate;
			node.startSpeed = node.averageSpeedDef;
			if (!prev.isNull())
				prev.distance = prev.pose.distance(node.pose);

			// store kinematics in trajectory
			if (time > endTime)
				setCurvePoint(endTime, node);
			else
				setCurvePoint(time, node);

			// next time step
			time += UITrajectorySampleRate;
			prev = node;
		}
	}

	// if a node has been removed, the currently selected node could be out of range
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

TrajectoryNode Trajectory::getCompiledNodeByTime(milliseconds time) {
	TrajectoryNode result;
	if (isCurveAvailable(time)) {
		result = getCurvePoint(time);
		return result;
	}

	result = getCurvePoint(time);
	return result;
}

TrajectoryNode Trajectory::computeNodeByTime(milliseconds time, bool select) {
	// find node that starts right before time_ms
	unsigned int idx = 0;
	while ((idx < trajectory.size()-1) && (trajectory[idx].time + trajectory[idx].duration< time)) {
		idx++;
	}
	if ((trajectory.size() > 0) && (trajectory[idx].time <= time)) {
		TrajectoryNode result;
		TrajectoryNode startNode= trajectory[idx];
		if (select)
			currentTrajectoryNode = idx;
		if (idx < trajectory.size()-1) {
			BezierCurve& bezier = interpolation[idx];
			SpeedProfile& profile= speedProfile[idx];
			float t = ((float)time-startNode.time) / ((float)startNode.duration);

			// adapt time ratio with speed profile
			t = profile.get(SpeedProfile::TRAPEZOIDAL, t);

			// now get position within bezier curve
			result = bezier.getCurrent(t);
		} else {
			result = trajectory[trajectory.size()-1];
		}
		return result;
	}
	return TrajectoryNode();
}

TrajectoryNode Trajectory::getCurvePoint(int time) {
    int idx = time / UITrajectorySampleRate;
    if (idx < (int)compiledCurve.size())
        return compiledCurve[idx];
    return compiledCurve[compiledCurve.size()-1]; // stay on last node if time > duration of trajectory
}

bool  Trajectory::isCurveAvailable(int time) {
    int idx = time / UITrajectorySampleRate;
    if (idx < (int)compiledCurve.size()) {
        return (!compiledCurve[idx].isNull());
    }
    return false;
}

void Trajectory::setCurvePoint(int time, const TrajectoryNode& node) {
    int idx = time / UITrajectorySampleRate;
    compiledCurve.resize(idx+1);
    compiledCurve.at(idx) = node;
}

void Trajectory::clearCurve() {
    compiledCurve.clear();
}

milliseconds Trajectory::getDuration() {
	int sum_ms = 0;
	for (unsigned i = 0;i<trajectory.size()-1;i++) {
		sum_ms += trajectory[i].duration;
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
	str.precision(floatPrecisionDigits);

	/*

	for (unsigned i = 0;i<t.trajectory.size();i++) {
		TrajectoryNode node = t.trajectory[i];
		str << fixed << "id=" << i << endl;
		str << "name=" << node.name << endl;
		str << "averagespeeddef=" << node.averageSpeedDef << endl;
		str << "durationdef=" << node.durationDef << endl;
		str << "interpolation=" << (int)node.interpolationTypeDef<< endl;

		str << "position.x=" << node.pose.position.x << endl;
		str << "position.y=" << node.pose.position.y << endl;
		str << "position.z=" << node.pose.position.z << endl;
		str << "orientation.x=" << node.pose.orientation.x << endl;
		str << "orientation.y=" << node.pose.orientation.y << endl;
		str << "orientation.z=" << node.pose.orientation.z << endl;
		str << "gripper=" << node.pose.gripperAngle << endl;
		str << "angles.0=" << node.pose.angles[0] << endl;
		str << "angles.1=" << node.pose.angles[1] << endl;
		str << "angles.2=" << node.pose.angles[2] << endl;
		str << "angles.3=" << node.pose.angles[3] << endl;
		str << "angles.4=" << node.pose.angles[4] << endl;
		str << "angles.5=" << node.pose.angles[5] << endl;
		str << "angles.6=" << node.pose.angles[6] << endl;

	}*/

	int indent = 0;
	str << t.toString(indent);

	return str.str();
}

Trajectory  Trajectory::unmarshal(string str) {
	stringstream f(str);
	TrajectoryNode node;
	Trajectory result;
	string line;
	string s;
	while (getline(f, line)) {
		s += line;
	}
	int idx = 0;
	result.fromString(s, idx);
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

	compile();
}


string Trajectory::toString(int &indent) const {

	stringstream str;
	str.precision(3);
	str << listStartToString("trajectory",indent);
	str << endofline(indent);
	for (unsigned i = 0;i< trajectory.size();i++) {
		str << trajectory[i].toString(indent);
	}
	str << listEndToString(indent);

	return str.str();
}

bool Trajectory::fromString(const string& str, int &idx) {
	bool ok = listStartFromString("trajectory", str, idx);

	trajectory.clear();
	do {
    	TrajectoryNode node;
    	string s = str.substr(idx);
        ok = node.fromString(str,idx);
        if (ok)
        	trajectory.insert(trajectory.end(),node);
    } while ((ok) && (idx < (int)str.size()-1));
	ok = listEndFromString(str, idx);

    return ok;
}


