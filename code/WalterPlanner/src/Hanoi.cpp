#include "Hanoi.h"
#include "logger.h"
#include "TrajectorySimulation.h"

#include <iostream>
using namespace std;

Hanoi::Hanoi() {
}

void Hanoi::solve(int numberOfDisks)
{
	init(numberOfDisks);
    towers(numberOfDisks, 0, 2, 1);
}

void Hanoi::towers(int NumberOfDisks, int frompeg, int topeg, int auxpeg)
{
    if (NumberOfDisks == 1)
    {
    	move(1, frompeg, topeg);
        LOG(DEBUG) << "Move disk 1 from peg "<<frompeg<<" to peg "<<topeg;
        return;
    }

	towers(NumberOfDisks - 1, frompeg, auxpeg, topeg);

	move(NumberOfDisks, frompeg, topeg);
	LOG(DEBUG) << "Move disk "<< NumberOfDisks <<" from peg "<<frompeg<<" to peg "<<topeg;

    towers(NumberOfDisks - 1, auxpeg, topeg, frompeg);
}

HanoiMoves::HanoiMoves() {
		heightPerDisk = 10;
		pegDistance = 70;
		pegHeight = 30;
		liftHeight = 100;

		pegsBase[0] = Point(250, -pegDistance,pegHeight);
		pegsBase[1] = Point(250, 0,pegHeight);
		pegsBase[2] = Point(250, pegDistance,pegHeight);
};

void HanoiMoves::init(int numberOfDisks) {
		numberOfDisksOnPeg[0] = numberOfDisks;
		numberOfDisksOnPeg[1] = 0;
		numberOfDisksOnPeg[2] = 0;
}

void HanoiMoves::addPose(Pose &pose, InterpolationType interpolationType, rational duration) {
	vector<TrajectoryNode>& trajectory = TrajectorySimulation::getInstance().getTrajectory().getSupportNodes();

	// this sets the pose, does inverse kinematics but does not return the angles
	TrajectorySimulation::getInstance().setPose(pose);
	// so fetch the angles explicitely
	pose.angles = TrajectorySimulation::getInstance().getCurrentAngles();

	TrajectoryNode node;
	node.pose = pose;
	if (duration == 0)
		node.averageSpeedDef = 0.100;
	else
		node.durationDef = duration;
	node.interpolationTypeDef = interpolationType;
	node.continouslyDef = false;

	trajectory.insert(trajectory.end(), node);
}
void HanoiMoves::move(int diskNumber, int fromPegNumber, int toPegNumber) {
		Pose pose;

		int fromPegDisks = numberOfDisksOnPeg[fromPegNumber];
		int toPegDisks = numberOfDisksOnPeg[toPegNumber];

		// move above the disk
		pose.position= pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(60);
		addPose(pose);

		// go down
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += heightPerDisk*fromPegDisks;
		// pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(60);
		addPose(pose,JOINT_LINEAR, 500);

		// grab the disk
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += heightPerDisk*fromPegDisks;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(11);
		addPose(pose);

		// move up
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(11);
		addPose(pose, POSE_CUBIC_BEZIER);

		// go to the other peg
		pose.position = pegsBase[toPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(11);
		addPose(pose);

		// go down
		pose.position = pegsBase[toPegNumber];
		pose.position.z = pegHeight + heightPerDisk*toPegDisks;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(11);
		addPose(pose, JOINT_LINEAR, 500);

		// open gripper and let disk there
		pose.position = pegsBase[toPegNumber];
		pose.position.z = pegHeight + heightPerDisk*toPegDisks;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(60);
		addPose(pose);

		// go up
		pose.position = pegsBase[toPegNumber];
		pose.position.z = pegHeight + heightPerDisk*toPegDisks+50;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperAngle = radians(60);
		addPose(pose);

		numberOfDisksOnPeg[fromPegNumber]--;
		numberOfDisksOnPeg[toPegNumber]++;

};

HanoiMoves hanoi;
