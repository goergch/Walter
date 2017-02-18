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
    	move(frompeg, topeg);
        LOG(DEBUG) << "Move disk 1 from peg "<<frompeg<<" to peg "<<topeg;
        return;
    }

	towers(NumberOfDisks - 1, frompeg, auxpeg, topeg);

	move(frompeg, topeg);
	LOG(DEBUG) << "Move disk "<< NumberOfDisks <<" from peg "<<frompeg<<" to peg "<<topeg;

    towers(NumberOfDisks - 1, auxpeg, topeg, frompeg);
}

HanoiMoves::HanoiMoves() {

		// dimensions of game
		diskHeight = 10;
		pegDistance = 70;
		gameBaseHeight = 30;
		diameterDifference = 10;
		smallestDiskDiameter = 30;

		// trajectory params
		liftHeight = gameBaseHeight+50;
		grippingDuration = 300;
		grippingDurationBreak = 800;
		gripperAddonToDisk = 20;

		pegsBase[0] = Point(250, -pegDistance,gameBaseHeight);
		pegsBase[1] = Point(250, 0,gameBaseHeight);
		pegsBase[2] = Point(250, pegDistance,gameBaseHeight);
};

void HanoiMoves::init(int numberOfDisks) {
		numberOfDisksOnPeg[0] = numberOfDisks;
		numberOfDisksOnPeg[1] = 0;
		numberOfDisksOnPeg[2] = 0;

		// empty the array that stores the disks per peg
		for (int i = 0;i<MaxDisks; i++) {
			diskNumbersPerPeg[0][i] = 0;
			diskNumbersPerPeg[1][i] = 0;
			diskNumbersPerPeg[2][i] = 0;
		}

		// on the left (0) peg we start with all the disks. Smallest Disk has the number 1
		for (int i = numberOfDisks; i> 0;i--)
			diskNumbersPerPeg[0][numberOfDisks-i] = i;
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
void HanoiMoves::move(int fromPegNumber, int toPegNumber) {
		Pose pose;

		int fromPegDisks = numberOfDisksOnPeg[fromPegNumber];
		int toPegDisks = numberOfDisksOnPeg[toPegNumber];

		// which disk is to be moved?
		int diskNumber = diskNumbersPerPeg[fromPegNumber][fromPegDisks-1];
		int diskDiameter = smallestDiskDiameter + (diskNumber-1)* diameterDifference;

		// move above the disk
		pose.position= pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperDistance = diskDiameter + gripperAddonToDisk;
		addPose(pose);

		// go down
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += diskHeight*fromPegDisks;
		addPose(pose,JOINT_LINEAR, grippingDuration);

		// grab the disk
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += diskHeight*fromPegDisks;
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperDistance = diskDiameter;
		addPose(pose, JOINT_LINEAR,grippingDurationBreak);
		addPose(pose);

		// move up
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		addPose(pose, POSE_LINEAR);

		// go to the other peg
		pose.position = pegsBase[toPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		addPose(pose);

		// go down
		pose.position = pegsBase[toPegNumber];
		pose.position.z += diskHeight*(toPegDisks+1);
		pose.orientation = Rotation(0,radians(90),0);
		addPose(pose, JOINT_LINEAR, grippingDuration);

		// open gripper and let disk there
		pose.position = pegsBase[toPegNumber];
		pose.position.z += diskHeight*(toPegDisks+1);
		pose.orientation = Rotation(0,radians(90),0);
		pose.gripperDistance = diskDiameter + gripperAddonToDisk;
		addPose(pose, JOINT_LINEAR, grippingDurationBreak);
		addPose(pose);

		// go up
		pose.position = pegsBase[toPegNumber];
		pose.position.z += liftHeight;
		pose.orientation = Rotation(0,radians(90),0);
		addPose(pose);

		// add disk to the other peg
		diskNumbersPerPeg[toPegNumber][numberOfDisksOnPeg[toPegNumber]] = diskNumber;
   		numberOfDisksOnPeg[toPegNumber]++;

   		// remove disk from first peg
		numberOfDisksOnPeg[fromPegNumber]--;
		diskNumbersPerPeg[fromPegNumber][numberOfDisksOnPeg[fromPegNumber]] = 0;
};

HanoiMoves hanoi;
