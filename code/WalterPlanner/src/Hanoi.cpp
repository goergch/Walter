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

HanoiTrajectory::HanoiTrajectory() {

		// dimensions of game all in [mm]
		diskHeight = 10;
		pegDistance = 72.5;
		gameBaseHeight = 18;
		diameterDifference = 12; // actually I leave out every second disk to have a bigger difference here
		smallestDiskDiameter = 23;

		// trajectory parameter
		liftHeight = gameBaseHeight+74;
		grippingDuration = 300;
		grippingDurationBreak = 800;
		gripperAddonToDisk = 20;
		orientationTurnDuration = 700;

		// position of middle peg
		towersNull=Point(350,0,gameBaseHeight);

		// all pegs
		pegsBase[0] = towersNull + Point(0, -pegDistance,	0);
		pegsBase[1] = towersNull + Point(0, 0,				0);
		pegsBase[2] = towersNull + Point(0, pegDistance,	0);

};

void HanoiTrajectory::init(int numberOfDisks) {
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

		Point tcp = Kinematics::getInstance().getTCPCoordinates();
		tcp.z = 24; // tip of the gripper is 25

		Kinematics::getInstance().setTCPCoordinates(tcp);

		Pose pose;
		pose.angles = Kinematics::getNullPositionAngles();
		Kinematics::getInstance().computeForwardKinematics(pose);

		addPose(pose);

}

void HanoiTrajectory::addPose(Pose &pose, InterpolationType interpolationType, rational duration) {
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
void HanoiTrajectory::move(int fromPegNumber, int toPegNumber) {
		Pose pose;

		int fromPegDisks = numberOfDisksOnPeg[fromPegNumber];
		int toPegDisks = numberOfDisksOnPeg[toPegNumber];

		// which disk is to be moved?
		int diskNumber = diskNumbersPerPeg[fromPegNumber][fromPegDisks-1];
		int diskDiameter = smallestDiskDiameter + (diskNumber-1)* diameterDifference;

		bool horizontal = (diskNumber >= 2);
		// if defined, do it horizontally

		if (horizontal !=horizontalPosition) {
			TrajectoryNode &prev = TrajectorySimulation::getInstance().getTrajectory().getSupportNodes().back();
			TrajectoryNode savePrev = prev; // copy

			// go to orientation changing position in the middle peg
			pose = savePrev.pose;
			pose.position.y = 0;
			addPose(pose, POSE_LINEAR, orientationTurnDuration);

			// now turn
			pose.position.x += 1.0;

			if (!horizontalPosition)
				pose.orientation = Rotation(0,radians(45),0);
			else
				pose.orientation = Rotation(0,radians(90),0);

			addPose(pose);
			horizontalPosition = horizontal;
		}

		// move above the disk
		pose.position= pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		if (horizontal)
			pose.orientation = Rotation(0,radians(45),0);
		else
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
		pose.gripperDistance = diskDiameter;
		addPose(pose, JOINT_LINEAR,grippingDurationBreak);
		addPose(pose);

		// move up
		pose.position = pegsBase[fromPegNumber];
		pose.position.z += liftHeight;
		addPose(pose, POSE_LINEAR);

		// go to the other peg
		pose.position = pegsBase[toPegNumber];
		pose.position.z += liftHeight;
		addPose(pose);

		// go down
		pose.position = pegsBase[toPegNumber];
		pose.position.z += diskHeight*(toPegDisks+1);
		addPose(pose, JOINT_LINEAR, grippingDuration);

		// open gripper and let disk there
		pose.position = pegsBase[toPegNumber];
		pose.position.z += diskHeight*(toPegDisks+1);
		pose.gripperDistance = diskDiameter + gripperAddonToDisk;
		addPose(pose, JOINT_LINEAR, grippingDurationBreak);
		addPose(pose);

		// go up
		pose.position = pegsBase[toPegNumber];
		pose.position.z += liftHeight;
		addPose(pose);

		// add disk to the other peg
		diskNumbersPerPeg[toPegNumber][numberOfDisksOnPeg[toPegNumber]] = diskNumber;
   		numberOfDisksOnPeg[toPegNumber]++;

   		// remove disk from first peg
		numberOfDisksOnPeg[fromPegNumber]--;
		diskNumbersPerPeg[fromPegNumber][numberOfDisksOnPeg[fromPegNumber]] = 0;
};

HanoiTrajectory hanoi;
