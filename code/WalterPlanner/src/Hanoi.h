/*
 * Hanoi.h
 *
 *  Author: JochenAlt
 */

#ifndef HANOI_H_
#define HANOI_H_

#include "spatial.h"

class Hanoi {
public:
	Hanoi();
	// starts the hanoi algorithm and calls move until all disks are moved from left peg to right peg
	// initialize, all disks are on the left peg.
	void solve(int numberOfDisks);

	// Is called by hanoi-algorithm and needs to be redefined
	// Represents one move of disk <diskNumber> from one peg to the other.
	// Pegs are 0,2,1, disks are number beginning from 1, disks are moved from peg 0 to peg 1 (via peg 2)
	virtual void move(int fromPegNumber, int toPegNumber)  = 0;
	virtual void init(int noOfDisks) = 0;

private:
	void towers(int num, int frompeg, int topeg, int auxpeg);
};

class HanoiMoves : public Hanoi {
public:
	HanoiMoves();

	virtual void init(int numberOfDisks);
	virtual void move(int fromPegNumber, int toPegNumber);

	void addPose(Pose &pose, InterpolationType interpolationType = POSE_LINEAR, rational duration = 0.0);

	// dimensions of towers of hanoi
	int gameBaseHeight;   			// height base of the games
	int diskHeight;					// height of one disk
	int pegDistance;				// distance of one peg to the next one
	int smallestDiskDiameter;		// diameter of smallest disk
	int diameterDifference;			// difference in diameter of a disk to its successor


	// trajectory
	int grippingDuration;
	int grippingDurationBreak;
	int liftHeight;
	int gripperAddonToDisk;

	// temp. dimensions
	int numberOfDisksOnPeg[3];
#define MaxDisks 10
	int diskNumbersPerPeg[3][MaxDisks];

	Point pegsBase[3];

};

extern HanoiMoves hanoi;

#endif /* HANOI_H_ */
