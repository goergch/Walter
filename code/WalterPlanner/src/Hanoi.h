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
	virtual void move(int diskNumber, int fromPegNumber, int toPegNumber)  = 0;
	virtual void init(int noOfDisks) = 0;

private:
	void towers(int num, int frompeg, int topeg, int auxpeg);
};

class HanoiMoves : public Hanoi {
public:
	HanoiMoves();

	virtual void init(int numberOfDisks);
	virtual void move(int diskNumber, int fromPegNumber, int toPegNumber);

	void addPose(Pose &pose, InterpolationType interpolationType = POSE_LINEAR, rational duration = 0.0);

	Point pegsBase[3];

	int pegHeight;
	int heightPerDisk;
	int pegDistance;
	int liftHeight;
	int numberOfDisksOnPeg[3];
	int grippingDuration;
	int grippingDurationBreak;

};

extern HanoiMoves hanoi;

#endif /* HANOI_H_ */
