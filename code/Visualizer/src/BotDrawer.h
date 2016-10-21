/*
 * BotDrawer.h
 *
 * Draw the bot given by stl files in openGL
 *
 *  Created on: 18.08.2016
 *      Author: JochenAlt
 */

#ifndef UI_BOTDRAWER_H_
#define UI_BOTDRAWER_H_

#include "CADObject.h"
#include "Kinematics.h"

class BotDrawer {
public:
	BotDrawer() {};
	static BotDrawer& getInstance() {
		static BotDrawer instance;
		return instance;
	}

	// display the bot with the given joint angles in the current openGL window
	void display(const JointAngles& angles, const Pose& pose, const GLfloat* color, const GLfloat* accentColor);

	// setup by looking for the STL files
	void setup();
private:
	// read the stl files per actuator in that path
	void readSTLFiles(string path);


	CADObject housing;
	CADObject shoulder;
	CADObject upperarm;
	CADObject ellbow;
	CADObject forearm;
	CADObject wrist;
	CADObject hand;
	CADObject gripper;
};

#endif /* UI_BOTDRAWER_H_ */
