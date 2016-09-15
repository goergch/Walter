/*
 * BotDrawer.h
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
	BotDrawer();
	static BotDrawer& getInstance() {
		static BotDrawer instance;
		return instance;
	}

	void display(const JointAngles& angles, const Pose& pose, const GLfloat* color, const GLfloat* accentColor);
	void setup();
	void readFiles(string path);
private:
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
