/*
 * BotDrawer.h
 *
 *  Created on: 18.08.2016
 *      Author: SuperJochenAlt
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

	void display(const JointAngleType& angles, const Pose& pose, GLfloat* color);
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
