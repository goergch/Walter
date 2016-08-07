/*
 * BotController.h
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#ifndef MAIN_BOTCONTROLLER_H_
#define MAIN_BOTCONTROLLER_H_

#include "spatial.h"
#include "Kinematics.h"

class MainBotController {
public:
	MainBotController();
	void setup();
	void loop();

	void setPose(const Pose& tcp);
	void setJointAngles(const JointAngleType& tcp);
	void getPose(Pose& tcp);
	void getJointAngles(KinematicsSolutionType& tcp);

	void getCurrentTCP(Pose& tcp) { tcp =  currTCP; };
	void getCurrentAngles(JointAngleType& angles) { angles = currJointAngles; };

private:
	void computeAngles(const Pose& tcp, const JointAngleType& currAngles, KinematicsSolutionType& angles);
	void computePose(const JointAngleType& currAngles, Pose& tcp);

	JointAngleType  currJointAngles;
	Pose currTCP;
};

extern MainBotController mainBotController;

#endif /* BOTCONTROLLER_H_ */
