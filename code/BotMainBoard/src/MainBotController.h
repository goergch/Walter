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
	static MainBotController& getInstance() {
			static MainBotController instance;
			return instance;
	}

	void setup();
	void loop();


	const Pose& getCurrentTCP() { return currTCP; };
	const JointAngleType& getCurrentAngles() { return currJointAngles; };
	const KinematicConfigurationType& getCurrentConfiguration() { return currConfiguration; };
	const std::vector<KinematicConfigurationType>& getValidConfiguration() { return validConfigurations;}

	// internally public
	void setPose(const Pose& pTcp) { currTCP = pTcp; };
	void setAngles(const JointAngleType& pAngles) { currJointAngles = pAngles; };
	void setConfiguration(const KinematicConfigurationType& pConfig) { currConfiguration = pConfig; }

private:
	void computeAngles(const Pose& tcp, const JointAngleType& currAngles, KinematicsSolutionType& angles);
	void computePose(const JointAngleType& currAngles, Pose& tcp);


	JointAngleType  currJointAngles;
	KinematicConfigurationType currConfiguration;
	Pose currTCP;
	std::vector<KinematicConfigurationType> validConfigurations;
};


#endif /* BOTCONTROLLER_H_ */
