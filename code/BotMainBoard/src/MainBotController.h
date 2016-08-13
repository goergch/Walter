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


	const Pose& getCurrentPose() { return currPose; };
	const JointAngleType& getCurrentAngles() { return currJointAngles; };
	const KinematicConfigurationType& getCurrentConfiguration() { return currConfiguration; };
	const std::vector<KinematicsSolutionType>& getPossibleSolutions() { return possibleSolutions;}

	bool setPose(const Pose& pPose);

	// internally public
	void setPoseImpl(const Pose& pTcp) { currPose = pTcp; };
	void setAnglesImpl(const JointAngleType& pAngles) { currJointAngles = pAngles; };
	void setConfigurationImpl(const KinematicConfigurationType& pConfig) { currConfiguration = pConfig; };
	void setPossibleSolutionsImpl(const std::vector<KinematicsSolutionType>& pConfig) { possibleSolutions = pConfig; };

private:
	JointAngleType  currJointAngles;
	KinematicConfigurationType currConfiguration;
	Pose currPose;
	std::vector<KinematicsSolutionType> possibleSolutions;
};


#endif /* BOTCONTROLLER_H_ */
