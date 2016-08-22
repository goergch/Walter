/*
 * MainBotController.h
 *
 * Main controller for everything. Coordinates UI with kinematics and trajectory control.
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

	// call me upfront before doing anything
	void setup();
	// call this often. Does the trajectory computation
	void loop();

	const Pose& getCurrentPose() { return currPose; };
	const JointAngleType& getCurrentAngles() { return currJointAngles; };
	const PoseConfigurationType& getCurrentConfiguration() { return currConfiguration; };
	void selectConfiguration(const PoseConfigurationType& config ) { currConfiguration = config;};
	const std::vector<KinematicsSolutionType>& getPossibleSolutions() { return possibleSolutions;}

	// set new pose, compute kinematics and send notification to UI
	bool setPose(const Pose& pPose);

	// trajectory player
	void playTrajectory();
	void stopTrajectory();
	void resetTrajectory();

	// internally public
	void setPoseImpl(const Pose& pTcp) { currPose = pTcp; };
	void setAnglesImpl(const JointAngleType& pAngles) { currJointAngles = pAngles; };
	void setConfigurationImpl(const PoseConfigurationType& pConfig) { currConfiguration = pConfig; };
	void setPossibleSolutionsImpl(const std::vector<KinematicsSolutionType>& pConfig) { possibleSolutions = pConfig; };

private:
	JointAngleType  currJointAngles;
	PoseConfigurationType currConfiguration;
	Pose currPose;
	std::vector<KinematicsSolutionType> possibleSolutions;
	uint32_t trajectoryPlayerTime_ms;
	bool trajectoryPlayerOn;
	uint32_t trajectoryPlayerStartTime;
};



#endif /* BOTCONTROLLER_H_ */
