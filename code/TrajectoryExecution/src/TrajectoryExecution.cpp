/*
 * TrajectoryExecController.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecution.h"
#include "ActuatorCtrlInterface.h"

TrajectoryExecution::TrajectoryExecution() {
	lastLoopInvocation = 0;
}

TrajectoryExecution& TrajectoryExecution::getInstance() {
	static TrajectoryExecution instance;
	return instance;
}


bool TrajectoryExecution::setup() {
	bool ok = ActuatorCtrlInterface::getInstance().setupCommunication();
	if (!ok)
    	LOG(ERROR) << "uC not present";

	TrajectoryPlayer::setup();

	return ok;
}

// send a direct command to uC
void TrajectoryExecution::directAccess(string cmd, string& response, bool &okOrNOk) {
	ActuatorCtrlInterface::getInstance().directAccess(cmd, response, okOrNOk);
}

void TrajectoryExecution::loguCToConsole() {
	ActuatorCtrlInterface::getInstance().loguCToConsole();
}

string TrajectoryExecution::currentTrajectoryNodeToString() {
	TrajectoryNode node = getCurrentTrajectoryNode();
	return node.toString();
}

void TrajectoryExecution::runTrajectory(const string& trajectoryStr) {
	Trajectory& traj = getTrajectory();
	int idx = 0;
	bool ok = traj.fromString(trajectoryStr, idx);
	if (!ok)
		LOG(ERROR) << "parse error trajectory";
	traj.compile();

	playTrajectory();
}

void TrajectoryExecution::setPose(const string& poseStr) {
	Pose pose;
	int idx = 0;
	bool ok = pose.fromString(poseStr, idx);
	if (!ok)
		LOG(ERROR) << "parse error trajectory";
	TrajectoryPlayer::setPose(pose);
}

string TrajectoryExecution::isBotSetup() {
	return boolToString("upandrunning",botIsUpAndRunning);
}


string TrajectoryExecution::setAnglesAsString(string anglesAsString) {
	JointAngles angles;
	int idx = 0;
	bool ok = angles.fromString(anglesAsString, idx);
	if (!ok)
		LOG(ERROR) << "parse error angles";
	TrajectoryPlayer::setAngles(angles); // this will call notify new Pose, which sends the pose to the bot
	return heartBeatSendOp(); // true, if a pose has been sent
}

void TrajectoryExecution::loop() {
	// take current time, compute IK and store pose and angles every TrajectorySampleRate.
	// When a new pose is computed, notifyNewPose is called
	TrajectoryPlayer::loop();
}


// is called by TrajectoryPlayer whenever a new pose is set
void TrajectoryExecution::notifyNewPose(const Pose& pPose) {
	// ensure that we are not called more often then TrajectorySampleRate
	uint32_t now = millis();
	if ((lastLoopInvocation>0) && (now<lastLoopInvocation+UITrajectorySampleRate)) {
		// we are called too early, wait
		LOG(ERROR) << "TrajectoryExecution:notifyNewPose called too early: now=" << now << " lastcall=" << lastLoopInvocation;
		delay(max(lastLoopInvocation+UITrajectorySampleRate-millis()-1,(uint32_t)0));
	}

	// move the bot to the passed position within the next TrajectorySampleRate ms.
	ActuatorCtrlInterface::getInstance().move(pPose.angles, UITrajectorySampleRate);
	heartbeatSend = true;
}

string  TrajectoryExecution::heartBeatSendOp() {
	bool result;
	if (heartbeatSend) {
		heartbeatSend = false;
		result = true;
	} else
		result = false;
	return boolToString("heartbeatsend",result);
}


void  TrajectoryExecution::startupBot() {

	LOG(INFO) << "initiating startup procedure";

	// if the bot is in zombie state, disable it properly
	bool enabled, setuped, powered;
	bool ok = ActuatorCtrlInterface::getInstance().info(powered, setuped, enabled);
	if (ok && !powered && enabled) {
		botIsUpAndRunning = false;
		ActuatorCtrlInterface::getInstance().disableBot();
		ActuatorCtrlInterface::getInstance().info(powered, setuped, enabled);
		if (enabled)
			LOG(ERROR) << "startupBot: disable did not work";
	}

	botIsUpAndRunning = false;

	// initialize all actuator controller, idempotent. Enables reading angle sensors
	ok = ActuatorCtrlInterface::getInstance().setupBot();
	if (!ok) {
		LOG(ERROR) << "startupBot: setup did not work";
		return ;
	}

	// read all angles and check if ok
	ActuatorStateType initialActuatorState[NumberOfActuators];
	ok = ActuatorCtrlInterface::getInstance().getAngles(initialActuatorState);
	if (!ok) {
		LOG(ERROR) << "startupBot: getAngles did not work";
		return ;
	}

	// power up if necessary
	if (ok && !powered)
		ok = ActuatorCtrlInterface::getInstance().power(true);
	if (!ok) {
		LOG(ERROR) << "startupBot: powerUp did not work";
		return ;
	}


	ok = ActuatorCtrlInterface::getInstance().enableBot();	// enable every actuator (now reacting to commands)
	if (!ok) {
		ok = ActuatorCtrlInterface::getInstance().power(false);
		LOG(ERROR) << "startupBot: enable did not work";
		return ;
	}

	// move to default position, but compute necessary time required with slow movement
	rational maxAngleDiff = 0;
	for (int i = 0;i<NumberOfActuators;i++) {
		rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-initialActuatorState[i].currentAngle);
		maxAngleDiff = max(maxAngleDiff, angleDiff);
	}
	rational speed_deg_per_s = 10; // degrees per second
	rational duration_ms = 0; // duration for movement

	// move to default position
	duration_ms = degrees(maxAngleDiff)/speed_deg_per_s*1000;
	ok = ActuatorCtrlInterface::getInstance().move(JointAngles::getDefaultPosition(), duration_ms);
	if (!ok) {
		ok = ActuatorCtrlInterface::getInstance().power(false);
		LOG(ERROR) << "startupBot: move to default position did not work";
		return ;
	}

	// wait until we are there
	delay(duration_ms+200);

	// fetch current angles, now from reset position
	ActuatorStateType resetActuatorState[NumberOfActuators];
	ok = ActuatorCtrlInterface::getInstance().getAngles(resetActuatorState);
	if (!ok) {
		ok = ActuatorCtrlInterface::getInstance().power(false);
		LOG(ERROR) << "startupBot: fetching reset position failed";
		return ;
	}

	// check that we really are in default position
	for (int i = 0;i<NumberOfActuators;i++) {
		rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-resetActuatorState[i].currentAngle);
		maxAngleDiff = min(maxAngleDiff, angleDiff);
	}
	if (maxAngleDiff > 1.0) {
		ok = ActuatorCtrlInterface::getInstance().power(false);
		LOG(ERROR) << "startupBot: checking reset position failed";
		return;

	}

	LOG(INFO) << "startup procedure completed";

	botIsUpAndRunning = true;
	return ;
}

void TrajectoryExecution::teardownBot() {
	LOG(INFO) << "initiating teardown procedure";

	bool enabled, setuped, powered;
	bool ok = ActuatorCtrlInterface::getInstance().info(powered, setuped, enabled);
	if (!ok) {
		botIsUpAndRunning = false;

		ActuatorCtrlInterface::getInstance().power(false); 	// delays are done internally
		LOG(ERROR) << "teardownBotBot: info failed";
		return ;
	}

	if (powered && enabled && setuped) {
		ActuatorStateType currentActuatorState[NumberOfActuators];
		ok = ActuatorCtrlInterface::getInstance().getAngles(currentActuatorState);
		if (!ok) {
			botIsUpAndRunning = false;

			ActuatorCtrlInterface::getInstance().power(false); 	// delays are done internally
			LOG(ERROR) << "teardownBotBot: getAngles failed";
			return ;
		}

		// move to default position
		rational maxAngleDiff = 0;
		for (int i = 0;i<NumberOfActuators;i++) {
			rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-currentActuatorState[i].currentAngle);
			maxAngleDiff = max(maxAngleDiff, angleDiff);
		}
		rational speed_deg_per_s = 10; // degrees per second
		rational duration_ms = degrees(maxAngleDiff) / speed_deg_per_s*1000.0; // duration for movement

		ActuatorCtrlInterface::getInstance().move(JointAngles::getDefaultPosition(), duration_ms);
		delay(duration_ms+200);
	}
	botIsUpAndRunning = false;

	/* ok = */ ActuatorCtrlInterface::getInstance().power(false);
}

