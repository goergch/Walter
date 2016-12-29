/*
 * TrajectoryExecController.cpp
 *
 *  Created on: 29.08.2016
 *      Author: JochenAlt
 */

#include "TrajectoryExecution.h"
#include "CortexController.h"
#include "logger.h"

TrajectoryExecution::TrajectoryExecution() {
	lastLoopInvocation = 0;
}

TrajectoryExecution& TrajectoryExecution::getInstance() {
	static TrajectoryExecution instance;
	return instance;
}


bool TrajectoryExecution::setup() {
	bool ok = CortexController::getInstance().setupCommunication();
	TrajectoryPlayer::setup();

	return ok;
}

// send a direct command to uC
void TrajectoryExecution::directAccess(string cmd, string& response, bool &okOrNOk) {
	CortexController::getInstance().directAccess(cmd, response, okOrNOk);
}

void TrajectoryExecution::loguCToConsole() {
	CortexController::getInstance().loguCToConsole();
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


bool TrajectoryExecution::isBotUpAndReady() {
	return botIsUpAndRunning;
}

bool TrajectoryExecution::setAnglesAsString(string anglesAsString) {
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

	// move the bot to the passed position within the next TrajectorySampleRate ms.
	if (now>=lastLoopInvocation+BotTrajectorySampleRate) {
		// take care that we call the uC with BotTrajectorySampleRate, so add
		// BotTrajectorySampleRate not to now, but to lastInvocation (otherwise timeing errors would sum up)
		if (lastLoopInvocation<now-BotTrajectorySampleRate)
			lastLoopInvocation = now;
		else
			lastLoopInvocation += BotTrajectorySampleRate;

		if (CortexController::getInstance().communicationOk()){
			int duration = BotTrajectorySampleRate*110/100; // add 10% in case of timing issues
			bool ok = CortexController::getInstance().move(pPose.angles, duration);
			heartbeatSend = ok;
		} else
			heartbeatSend = false; // no heartbeat when communication is down
	}
}

// return true if a heart beat has been sent. Works only once, if a heart beat has been given,
// this returns false until the next uC call happened
bool TrajectoryExecution::heartBeatSendOp() {
	bool result;
	if (heartbeatSend) {
		heartbeatSend = false;
		result = true;
	} else
		result = false;
	return result;
}


bool  TrajectoryExecution::startupBot() {

	LOG(INFO) << "initiating startup procedure";

	// if the bot is in zombie state, disable it properly
	bool enabled, setuped, powered;
	bool ok = CortexController::getInstance().info(powered, setuped, enabled);
	if (ok && !powered && enabled) {
		botIsUpAndRunning = false;
		CortexController::getInstance().disableBot();
		CortexController::getInstance().info(powered, setuped, enabled);
		if (enabled)
			LOG(ERROR) << "startupBot: disable did not work";
	}

	botIsUpAndRunning = false;

	// initialize all actuator controller, idempotent. Enables reading angle sensors
	ok = CortexController::getInstance().setupBot();
	if (!ok) {
		LOG(ERROR) << "startupBot: setup did not work";
		return false;
	}

	// read all angles and check if ok
	ActuatorStateType initialActuatorState[NumberOfActuators];
	ok = CortexController::getInstance().getAngles(initialActuatorState);
	if (!ok) {
		LOG(ERROR) << "startupBot: getAngles did not work";
		return false;
	}

	// power up if necessary
	if (ok && !powered)
		ok = CortexController::getInstance().power(true);
	if (!ok) {
		LOG(ERROR) << "startupBot: powerUp did not work";
		return false;
	}

	ok = CortexController::getInstance().enableBot();	// enable every actuator (now reacting to commands)
	if (!ok) {
		ok = CortexController::getInstance().power(false);
		LOG(ERROR) << "startupBot: enable did not work";
		return false;
	}

	// move to default position, but compute necessary time required with slow movement
	rational maxAngleDiff = 0;
	for (int i = 0;i<NumberOfActuators;i++) {
		rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-initialActuatorState[i].currentAngle);
		maxAngleDiff = max(maxAngleDiff, angleDiff);
	}
	rational speed_deg_per_s = 20; // degrees per second
	rational duration_ms = 0; // duration for movement

	// move to default position
	duration_ms = degrees(maxAngleDiff)/speed_deg_per_s*1000;
	ok = CortexController::getInstance().move(JointAngles::getDefaultPosition(), duration_ms);
	if (!ok) {
		ok = CortexController::getInstance().power(false);
		LOG(ERROR) << "startupBot: move to default position did not work";
		return false;
	}

	// wait until we are there
	delay(duration_ms+200);

	// fetch current angles, now from reset position
	ActuatorStateType resetActuatorState[NumberOfActuators];
	ok = CortexController::getInstance().getAngles(resetActuatorState);
	if (!ok) {
		ok = CortexController::getInstance().power(false);
		LOG(ERROR) << "startupBot: fetching reset position failed";
		return false;
	}

	// check that we really are in default position
	for (int i = 0;i<NumberOfActuators;i++) {
		rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-resetActuatorState[i].currentAngle);
		maxAngleDiff = min(maxAngleDiff, angleDiff);
	}
	if (maxAngleDiff > 1.0) {
		ok = CortexController::getInstance().power(false);
		LOG(ERROR) << "startupBot: checking reset position failed";
		return false;

	}

	LOG(INFO) << "startup procedure completed";

	botIsUpAndRunning = true;
	return true;
}

bool TrajectoryExecution::teardownBot() {
	LOG(INFO) << "initiating teardown procedure";

	bool enabled, setuped, powered;
	bool ok = CortexController::getInstance().info(powered, setuped, enabled);
	if (!ok) {
		botIsUpAndRunning = false;

		CortexController::getInstance().power(false); 	// delays are done internally
		LOG(ERROR) << "teardownBotBot: info failed";
		return false;
	}

	if (powered && enabled && setuped) {
		ActuatorStateType currentActuatorState[NumberOfActuators];
		ok = CortexController::getInstance().getAngles(currentActuatorState);
		if (!ok) {
			botIsUpAndRunning = false;

			CortexController::getInstance().power(false); 	// delays are done internally
			LOG(ERROR) << "teardownBotBot: getAngles failed";
			return false;
		}

		// move to default position
		rational maxAngleDiff = 0;
		for (int i = 0;i<NumberOfActuators;i++) {
			rational angleDiff = fabs(JointAngles::getDefaultPosition()[i]-currentActuatorState[i].currentAngle);
			maxAngleDiff = max(maxAngleDiff, angleDiff);
		}
		rational speed_deg_per_s = 20; // degrees per second
		rational duration_ms = degrees(maxAngleDiff) / speed_deg_per_s*1000.0; // duration for movement

		CortexController::getInstance().move(JointAngles::getDefaultPosition(), duration_ms);
		delay(duration_ms+200);
	}
	botIsUpAndRunning = false;

	/* ok = */ CortexController::getInstance().power(false);

	return true;
}

