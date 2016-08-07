/*
 * BotController.cpp
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#include <MainBotController.h>
#include "Util.h"

MainBotController mainBotController;

MainBotController::MainBotController() {

}

void MainBotController::setup() {

}

void MainBotController::loop() {
	delay(10);
}

void MainBotController::computeAngles(const Pose& tcp, const JointAngleType& currAngles, JointAngleType& angles) {
}

void MainBotController::computePose(const JointAngleType& currAngles, Pose& tcp) {

}
