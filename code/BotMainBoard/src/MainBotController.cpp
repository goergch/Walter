/*
 * BotController.cpp
 *
 *  Created on: 06.08.2016
 *      Author: JochenAlt
 */

#include <MainBotController.h>
#include "Util.h"
#include "ui/BotWindowCtrl.h"

MainBotController mainBotController;

MainBotController::MainBotController() {

}

void MainBotController::setup() {
}

void MainBotController::loop() {
	delay(10);
}

void MainBotController::computeAngles(const Pose& tcp, const JointAngleType& currAngles, KinematicsSolutionType& angles) {
}

void MainBotController::computePose(const JointAngleType& currAngles, Pose& tcp) {

}
