/*
 * LightsController.cpp
 *
 *  Created on: 07.01.2017
 *      Author: SuperJochenAlt
 */

#include <LightsController.h>
#include "sn3218.h"
#include "pins.h"
#include "TimePassedBy.h"
#include "config.h"

TimePassedBy 	lightsTimer;

int 			heartBeatTimer = 0;
int 			brokenLightsCounter = 0;
int 			poseSampleCounter  = 0;

bool enableMode  = false;
bool powerMode = false;
bool powerControlMode = false;
bool powerAmokMode = false;
bool startup = true;
int actuatorCounter = 0;

struct ActuatorValueData {
	float value;
	float min;
	float max;
	int led;
};

ActuatorValueData  actuatorValue[MAX_ACTUATORS];

LightsController lights;
LightsController::LightsController() {
}

void LightsController::setup() {
	for (int i = 0;i<MAX_ACTUATORS;i++) {
		actuatorValue[i].value = 0;
		actuatorValue[i].min = 0;
		actuatorValue[i].max= 0;
	}
	sn3218.begin(Wires[1]);
	sn3218.enable_leds(SN3218_CH_ALL); // Enable all channels
	for (int i = 0;i<18;i++) {
		sn3218.set(i, 0); // Set channel 0 to 50/255
	}
	sn3218.update();

	for (int counter = 0;counter < 30;counter++) {
		for (int i = 0;i<16;i++) {
			sn3218.set(i, 120-counter*4); // Set channel 0 to 50/255
		}
		sn3218.update();
	}

	actuatorValue[WRIST].led = LED_WRIST;
	actuatorValue[HIP].led = LED_HIP;
	actuatorValue[FOREARM].led = LED_FOREARM;
	actuatorValue[ELLBOW].led = LED_ELBOW;
	actuatorValue[HAND].led = LED_HAND;
	actuatorValue[WRIST].led = LED_WRIST;
	actuatorValue[GRIPPER].led = LED_FINGER;
}

void LightsController::set(uint8_t channel, int value) {
	value = constrain(value, 0,255);
	sn3218.set(channel, value); // Set channel 0 to 50/255
}

void LightsController::heartbeat() {
	heartBeatTimer += 2;
	if (heartBeatTimer >= 150) {
		heartBeatTimer = 0;
	} else
		set(LED_HEARTBEAT, 100-heartBeatTimer);
}

void LightsController::brokenLight() {
	brokenLightsCounter -= 2;
	if (brokenLightsCounter < 0) {
		brokenLightsCounter = ((int)random(300));
	} else {
		if (brokenLightsCounter > 200)
			set(LED_BROKEN_LIGHT, 100);
		else
			if (brokenLightsCounter < 100)
				set(LED_BROKEN_LIGHT, 0);
			else
				set(LED_BROKEN_LIGHT, (brokenLightsCounter-100)*6-500);
	}
}

void LightsController::setEnableMode (bool ok) {
	set(LED_ENABLED, 100*(int)ok);
}
void LightsController::setPowerMode (bool ok) {
	set(LED_POWER_ON, 100*(int)ok);
}
void LightsController::setSetupMode (bool ok) {
	set(LED_SETUP, 100*(int)ok);
}
void LightsController::setManualKnobMode (bool ok) {
	set(LED_CONTROL_MODE, 100*(int)ok);
}
void LightsController::setAmokMode (bool ok) {
	set(LED_AMOK_MODE, 100*(int)ok);
}

void LightsController::setTrajectoryMode (bool ok) {
	set(LED_TRAJECTORY_MODE, 100*(int)ok);
}

void LightsController::setPoseSample() {
	poseSampleCounter = 100;
}

void LightsController::setActuator(int no, float angle) {
	actuatorValue[no].value = angle;
}

void LightsController::setActuatorMinMax(int no, float min, float max) {
	actuatorValue[no].min = min;
	actuatorValue[no].min = max;
}

void LightsController::posesample() {
	poseSampleCounter -= 5;
	if (poseSampleCounter < 0)
		poseSampleCounter = 0;
	set(LED_POSE_SAMPLE, poseSampleCounter);
}

void LightsController::actuator() {
	actuatorCounter = (actuatorCounter + 1) % MAX_ACTUATORS;

	// compute value [0..1] representing the angle in its min/max range
	float ratio =
			(actuatorValue[actuatorCounter].value - actuatorValue[actuatorCounter].min) /
			(actuatorValue[actuatorCounter].max   - actuatorValue[actuatorCounter].min);
	set(actuatorValue[actuatorCounter].led, 20.0 + ratio*100.0);
}

void LightsController::loop(uint32_t now) {
	if (lightsTimer.isDue_ms(LED_UPDATE_RATE, now)) {
		if (startup) {
			for (int i = 0;i<16;i++) {
				sn3218.set(i,0);
			}
			sn3218.update();
			startup = false;
		}
		heartbeat();
		brokenLight();

		// show actuators
		actuator();

		sn3218.update();
	}
}
