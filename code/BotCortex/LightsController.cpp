#include <LightsController.h>
#include "sn3218.h"
#include "pins.h"
#include "TimePassedBy.h"
#include "config.h"
#include "Controller.h"
#include "I2CPortScanner.h"


LightsController lights;
LightsController::LightsController() {
}

void LightsController::setup() {
	logger->println("--- light controller setup start ---");
	setuped = true;
	for (int i = 0;i<MAX_ACTUATORS;i++) {
		actuatorValue[i].currentAngle = 0;
		actuatorValue[i].min = 0;
		actuatorValue[i].max= 0;
	}

	// initalize the SN3218 LED controller with I2C
	sn3218.begin(Wires[1]);

	// check that lights board is connected and reacted on the i2c address
	byte error = 0;
	bool ok = scanI2CAddress(Wires[1], SN3218_ADDR, error);

	if (!ok) {
		logger->print("LED driver on I2C address 0x");
		logger->print(SN3218_ADDR, HEX);
		logger->println(" not detected");
		setuped = false;
		return;
	}

	sn3218.enable_leds(SN3218_CH_ALL); // Enable all channels

	for (int i = 0;i<16;i++) {
		sn3218.set(i, 255); // Set channel 0 to 50/255
		sn3218.update();
		delay(30);
	}

	for (int counter = 0;counter < 50;counter++) {
		for (int i = 0;i<16;i++) {
			sn3218.set(i, (5*49-counter*5)); // Set channel 0 to 50/255
		}
		sn3218.update();
	}

	actuatorValue[HIP].led_channel = LED_HIP;
	actuatorValue[UPPERARM].led_channel = LED_UPPERARM;
	actuatorValue[FOREARM].led_channel = LED_FOREARM;
	actuatorValue[ELLBOW].led_channel = LED_ELBOW;
	actuatorValue[WRIST].led_channel = LED_WRIST;
	actuatorValue[HAND].led_channel = LED_HAND;
	actuatorValue[GRIPPER].led_channel = LED_FINGER;
	logger->println("--- light controller setup end ---");
}

void LightsController::set(uint8_t channel, int value) {
	uint8_t pwmValue = constrain(value, 0,255);
	if (setuped)
		sn3218.set(channel, pwmValue);
}

void LightsController::updateHeartbeat() {
	heartBeatTimer++;
	if (heartBeatTimer >= 100) {
		heartBeatTimer = 0;
	} else {
		if (heartBeatTimer < 15)
			set(LED_HEARTBEAT, 60-heartBeatTimer*10);
		else
			set(LED_HEARTBEAT, 60-(heartBeatTimer-15)*3);
	}
}

void LightsController::updateBrokenLight() {
	const int interval=33;
	brokenLightsCounter--;
	if ((brokenLightsCounter < 0) || (random(3) == 0))
	{
		brokenLightsCounter = ((int)random(interval*3));
		brokenLightsPhaseCounter--;
		if (brokenLightsPhaseCounter < 0)
			brokenLightsPhaseCounter = 100;
	} else {
		if (brokenLightsPhaseCounter < 30) {
			if (brokenLightsCounter > interval*2)
				set(LED_BROKEN_LIGHT, 35);
			else
				if (brokenLightsCounter < interval) {
					set(LED_BROKEN_LIGHT, 0);
				}
				else
					set(LED_BROKEN_LIGHT, (brokenLightsCounter-interval)*20-(interval*20-200));
		}
	}
}

void LightsController::setEnableMode (bool ok) {
	set(LED_ENABLED, 100*(int)ok);
	enableMode = ok;
	if (setuped)
		sn3218.update(); // update immediately, since this can happen in setup phase when loop() doe snot update
}
void LightsController::setPowerMode (bool ok) {
	set(LED_POWER_ON, 100*(int)ok);
	powerMode = ok;
	if (!powerMode) {
		enableMode = false;
	}
	if (setuped)
		sn3218.update(); // update immediately, since this can happen in setup phase when loop() doe snot update
}
void LightsController::setSetupMode (bool ok) {
	set(LED_SETUP, ok?100:0);
	if (setuped)
		sn3218.update(); // update immediately, since this can happen in setup phase when loop() doe snot update
}

void LightsController::setManualKnobMode (bool ok) {
	set(LED_CONTROL_MODE, 100*(int)ok);
	powerControlMode = ok;
}

void LightsController::setAmokMode (bool ok) {
	set(LED_AMOK_MODE, 100*(int)ok);
	powerAmokMode = ok;
}

void LightsController::setTrajectoryMode (bool ok) {
	set(LED_TRAJECTORY_MODE, 100*(int)ok);
}

void LightsController::setPoseSample() {
	poseSampleCounter = 100;
}

void LightsController::setActuatorMinMax(int no, float min, float max) {
	actuatorValue[no].min = min;
	actuatorValue[no].max = max;
}

void LightsController::updatePosesample() {
	poseSampleCounter -= 5;
	if (poseSampleCounter < 0)
		poseSampleCounter = 0;
	set(LED_POSE_SAMPLE, poseSampleCounter);
}

void LightsController::updateActuator() {
	// iterate through all the actors (one per call) to reduce traffic on I2C line
	actuatorCounter = (actuatorCounter + 1) % MAX_ACTUATORS;

	// compute value [0..1] representing the angle in its min/max range
	ActuatorValueData& data = actuatorValue[actuatorCounter];
	data.currentAngle = controller.getActuator(actuatorCounter)->getCurrentAngle();

	float ratio =
			(data.currentAngle - data.min) /
			(data.max   - data.min);
	uint8_t pwmValue = 0.0 + ratio*150.0;
	if (enableMode)
		set(data.led_channel,pwmValue);
	else
		set(data.led_channel,0);
}

void LightsController::loop(uint32_t now) {
	if (setuped) {
		if (lightsTimer.isDue_ms(LED_UPDATE_RATE, now)) {
			if (startup) {
				for (int i = 0;i<16;i++) {
					sn3218.set(i,0);
				}
				startup = false;
			}
			updateHeartbeat();
			updateBrokenLight();
			updateActuator();

			sn3218.update();
		}
	}
}
