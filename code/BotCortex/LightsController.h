/*
 * LightsController.cpp
 *
 * LightController controls the lights on the front panel by fetching the controllers
 * state and setting according PWM values for the LEDs. Where switches are used (e.g.
 * power on, or enable), the call to LightsController is issued by the controller itself
 * by using functions like setPowerMode etc.
 *
 *  Author: JochenAlt
 */

#ifndef LIGHTSCONTROLLER_H_
#define LIGHTSCONTROLLER_H_

#include "Arduino.h"
#include "TimePassedBy.h"
#include "config.h"

struct ActuatorValueData {
	float value;
	float min;
	float max;
	int led;
};


class LightsController {
public:
	LightsController();

	// initialize 18-channel pwm driver
	void setup();

	// to be called at least once per ms.
	void loop(uint32_t now);

	// these functions are to be called when Walter's state
	// changes in order to switch the according LED.
	void setEnableMode (bool ok);
	void setPowerMode (bool ok);
	void setSetupMode (bool ok);
	void setManualKnobMode (bool ok);
	void setAmokMode (bool ok);
	void setTrajectoryMode (bool ok);

	// to be called whenever a move command has been issued by the cerebellum
	void setPoseSample();

	// The actuator LEDs represent the actuator's angle, so we need to set min/max upfront.
	void setActuatorMinMax(int no, float min, float max);

private:
	void heartbeat();
	void brokenLight();
	void actuator();
	void posesample();

	void set(uint8_t channel, int value);
	TimePassedBy 	lightsTimer;

	int heartBeatTimer = 0;
	int brokenLightsCounter = 0;
	int poseSampleCounter  = 0;
	bool enableMode  = false;
	bool powerMode = false;
	bool powerControlMode = false;
	bool powerAmokMode = false;
	bool startup = true;
	int actuatorCounter = 0;
	bool setuped = false;
	ActuatorValueData  actuatorValue[MAX_ACTUATORS];
};

extern LightsController lights;

#endif /* LIGHTSCONTROLLER_H_ */
