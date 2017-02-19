/*
 * LightsController.cpp
 *
 * LightController controls the lights on the front panel by getting the controllers
 * state and setting according PWM values for the LEDs. Where switches are used (e.g.
 * power on, or enable), the call to LightsController is issued by the controller itself
 * by using functions like setPowerMode etc.
 *
 *  Author: JochenAlt
 */

#ifndef LIGHTSCONTROLLER_H_
#define LIGHTSCONTROLLER_H_

#include <Config.h>
#include "Arduino.h"
#include "TimePassedBy.h"

// ActuatorValue contains all data necessary to
// compute the brightness of the according LED.
struct ActuatorValueData {
	float currentAngle;	// [°] current angle of the applied actuator
	float min;			// [°] defines angle of darkness
	float max;			// [°] defines angle of max brightness
	int led_channel;	// LED driver's channel ID of this LED
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

	// The actuator LEDs represent the actuator's angle, so we need to set min/max
	// upfront in order to know the position of dark and max-brightness
	void setActuatorMinMax(int no, float min, float max);

private:
	void updateHeartbeat();
	void updateBrokenLight();
	void updateActuator();
	void updatePosesample();
	void set(uint8_t channel, int value);

	TimePassedBy 	lightsTimer;		// controls timer of 100ms to change lights

	int heartBeatTimer = 0;				// controls the heartbeat
	int brokenLightsCounter = 0;		// controls the broken blinking
	int brokenLightsPhaseCounter= 0;	// controls when broken blinking happens

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
