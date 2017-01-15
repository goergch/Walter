/*
 * LightsController.h
 *
 *  Created on: 07.01.2017
 *      Author: SuperJochenAlt
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
	void setup();
	void loop(uint32_t now);

	void setEnableMode (bool ok);
	void setPowerMode (bool ok);
	void setSetupMode (bool ok);
	void setManualKnobMode (bool ok);
	void setAmokMode (bool ok);
	void setTrajectoryMode (bool ok);
	void setPoseSample();

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
