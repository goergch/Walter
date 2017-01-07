/*
 * LightsController.h
 *
 *  Created on: 07.01.2017
 *      Author: SuperJochenAlt
 */

#ifndef LIGHTSCONTROLLER_H_
#define LIGHTSCONTROLLER_H_

#include "Arduino.h"

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
};

extern LightsController lights;

#endif /* LIGHTSCONTROLLER_H_ */
