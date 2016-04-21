/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: SuperJochenAlt
 */ 

#include "Arduino.h"
#include "MotorDriver.h"
#include "BotMemory.h"
MotorDriverHerkulexImpl wristMotor(0);
extern BotMemory botMemory;

void setup() {
	memory.setup();
	wristMotor.setup(57600);
}


	
void loop() {
	memory.loop(); // check if config values have to be stored in EEprom
}
	