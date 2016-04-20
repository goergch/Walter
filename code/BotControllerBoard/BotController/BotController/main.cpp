/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: SuperJochenAlt
 */ 

#include "Arduino.h"
#include "MotorDriver.h"

MotorDriverHerkulexImpl wristMotor;

void setup() {
	wristMotor.setup(57600);
}


	
void loop() {}
	