/*
 * BotController.cpp
 *
 * Created: 20.04.2016 09:47:33
 *  Author: SuperJochenAlt
 */ 

#include "Arduino.h"
#include "setup.h"
#include "MotorDriver.h"
#include "Motors.h"

#include "BotMemory.h"
#include "setup.h"
#include <HkxPosControl.h>
#include <avr/wdt.h>

Motors motors;
extern BotMemory botMemory;

void setup() {
	// being stuck after 4s let the watchdog catch it
	wdt_enable(WDTO_4S);

	pinMode(LED,OUTPUT);
	digitalWrite(LED, HIGH);
	delay(50);
	digitalWrite(LED, LOW);
	delay(100);
	digitalWrite(LED, HIGH);
	delay(50);
	digitalWrite(LED, LOW);
	
	// in case anything during setup goes wrong, start with UART
	Serial.begin(CONNECTION_BAUD_RATE);
	Serial.println(F("Snorre"));
	delay(1000);
		
	// initialize eeprom configuration values
	memory.setup();
	
	// initialize wrist motor with herkulex servo
	motors.setup();
	
	/*
	
	 HkxPrint printout = HkxPrint();  // No printout with Arduino UNO
	 HkxCommunication communication = HkxCommunication(HKX_115200, Serial1, printout);  // Communication with the servo on Serial1
	 HkxPosControl servo(253, communication, printout);  // control position for the servo ID=253 (factory default value)

	 delay(100); // wait 10 seconds
	Serial.println("set torque");
	 // servo.setTorqueLEDControl(HKX_NO_VALUE, HKX_LED_RED);  // set the LED to red
	 delay(100); // wait 10 seconds
	servo.movePosition(-900, 1000, HKX_LED_BLUE, false);  // set the servo to 45° in 2 seconds, led turns to blue during the move

	for (int i = -900;i<=900;i=i+20) {
		Serial.println("move");
		Serial.println(i);
		servo.movePosition(i, 20, HKX_LED_BLUE, false);  // set the servo to 45° in 2 seconds, led turns to blue during the move
		wdt_reset();
		delay(10);

	 uint16_t inputVoltage;
	 int16_t position;
	 // get the current behaviour of the servo
	 servo.getBehaviour(&inputVoltage, HKX_NO_VALUE, &position, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE, HKX_NO_VALUE);
	 // do whatever you need with inputVoltage and position variables
	 Serial.print("position");
	 Serial.print(position);

	 Serial.print("voltage");
	 Serial.print(inputVoltage);
	 Serial.println();

	}
	 

	  /*
	
	herkulexServo.beginSerial1(HERKULEX_SERVO_BAUD_RATE);		// baud rate is detected automatically by herkulex servo
	herkulexServo.reboot(1);	//reboot our motor
	delay(500);
	herkulexServo.initialize(); //initialize motors
	delay(200);
	Serial.print("stat(1)=");
	Serial.println(herkulexServo.stat(1));
	Serial.print("stat(BROADCAST)=");
	Serial.println(herkulexServo.stat(HERKULEX_BROADCAST_ID));
*/	
}


	
void loop() {
	wdt_reset();

	memory.loop(); // check if config values have to be stored in EEprom

	motors.loop();	
	
}
	