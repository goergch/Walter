/* 
* Motors.cpp
*
* Created: 22.04.2016 19:26:44
* Author: JochenAlt
*/

#include "Arduino.h"
#include "utilities.h"
#include "BotMemory.h"
#include "Controller.h"
#include <I2CPortScanner.h>
#include "GearedStepperDrive.h"
#include "RotaryEncoder.h"
#include "watchdog.h"
#include "core.h"
#include "limits.h"
#include "LightsController.h"

Controller controller;
TimePassedBy servoLoopTimer;
TimePassedBy encoderLoopTimer;
bool adjustWhat = false;

// stepperloop needs to be called as often as possible, since the stepper impulse has to happen at 5000Hz at top speed
// So, call that whereever you can, even during delay() 
void plainStepperLoop() {
	controller.stepperLoop(); // take care that nothing happens inside until everything is properly initialized
}

// yield is called in delay(), used to leverage breaks during serial communication time
void yield() {
	controller.stepperLoop();
}

Controller::Controller()
{
	currentMotor = NULL;				// current motor used for manual control
	numberOfActuators = 0;				// number of motors that have been initialized
	numberOfEncoders = 0;				// number of rotary encoders that have been initialized
	numberOfSteppers = 0;				// number of steppers that have been initialized
	setuped= false;						// flag to indicate a finished setup (used in stepperloop())
	enabled = false;					// motors are disabled until explicitly enabled
}

void Controller::enable() {
	if (isSetup()) {
		for (int i = 0;i<numberOfActuators;i++) { 
			getActuator(i)->enable();
			// give it a break to not overload power supply by switching on all steppers at the same time
		}
		enabled = true;
	}
	lights.setEnableMode(enabled);

	// wait some time before starting the loop
	delay(10);
}

void Controller::disable() {
	if (isSetup()) {
		for (int i = 0;i<numberOfActuators;i++) {
			getActuator(i)->disable();
			// give it a break to not overload power supply by switching off all steppers at the same time
			delay(5);
		}
		enabled = false;
		lights.setEnableMode(enabled);
	}
}

void Controller::selectActuator(uint8_t no) {
	// disable other motors
	if (currentMotor != NULL) {
		currentMotor->disable();
		currentMotor = NULL;
	}
	
	if ((no>=0) || (no<MAX_ACTUATORS)) {
		currentMotor = getActuator(no);
		if (currentMotor != NULL)
			currentMotor->enable();
	} 
}

Actuator* Controller::getCurrentActuator() {
	return currentMotor;
}

void Controller::logConfiguration() {
	logger->println(F("ACTUATOR SETUP"));
	for (int i = 0;i<numberOfActuators;i++) {
		// ActuatorSetupData* thisActuatorSetup = &actuatorSetup[i];
		// ActuatorIdentifier id = thisActuatorSetup->id;
		//thisActuatorSetup->print();
		for (int j = 0;j<numberOfServos;j++) {
			ServoSetupData* thisServoSetup = &servoSetup[j];
			if (thisServoSetup->id == i) {
				logger->print(F("   "));
				thisServoSetup->print();
			}			
		}
		for (int j = 0;j<numberOfSteppers;j++) {
			StepperSetupData* thisStepperSetup = &stepperSetup[j];
			if (thisStepperSetup->id == i) {
				logger->print(F("   "));
				thisStepperSetup->print();
			}
		}
		for (int j = 0;j<numberOfEncoders;j++) {
			RotaryEncoderSetupData* thisEncoderSetup = &encoderSetup[j];
			if (thisEncoderSetup->id == i) {
				logger->print(F("   "));
				thisEncoderSetup->print();
			}
		}

	}
	
	logger->println(F("ACTUATOR CONFIG"));
	memory.println();
}

bool Controller::setup() {
	switchServoPowerSupply(true);

	lights.setSetupMode(true);
	resetError();

	// reset any remains
	disable();
	// but leave power on if it is already

	if (memory.persMem.logSetup) {
		logger->println(F("--- switch on servo "));
	}

	// set the i2c lines to output to reset the sensors when starting up

	pinMode(PIN_SDA0, OUTPUT);
	digitalWrite(PIN_SDA0, LOW);
	pinMode(PIN_SCL0, OUTPUT);
	digitalWrite(PIN_SCL0, LOW);
	pinMode(PIN_SDA1, OUTPUT);
	digitalWrite(PIN_SDA1, LOW);
	pinMode(PIN_SCL1, OUTPUT);
	digitalWrite(PIN_SCL1, LOW);

	// reset number of correctly initialized devices
	numberOfSteppers = 0;
	numberOfEncoders = 0;
	numberOfServos = 0;

	if (memory.persMem.logSetup) {
		logger->println(F("--- I2C initialization"));
	}

	// initialize I2C0 and I2C1
	Wires[0]->begin();
	// timeout should be enough to repeat the sensor request within one sample
	// on I2C0 we have 4 clients (encoder of upperarm, forearm, elbow, wrist)
	Wires[0]->setDefaultTimeout(1000);
	Wires[0]->setRate(I2C_BUS_RATE);

	Wires[1]->begin();
	// on I2C0 we have 2 clients  (hip encoder, LED driver)
	Wires[1]->setDefaultTimeout(1000);
	Wires[1]->setRate(I2C_BUS_RATE);


	if (memory.persMem.logSetup) {
		logger->println(F("--- com to servo"));
	}

	delay(20); // necessary between switching on power supply on setting up serial communication
	HerkulexServoDrive::setupCommunication();

	if (memory.persMem.logSetup) {
		logger->println(F("--- initializing actuators"));
	}

	for (numberOfActuators = 0;numberOfActuators<MAX_ACTUATORS;numberOfActuators++) {
		watchdogReset(); // this takes a bit longer, kick the dog regularly

		if (memory.persMem.logSetup) {
			logger->print(F("--- setup "));
			logActuator((ActuatorIdentifier)numberOfActuators);
			logger->println(F(" ---"));
		}

		Actuator* thisActuator = &actuators[numberOfActuators];
		ActuatorConfig* thisActuatorConfig = &(memory.persMem.armConfig[numberOfActuators]);
		switch (thisActuatorConfig->actuatorType) {
		memory.persMem.armConfig[HIP].config.stepperArm.stepper.maxAngle= +90.0;
			case SERVO_TYPE: {
				if (numberOfServos >= MAX_SERVOS) {
					setError(MISCONFIG_TOO_MANY_SERVOS);
				}

				HerkulexServoDrive* servo = &servos[numberOfServos];
				servo->setup( &(memory.persMem.armConfig[numberOfActuators].config.servoArm.servo), &(servoSetup[numberOfServos]));
				thisActuator->setup(thisActuatorConfig, servo);

				lights.setActuatorMinMax(numberOfActuators, memory.persMem.armConfig[numberOfActuators].config.servoArm.servo.minAngle,memory.persMem.armConfig[numberOfActuators].config.servoArm.servo.maxAngle);

				numberOfServos++;

				if (thisActuator->hasStepper())
					setError(MISCONFIG_SERVO_WITH_STEPPER);
				if (thisActuator->hasEncoder())
					setError(MISCONFIG_SERVO_WITH_ENCODER);
				if (!thisActuator->hasServo())
					setError(MISCONFIG_SERVO);

				break;
			}

			case STEPPER_ENCODER_TYPE: {
				if (numberOfEncoders >= MAX_ENCODERS) {
					setError(MISCONFIG_TOO_MANY_ENCODERS);
				}
				if (numberOfSteppers >= MAX_STEPPERS) {
					setError(MISCONFIG_TOO_MANY_STEPPERS);
				}

				RotaryEncoder* encoder = &encoders[numberOfEncoders];
				GearedStepperDrive* stepper = &steppers[numberOfSteppers];

				encoder->setup(&actuatorConfigType[numberOfActuators], &(thisActuatorConfig->config.stepperArm.encoder), &(encoderSetup[numberOfEncoders]));
				stepper->setup(&(thisActuatorConfig->config.stepperArm.stepper), &actuatorConfigType[numberOfActuators], &(stepperSetup[numberOfSteppers]), encoder);
				thisActuator->setup(thisActuatorConfig, stepper, encoder);

				lights.setActuatorMinMax(numberOfActuators, memory.persMem.armConfig[numberOfActuators].config.stepperArm.stepper.minAngle,memory.persMem.armConfig[numberOfActuators].config.stepperArm.stepper.maxAngle);

				if (!thisActuator->hasStepper())
					setError(MISCONFIG_NO_STEPPERS);
				if (!thisActuator->hasEncoder())
					setError(MISCONFIG_NO_ENCODERS);
				if (thisActuator->hasServo())
					setError(MISCONFIG_STEPPER);
				numberOfEncoders++;
				numberOfSteppers++;
				break;
			}

			default:
				logFatal(F("unknown actuator type"));
		}
	}
	// get measurement of encoder and ensure that it is plausible 
	// (variance of a couple of samples needs to be low)
	for (int i = 0;i<numberOfEncoders;i++) {
		bool encoderCheckOk = checkEncoder(i);

		logger->print(F("enc(0x"));
		logger->print(encoders[i].i2CAddress(), HEX);
		logger->print(F(")"));
		if (!encoderCheckOk) {
			setError(ENCODER_CHECK_FAILED);
			logger->println(F(" not ok!"));
		} else {
			logger->println(F(" ok"));
		}
	}
	
	// set measured angle of the actuators and define that angle as current position by setting the movement
	for (int i = 0;i<numberOfActuators;i++) {
		Actuator* actuator = getActuator(i);
		if (actuator->hasEncoder()) {
			RotaryEncoder& encoder = actuator->getEncoder();

			// find corresponding stepper
			if (actuator->hasStepper()) {
				GearedStepperDrive& stepper= actuator->getStepper();
				if (encoder.getConfig().id != stepper.getConfig().id) {
					setError(MISCONFIG_ENCODER_STEPPER_MISMATCH);
					logActuator(stepper.getConfig().id);
					logFatal(F("encoder and stepper different"));
				} else {
					if (encoder.isOk()) {
						float angle = encoder.getAngle();
						stepper.setCurrentAngle(angle);  			// initialize current motor angle
						stepper.setMeasuredAngle(angle, millis());	// tell stepper that this is a measured position
						stepper.setAngle(angle,1);	     			// define a current movement that ends up at current angle. Prevents uncontrolled startup.
					}
					else  {
						logActuator(stepper.getConfig().id);
						logFatal(F("encoder not ok"));
						setError(ENCODER_CALL_FAILED);
					}
				}
			} else {
					actuator->printName();
					logFatal(F("encoder has no stepper"));
					setError(MISCONFIG_ENCODER_WITH_NO_STEPPER);
			}
		}
	}
	

	if (memory.persMem.logSetup) {
		logger->println(F("--- initialize ADC"));
	}
	
	// manual control of a motor uses a poti that is measured with the internal adc
	analogReference(DEFAULT); // use voltage of 3.3V as reference
	
	if (memory.persMem.logSetup) {
		logger->println(F("setup done"));
	}

	// if setup is not successful power down everything
	 if (isError()) {
	 	switchServoPowerSupply(false);
	 	switchStepperPowerSupply(false);
	 }

	 // calibrate timing of each sensor equidistant
	 for (int i = 0;i<numberOfSteppers;i++) {
		 steppers[i].setDueTime(millis() + 20/numberOfSteppers);
	 }
	 setuped= true;
    lights.setSetupMode(false); // tell panel that setup has been success
	return !isError();
}

Actuator* Controller::getActuator(uint8_t actuatorNumber) {
	if ((actuatorNumber>= 0) && (actuatorNumber<numberOfActuators))
		return &actuators[actuatorNumber];
	else
		return NULL;
}	

void Controller::switchManualActuatorControl(bool onOff) {
	adjustWhat = onOff;
	lights.setManualKnobMode(true);
}

void Controller::changeAngle(float incr, int duration_ms) {
	if (currentMotor != NULL) {
		currentMotor->changeAngle(incr, duration_ms);
	}
}

void Controller::switchStepperPowerSupply(bool on) {
	if (on) {
		digitalWrite(POWER_SUPPLY_STEPPER_PIN, HIGH);
	} else {
		digitalWrite(POWER_SUPPLY_STEPPER_PIN, LOW);	// start with stepper to not confuse servo by switch-off impulse
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);
	}
	powered = on;
	lights.setEnableMode(powered);
}

void Controller::switchServoPowerSupply(bool on) {
	if (on) {
		digitalWrite(POWER_SUPPLY_SERVO_PIN, HIGH);
	} else {
		digitalWrite(POWER_SUPPLY_SERVO_PIN, LOW);		
	}
}


void Controller::stepperLoop() {
	if (isSetup()) {
		for (int currentStepper = 0;currentStepper<numberOfSteppers;currentStepper++)
			steppers[currentStepper].loop();
	}
}

void Controller::loop(uint32_t now) {

	stepperLoop(); // send impulses to steppers

	// check the manual control by knob
	if (currentMotor != NULL) {
		if (adjustWhat) {
			if (manualControlTimer.isDue_ms(MOTOR_KNOB_SAMPLE_RATE, now)) {
				// fetch value of potentiometer, returns 0..1024 representing 0..2.56V
				int16_t adcValue = analogRead(MOTOR_KNOB_PIN);

				// compute angle out of adcDiff, potentiometer turns between 0°..270°
				float angle = (float(adcValue-512)/512.0) * (270.0 / 2.0);			
				static float lastAngle = 0;

				if ( (abs(adcValue-512)<500)) {
					// if the sensor is active, set an absolute angle, otherwise, use a relative one
					if (getCurrentActuator()->hasServo() || 
						(getCurrentActuator()->hasEncoder() && getCurrentActuator()->getEncoder().isOk())) {
						if (fabs(angle-lastAngle)> 0.3) {
							logger->print(F("knob:set to "));
							logger->print(angle,1);
						}
						currentMotor->setAngle(angle,MOTOR_KNOB_SAMPLE_RATE);
					}
					else {
						if (fabs(angle-lastAngle)> 0.3) {
							logger->print(F("knob:adjust by "));
							logger->print(angle-lastAngle,1);
						}

						currentMotor->changeAngle(angle-lastAngle,MOTOR_KNOB_SAMPLE_RATE);
					}
				}
				lastAngle = angle;				
			}
		}
	};

	// update the servo position
	if (servoLoopTimer.isDue_ms(SERVO_SAMPLE_RATE/2,now)) {
		static int servoCount = 0;
		servoCount = (servoCount + 1) % MAX_SERVOS;
		servos[servoCount].loop(now);
		stepperLoop(); // meanwhile, send impulses to steppers
	}

	// fetch the angles from the encoders and tell the stepper controller
	for (int encoderIdx = 0;encoderIdx<numberOfEncoders;encoderIdx++) {
		// find corresponding actuator of this encoder
		ActuatorIdentifier actuatorID = encoders[encoderIdx].getConfig().id;
		Actuator* actuator = getActuator(actuatorID);
		if (actuator->hasStepper()) {
			GearedStepperDrive& stepper = actuator->getStepper();
			if (stepper.isDue(now)) {
				if (stepper.getConfig().id != actuatorID) {
					logActuator(actuatorID);
					logger->print(actuatorID);
					logger->print(encoderIdx);
					logger->print(stepper.getConfig().id);
					logFatal(F("wrong stepper identified"));
				}
				bool angleFromEncoderIsOk = false;
				if (encoders[encoderIdx].isOk()) {
					// encoders needs some delay between being asked, otherwise
					// many communication failure happen
					angleFromEncoderIsOk = encoders[encoderIdx].readNewAngleFromSensor(); // measure the encoder's angle
				}
				if (angleFromEncoderIsOk) {
					float currentAngle = encoders[encoderIdx].getAngle();
					stepper.setMeasuredAngle(currentAngle,now);		// set current angle and adapt speed
				} else {
					float currentAngle = stepper.getCurrentAngle();
					stepper.setMeasuredAngle(currentAngle,now);		// set current angle and adapt speed
				}
				stepperLoop(); 									// send impulses to steppers immediately in case a correction has to happen
			}
		}
	}

	if (memory.persMem.logEncoder)
		logAngles();
}

void Controller::logAngles() {
	logger->print(F("angles{"));
	for (int actNo = 0;actNo<numberOfActuators;actNo++) {
		Actuator* actuator=  getActuator(actNo);
		logActuator(actuator->getConfig().id);
		
		if (actuator->hasEncoder()) {
			logger->print(F(" enc="));

			RotaryEncoder& encoder = actuator->getEncoder();
			float measuredAngle = encoder.getAngle();
			logger->print(measuredAngle,2);
			logger->print("(");
			measuredAngle = encoder.getRawSensorAngle();
			logger->print(measuredAngle,2);
			logger->print(")");
		}
		
		if (actuator->hasServo()) {
			logger->print(F(" srv="));
			HerkulexServoDrive& servo= actuator->getServo();
			float measuredAngle = servo.getCurrentAngle();
			logger->print(measuredAngle);
			logger->print("(");
			
			measuredAngle = servo.getRawAngle();
			logger->print(measuredAngle);
			logger->print(") ");
		}
	}
		
	logger->println("}");
}

bool  Controller::checkEncoder(int encoderNo) {
	bool ok = true;
	encoders[encoderNo].checkEncoderVariance(); 
	if (!encoders[encoderNo].isOk())			// isOk returns if communication and checkEncoderVariance went fine
		ok = false;

	return ok;
}
