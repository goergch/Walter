
/*
 * ActuatorConfig.h
 *
 * Created: 07.06.2016 14:05:35
 *  Author: JochenAlt
 */ 




#ifndef __ACTUATOR_CONFIG_H_
#define __ACTUATOR_CONFIG_H_


#include "Arduino.h"
#include "ams_as5048b.h"

#define MAX_ACTUATORS 7						// total number of arms, some servos, some stepper
#define MAX_ENCODERS 5						// total number of encoders
#define MAX_STEPPERS 5						// total number of steppers
#define MAX_SERVOS 2						// total number of servos

#define CONNECTION_BAUD_RATE 115200			// baud rate for connection to main board
#define LOGGER_BAUD_RATE 115200				// baud rate for logger (Soft Serial TX, no RX)

// the AMS AS5048B sensors have an issue, the documented procedure to presistently re-programming the I2C address does not work (at least I did not make it)
// Solution is in the startup procedure: All sensors besides one have a different i2c address (by soldering some pins), but two sensors have the same address
// one of those is powered by pins, such that I can switch it off during startup.
// After starting up, the conflicting sensor's address is reprogrammed transiently, then we other one is powered on and appears on the bus.
#define I2C_ADDRESS_ADDON 1					// add one to I2C address of conflicting sensor
#define I2C_ADDRESS_ADDON_VDD_PIN PIN_B1	// power pins for sensor with conflicting I2C address
#define I2C_ADDRESS_ADDON_GND_PIN PIN_B0	// GND pin for sensor with conflicting I2C address

#define MOTOR_KNOB_PIN PIN_A0				// potentiometer on PCB
#define MOTOR_KNOB_SAMPLE_RATE 100			// every [ms] the potentiometer is sampled
#define SERVO_SAMPLE_RATE  (112*1)			// every [ms] the motors get a new position. 11.2ms is the unit Herkulex servos are working with, sample rate should be a multiple of that
#define SERVO_TARGET_TIME_ADDON (SERVO_SAMPLE_RATE*3) // herkulex servos have their own PID controller, so we need to add some time to a sample to make the movement smooth. Give it 50ms

#define ENCODER_SAMPLE_RATE 50				// every [ms] the motors get a new position
#define ANGLE_SAMPLE_RATE 100				// every [ms] the uC expects a new angle
#define STEPPER_SPEED_SAMPLE_RATE 100L		// in [ms]

#define LED_PIN PIN_B2						// blinking LED
#define LOGGER_TX_PIN PIN_D4				// SoftSerial Log interface that uses TX only
#define POWER_SUPPLY_STEPPER_PIN PIN_B4		// line connected to a relay powering on steppers
#define POWER_SUPPLY_SERVO_PIN PIN_B3		// line connected to a relay powering on servos

#define MAX_INT_16 ((2<<15)-1)
#define sgn(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

#define HERKULEX_MOTOR_ID 0xFD			   // HERKULEX_BROADCAST_ID

enum ActuatorId {GRIPPER=0, HAND=1, WRIST=2, ELLBOW=3,FOREARM=4, UPPERARM=5, HIP=6 };
extern void logActuator(ActuatorId actuatorNumber);

struct ActuatorSetupData {
	ActuatorId id;
	void print();
};

struct ServoSetupData {
	ActuatorId id;
	uint8_t herkulexMotorId;
	bool reverse;
	int16_t minTorque;
	int16_t maxTorque; // actually this is the maximum PWM value of Herkulex servo which is prop. to torque
	int16_t setupSpeed;
	void print();
};


enum Color { BLACK, GREEN, BLUE, RED, NON_COLOR };

struct StepperSetupData {
	ActuatorId id;
	bool direction;			// forward or reverse direction?
	uint8_t microSteps;		// configured micro steps of stepper driver, typically 1, 2, 4, 16

	uint8_t enablePIN;		// enabling the stepper driver
	uint8_t directionPIN;	// selecting direction of stepper driver
	uint8_t clockPIN;		// clock of stepper driver
	
	float degreePerStep;	// typically 1.8 or 0.9° per step
	float amps;				// current of the motor
	
	Color driverA1;			// just for documentation, color of stepper PINS
	Color driverA2;
	Color driverB1;
	Color driverB2;
	
	void print();
};


struct RotaryEncoderSetupData {
	ActuatorId id;
	bool programmI2CAddress;
	uint8_t I2CAddress;
	bool clockwise;
	void print();
};

// all setup data is stored in a global structure
extern ActuatorSetupData		actuatorSetup[MAX_ACTUATORS];
extern StepperSetupData			stepperSetup[MAX_STEPPERS];
extern ServoSetupData			servoSetup[MAX_SERVOS];
extern RotaryEncoderSetupData	encoderSetup[MAX_ENCODERS];

// encoder values have statics, so for calibration we take a some samples and use the average, if all samples are quite close to each other.
#define ENCODER_CHECK_MAX_VARIANCE 1.0 // maximum variance [°] in encoder calibration which is ok
#define ENCODER_CHECK_NO_OF_SAMPLES 4  // so many samples for calibration

#define USE_FAST_DIGITAL_WRITE // use macro based digitalWrite instead of Arduinos methods. Much faster.

extern void logFatal(const __FlashStringHelper *ifsh);
extern void logError(const __FlashStringHelper *ifsh);
extern void logPin(uint8_t PINnumber);

#include "SoftwareSerial.h" // used for logger
extern Stream* logger;

bool scanI2CAddress(uint8_t address, byte &error);
struct RotaryEncoderConfig {
	ActuatorId  id;
	// uint8_t setupid;
	float  nullAngle;
	
	void print();
};

struct ServoConfig {
	ActuatorId  id;

	float nullAngle;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	
	void print();
};

struct StepperConfig {
	ActuatorId id;

	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	float degreePerMicroStep;
	void print();
};


enum ActuatorType { SERVO_TYPE, STEPPER_ENCODER_TYPE, NO_ACTUATOR};
class ActuatorConfig {
	public:
	static void setDefaults();
	void print();

	ActuatorType actuatorType;
	ActuatorId id;
	union ConfigUnion {
		struct {
			ServoConfig servo;
		} servoArm;
		struct {
			RotaryEncoderConfig  encoder;
			StepperConfig stepper;	
		} stepperArm;
	} config;
};

#endif