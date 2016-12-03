
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
#define SERVO_MOVE_DURATION (SERVO_SAMPLE_RATE*2) // herkulex servos have their own PID controller, so we need to add some time to a sample to make the movement smooth. Give it 50ms

#define ENCODER_SAMPLE_RATE 20				// every [ms] the motors get a new position ( encoders could work up to 500Hz, but then we have less time to control the steppers))
#define ENCODER_FILTER_RESPONSE_TIME 30		// complementary filter of rotary encoder has this response time in [ms] 

#define ANGLE_SAMPLE_RATE 100				// every [ms] the uC expects a new angle
#define STEPPER_SPEED_SAMPLE_RATE 100L		// in [ms]

#define LED_PIN PIN_B2						// blinking LED
#define LOGGER_TX_PIN PIN_D4				// SoftSerial Log interface that uses TX only
#define POWER_SUPPLY_STEPPER_PIN PIN_B3		// line connected to a relay powering on steppers
#define POWER_SUPPLY_SERVO_PIN PIN_B4		// line connected to a relay powering on servos

#define HAND_HERKULEX_MOTOR_ID    0xFD		// this is the HERKULEX_BROADCAST_ID used for all servos
#define GRIPPER_HERKULEX_MOTOR_ID 0xFC		// this ID has been programmed into the gripper servo explicitly

enum ActuatorIdentifier {HIP=0 , UPPERARM=1, FOREARM=2, ELLBOW=3, WRIST=4, HAND=5, GRIPPER=6 };
extern void logActuator(ActuatorIdentifier actuatorNumber);

struct ActuatorSetupData {
	ActuatorIdentifier id;
	void print();
};

struct ServoSetupData {
	ActuatorIdentifier id;
	uint8_t herkulexMotorId;
	bool reverse;
	int16_t minTorque;
	int16_t maxTorque; // actually this is the maximum PWM value of Herkulex servo which is prop. to torque
	int16_t setupSpeed;
	void print();
};

enum Color { BLACK, GREEN, BLUE, RED, NON_COLOR };

struct StepperSetupData {
	ActuatorIdentifier id;
	bool direction;			// forward or reverse direction?
	uint8_t microSteps;		// configured micro steps of stepper driver, typically 1, 2, 4, 16

	uint8_t enablePIN;		// enabling the stepper driver
	uint8_t directionPIN;	// selecting direction of stepper driver
	uint8_t clockPIN;		// clock of stepper driver
	
	float degreePerStep;	// typically 1.8 or 0.9° per step
	float maxAcc;			// maximum acceleration in rpm/s
	float maxSpeed;			// maximum speed in rpm
	float amps;				// current of the motor, not in use, for documentation only
	
	Color driverA1;			// not in use, just for documentation, color of stepper PINS
	Color driverA2;
	Color driverB1;
	Color driverB2;
	
	void print();
};


struct RotaryEncoderSetupData {
	ActuatorIdentifier id;
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
#define ENCODER_CHECK_MAX_VARIANCE 1.0	// maximum variance [°] in encoder calibration which is ok
#define ENCODER_CHECK_NO_OF_SAMPLES 4	// so many samples for calibration

struct RotaryEncoderConfig {
	ActuatorIdentifier  id;
	// uint8_t setupid;
	float  nullAngle;
	
	void print();
};

struct ServoConfig {
	ActuatorIdentifier  id;

	float nullAngle;
	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	
	void print();
};

struct StepperConfig {
	ActuatorIdentifier id;

	float  maxAngle;			// [°]
	float  minAngle;			// [°]
	float degreePerMicroStep;
	float kP;
	float kD;
	float kG;

	void print();
};


enum ActuatorType { SERVO_TYPE, STEPPER_ENCODER_TYPE, NO_ACTUATOR};
class ActuatorConfig {
	public:
	static void setDefaults();
	void print();

	ActuatorType actuatorType;
	ActuatorIdentifier id;
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