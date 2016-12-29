#ifndef CORE_H_
#define CORE_H_

#include <string>


using namespace std;
// global error definition
// all possible commands the uC provides
enum ErrorCodeType { NO_ERROR = 0,
	// cortex communication errors
	CHECKSUM_EXPECTED = 1 , CHECKSUM_WRONG = 2,	PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5,
	CORTEX_POWER_ON_WITHOUT_SETUP= 6,

	// encoder errors
	ENCODER_CONNECTION_FAILED = 10 ,ENCODER_CALL_FAILED = 11,ENCODER_CHECK_FAILED = 12,

	// configuration errors
	MISCONFIG_TOO_MANY_SERVOS= 20,	MISCONFIG_TOO_MANY_ENCODERS = 21,MISCONFIG_TOO_MANY_STEPPERS = 22,
	MISCONFIG_SERVO_WITH_STEPPER=23, MISCONFIG_SERVO_WITH_ENCODER = 24,  MISCONFIG_ENCODER_STEPPER_MISMATCH = 25,
	MISCONFIG_NO_STEPPERS=26, MISCONFIG_NO_ENCODERS = 27, MISCONFIG_ENCODER_WITH_NO_STEPPER=28,
	MISCONFIG_STEPPER=29, MISCONFIG_SERVO=30,

	// herkulex errors
	HERKULEX_COMMUNICATION_FAILED = 40, HERKULEX_STATUS_FAILED = 41,

	// Cortex errors
	CORTEX_CONNECTION_FAILED = 50, CORTEX_COM_FAILED = 51, CORTEX_LOG_COM_FAILED=52, CORTEX_NO_RESPONSE =53,

	// Webserver errors
	WEBSERVER_TIMEOUT = 60,

	// last exit Brooklyn
	UNKNOWN_ERROR= 99
};

// set error gto NO_ERROR
void resetError();

// return last error set by setError
ErrorCodeType getLastError();

// set the passed error
void setError(ErrorCodeType err);

// returns prosa message of error code
string getErrorMessage(ErrorCodeType err);
string getLastErrorMessage();

// true, if error has been set
bool isError();


#endif
