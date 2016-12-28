#ifndef CORE_H_
#define CORE_H_

#include <string>


using namespace std;
// global error definition
// all possible commands the uC provides
enum ErrorCodeType { NO_ERROR= 0,
				// errors from HostCommunication
				CHECKSUM_EXPECTED = 1 , CHECKSUM_WRONG = 2,	PARAM_WRONG = 3, PARAM_NUMBER_WRONG = 4, UNRECOGNIZED_CMD = 5, NO_RESPONSE_CODE= 7,

				// encoders
				ENCODER_CONNECTION_FAILED = 8 ,ENCODER_CALL_FAILED = 21,ENCODER_CHECK_FAILED = 16,

				// configuration
				MISCONFIG_TOO_MANY_SERVOS= 9,	MISCONFIG_TOO_MANY_ENCODERS = 14,MISCONFIG_TOO_MANY_STEPPERS = 15,
				MISCONFIG_SERVO_WITH_STEPPER=12, MISCONFIG_SERVO_WITH_ENCODER = 13,  MISCONFIG_ENCODER_STEPPER_MISMATCH = 20,
				MISCONFIG_NO_STEPPERS=17, MISCONFIG_NO_ENCODERS = 18, MISCONFIG_ENCODER_WITH_NO_STEPPER=22,
				MISCONFIG_STEPPER=19, MISCONFIG_SERVO=14,

				// herkulex
				HERKULEX_COMMUNICATION_FAILED = 10, HERKULEX_STATUS_FAILED = 11,

				// CortexController
				CORTEX_CONNECTION_FAILED = 23,
				// last exit Brooklyn
				UNKNOWN_ERROR= 99 };

void resetError();
ErrorCodeType getLastError();
void setError(ErrorCodeType err);
string getErrorMessage(ErrorCodeType err);
bool isError();


#endif
