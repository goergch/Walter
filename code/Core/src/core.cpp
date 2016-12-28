/*
 * core.cpp
 *
 *  Created on: 14.07.2016
 *      Author: JochenAlt
 */


#include "core.h"
#include <sstream>
#include <string>

ErrorCodeType glbError = NO_ERROR;

void resetError() {
	glbError = NO_ERROR;
}

ErrorCodeType getLastError() {
	return glbError;
}

void setError(ErrorCodeType err) {
	glbError = err;
}

bool isError() {
	return glbError != NO_ERROR;
}


std::string getErrorMessage(ErrorCodeType err) {
	std::ostringstream msg;
	switch (err) {
	case NO_ERROR: msg << "no error";break;

	// hostCommunication
	case CHECKSUM_EXPECTED: msg << "checksum expected";break;
	case CHECKSUM_WRONG: msg << "checksum wrong";break;
	case PARAM_WRONG: msg << "parameter wrong";break;
	case PARAM_NUMBER_WRONG: msg << "number of parameter wrong";break;
	case UNRECOGNIZED_CMD: msg << "unknown command";break;
	case NO_RESPONSE_CODE: msg << "no reponse code";break;

	// encoder
	case ENCODER_CONNECTION_FAILED: msg << "Encoder connection failed";break;
	case ENCODER_CALL_FAILED: msg << "Rotary Encoder Call failed";break;
	case ENCODER_CHECK_FAILED: msg << "Rotary Encoder Check failed";break;

	// servos
	case HERKULEX_COMMUNICATION_FAILED: msg << "HerkuleX communication failed";break;
	case HERKULEX_STATUS_FAILED: msg << "HerkuleX status could not be retrieved";break;

	//cortexController
	case CORTEX_CONNECTION_FAILED: msg << "cortex connection failed";break;


	case UNKNOWN_ERROR: msg << "unknown error";break;

	default:
		msg << "unknown message";
	}
	msg << " (" << (int)err << ")";

	return msg.str();
}
