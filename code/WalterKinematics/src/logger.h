#ifndef LOGGER_H_
#define LOGGER_H_

#include <thread>
#define _ELPP_THREAD_SAFE
#define ELPP_THREAD_SAFE
#ifdef _WIN32
	#define ELPP_DEFAULT_LOG_FILE "logs/walter.log"
#else
	#define ELPP_DEFAULT_LOG_FILE "/var/log/walter.log"
#endif

#include "easylogging++.h"


#endif
