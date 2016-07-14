//============================================================================
// Name        : BotMain.cpp
// Author      : Jochen Alt
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ActuatorCtrlInterface.h"

#include <conio.h>

#define ELPP_DEFAULT_LOG_FILE "logs/Snorre.log"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP


using namespace std;

bool setup() {

	// setup logger
	el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Info,el::ConfigurationType::Format, "%datetime %level %msg");
    defaultConf.set(el::Level::Info, el::ConfigurationType::Filename, "logs/snorre.log");
    defaultConf.set(el::Level::Debug, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format, std::string("%datetime %level [%func] [%loc] %msg"));


    el::Loggers::reconfigureLogger("default", defaultConf);
    LOG(INFO) << "Snorre Setup";

	bool ok = ActuatorCtrlInterface::getInstance().setup();
	if (ok) {
	}
	return ok;
}


int main() {
	bool ok = setup();


	if (ok) {
		string cmd;
		string echo;
		while (true) {

			if (kbhit()!=0) {
				char c = getch();
				cmd += c;
				if (c == 0x10) {
					cout << "sending \"" << cmd << "\"";
				}
			}

			int bytesRead = ActuatorCtrlInterface::getInstance().receive(echo, 100);
			if (bytesRead > 0) {
				cout << echo;
			}

		}
	}
	return 0;
}
