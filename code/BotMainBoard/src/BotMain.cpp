//============================================================================
// Name        : BotMain.cpp
// Author      : Jochen Alt
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "ActuatorCtrlInterface.h"
#include "Util.h"

#define _ELPP_THREAD_SAFE
#define ELPP_DEFAULT_LOG_FILE "logs/Snorre.log"
#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP


using namespace std;

bool setup() {

	// setup logger
	el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Error,el::ConfigurationType::Format, "%datetime %level [%func] [%loc] %msg");
    defaultConf.set(el::Level::Error, el::ConfigurationType::Filename, "logs/snorre.log");

    defaultConf.set(el::Level::Info,el::ConfigurationType::Format, "%datetime %level %msg");
    defaultConf.set(el::Level::Info, el::ConfigurationType::Filename, "logs/snorre.log");


    defaultConf.set(el::Level::Debug, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format, std::string("%datetime %level [%func] [%loc] %msg"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Filename, "logs/snorre.log");

    // logging from uC is on level Trace
    defaultConf.set(el::Level::Trace, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Format, std::string("%datetime %level [uC] %msg"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Filename, "logs/snorre.log");

    el::Loggers::reconfigureLogger("default", defaultConf);
    LOG(INFO) << "Snorre Setup";

	bool ok = ActuatorCtrlInterface::getInstance().setupCommunication();
	return ok;
}

char* getCmdOption(char ** begin, char ** end, const std::string & option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

void printUsage(string prg) {
	cout << "usage: " << prg << " [-h] [-d]" << endl
		 << "   [-h]  				help" << endl
		 << "   [-d] \"<command>\" 		send command to uC" << endl
	 	 << "   [-i] 					direct console to uC" << endl;

}
int main(int argc, char *argv[]) {
	bool ok = setup();
	ok = true;
	if (!ok) {
		cerr << "setup failed" << endl;
		exit(1);
	}

	if(cmdOptionExists(argv, argv+argc, "-h"))
    {
		printUsage(argv[0]);
		exit(0);
    }

    char * directCommand= getCmdOption(argv, argv + argc, "-d");
    if (directCommand)
    {
    	string directCmdStr(directCommand);
    	string reponse;
		bool okOrNOk;
    	cout << ">" << directCommand << endl;
    	ActuatorCtrlInterface::getInstance().directAccess(directCommand,reponse, okOrNOk);
    	cout << "<" << reponse;
    	if (okOrNOk)
    		cout << "ok" << endl;
    	else
    		cout << "nok" << endl;

    	exit(0);
    }

	if (cmdOptionExists(argv, argv+argc, "-i"))
    {
		string cmdStr;
		string reponse;
		ActuatorCtrlInterface::getInstance().loguCToConsole();
		do {
			cout << ">";
		    std::getline(cin, cmdStr);
			bool okOrNOk;
			if (cmdStr.length() > 0) {
				ActuatorCtrlInterface::getInstance().directAccess(cmdStr,reponse, okOrNOk);
		    	cout << reponse;
		    	if (okOrNOk)
		    		cout << "ok" << endl;
		    	else
		    		cout << "nok" << endl;
			}
		}
		while ((cmdStr.compare("quit") != 0) && (cmdStr.compare("exit") != 0));
		exit(0);
	}

    cout << "no dwim running. Try -h" << endl;
	return 0;
}
