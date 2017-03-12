//============================================================================
// Name        : main.cpp
// Author      : Jochen Alt
//============================================================================

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "core.h"

#include "Util.h"
#include "Kinematics.h"
#include "WindowController.h"
#include "TrajectorySimulation.h"
#include "ExecutionInvoker.h"
#include "logger.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;

bool exitMode = false;


void signalHandler(int s){
	exitMode = true;
	cout << "Signal " << s << ". Exiting";
	cout.flush();
	exit(1);
}

void setupLogging(int argc, char *argv[]) {
	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

	// setup logger
	el::Configurations defaultConf;
    defaultConf.setToDefault();
    defaultConf.set(el::Level::Error,el::ConfigurationType::Format, "%datetime %level [%func] [%loc] %msg");
    defaultConf.set(el::Level::Error, el::ConfigurationType::Filename, "logs/walter.log");

    defaultConf.set(el::Level::Info,el::ConfigurationType::Format, "%datetime %level %msg");
    defaultConf.set(el::Level::Info, el::ConfigurationType::Filename, "logs/walter.log");

    defaultConf.set(el::Level::Debug, el::ConfigurationType::ToStandardOutput,std::string("false"));
    // defaultConf.set(el::Level::Debug, el::ConfigurationType::Enabled,std::string("false"));

    defaultConf.set(el::Level::Debug, el::ConfigurationType::Format, std::string("%datetime %level [%func] [%loc] %msg"));
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Filename, "logs/walter.log");

    // logging from uC is on level Trace
    defaultConf.set(el::Level::Trace, el::ConfigurationType::ToStandardOutput,std::string("false"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Format, std::string("%datetime %level [uC] %msg"));
    defaultConf.set(el::Level::Trace, el::ConfigurationType::Filename, "logs/walter.log");

    el::Loggers::reconfigureLogger("default", defaultConf);

    LOG(INFO) << "Walter Setup";
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
	cout << "usage: " << prg << " [-h] [-d \"<command>\"] [-i]" << endl
	 	 << "  -host <ip>           webserver ip address" << endl
	 	 << "  -port <port>         webserver's port (if not " << SERVER_PORT << ")" << endl
	     << "   [-h]                help" << endl
		 << "   [-d] \"<command>\"    send command to uC" << endl
	 	 << "   [-i]                direct console to uC" << endl
		 << "   <without par>       start engine and ui" << endl;

}

int main(int argc, char *argv[]) {

	// initialize Logging
	setupLogging(argc, argv);

	// print help
	string webserver_host = SERVER_HOST;
	int webserver_port = SERVER_PORT;
    char * arg= getCmdOption(argv, argv + argc, "-host");
	if(arg != NULL)
		webserver_host = arg;

	arg= getCmdOption(argv, argv + argc, "-port");
	if(arg != NULL)
		webserver_port = atoi(arg);

	// setup webserver
	ExecutionInvoker::getInstance().setHost(webserver_host,webserver_port);
	LOG(INFO) << "calling webserver via http://" << webserver_host << ":" << webserver_port << endl;

	// initialize kinematics and trajectory compilation
	Kinematics::getInstance().setup();

	// initialize trajectory planning controller
	TrajectorySimulation::getInstance().setup(UITrajectorySampleRate);

	// print help
	if(cmdOptionExists(argv, argv+argc, "-h")) {
		printUsage(argv[0]);
		exit(0);
    }

	// provide direct access to cortex per call
    char * directCommand= getCmdOption(argv, argv + argc, "-d");
    if (directCommand) {
    	string directCmdStr(directCommand);
    	string response;
    	cout << ">" << directCommand << endl;
    	ExecutionInvoker::getInstance().directAccess(directCommand,response);
    	cout << "<" << response;
    	exit(0);
    }

	// provide command shell to the cortex
	if (cmdOptionExists(argv, argv+argc, "-i"))
    {
		string cmdStr;
		string response;
		// TrajectoryExecution::getInstance().loguCToConsole();
		cout << "help for help" << endl;

		exitMode = false;
		do {
			cout << ">";
			cout.flush();
			cmdStr = "";
		    std::getline(cin, cmdStr);
		    // cmdStr = "info";
		    if ((cmdStr.compare(0,4,"quit") == 0) || (cmdStr.compare(0,4,"exit") == 0))
		    	exitMode = true;
		    else {
				if (cmdStr.length() > 0) {
			    	ExecutionInvoker::getInstance().directAccess(cmdStr,response);
					cout << response;
				}
		    }
		}
		while (!exitMode);
		exit(0);
	}


	// initialize ui
	bool UISetupOk= WindowController::getInstance().setup(argc, argv);
	if (!UISetupOk) {
		cerr << "ui initialization failed" << endl;
		exit(1);
	}

	while (true) {
		TrajectorySimulation::getInstance().loop();
		delay(1);
	}

    cout << "no dwim running. Try -h" << endl;
	return 0;
}
