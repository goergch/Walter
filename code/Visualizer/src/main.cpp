//============================================================================
// Name        : BotMain.cpp
// Author      : Jochen Alt
//============================================================================

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "Util.h"
#include "Kinematics.h"
#include "BotWindowCtrl.h"
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


	/*
	JointAngleType currentAngle = { radians(12), radians(0), radians(0), radians(0), radians(0), radians(0), radians(50) };
	Pose pose;
	Kinematics::getInstance().computeForwardKinematics(currentAngle, pose);

		pose.position[X] = 323.7;
		pose.position[Y] =68.81;
		pose.position[Z] = 400;

		pose.orientation[X] =0;
		pose.orientation[Y] =0.1;
		pose.orientation[Z] =0.1;
		pose.gripperAngle = 0.6109;
		KinematicsSolutionType solution;
		std::vector<KinematicsSolutionType> validConfigurations;
		Kinematics::getInstance().computeInverseKinematics(actuatorLimits, currentAngle, pose, solution, validConfigurations);
*/
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
		 << "   [-h]                help" << endl
		 << "   [-d] \"<command>\"    send command to uC" << endl
	 	 << "   [-i]                direct console to uC" << endl
		 << "   <without par>       start engine and ui" << endl;

}

int main(int argc, char *argv[]) {

	// initialize Logging
	setupLogging(argc, argv);

	// setup webserver
	ExecutionInvoker::getInstance().setHost(SERVER_HOST,SERVER_PORT);
	cout << "calling webserver via http://" << SERVER_HOST << ":" << SERVER_PORT << endl;

	// initialize kinematics and trajectory compilation
	Kinematics::getInstance().setup();

	// initialize trajectory planning controller
	TrajectorySimulation::getInstance().setup();


	// print help
	if(cmdOptionExists(argv, argv+argc, "-h")) {
		printUsage(argv[0]);
		exit(0);
    }

	// provide direct access to cortex per call
    char * directCommand= getCmdOption(argv, argv + argc, "-d");
    if (directCommand) {
    	string directCmdStr(directCommand);
    	string reponse;
    	cout << ">" << directCommand << endl;
    	ExecutionInvoker::getInstance().directAccess(directCommand,reponse);
    	cout << "<" << reponse;
    	exit(0);
    }

	// provide command shell to the cortex
	if (cmdOptionExists(argv, argv+argc, "-i"))
    {
		string cmdStr;
		string reponse;
		// TrajectoryExecution::getInstance().loguCToConsole();
		cout << "help for help" << endl;

		exitMode = false;
		do {
			cout << ">";
			cout.flush();
			cmdStr = "";
		    std::getline(cin, cmdStr);
		    if ((cmdStr.compare(0,4,"quit") == 0) || (cmdStr.compare(0,4,"exit") == 0))
		    	exitMode = true;
		    else {
				if (cmdStr.length() > 0) {
			    	ExecutionInvoker::getInstance().directAccess(directCommand,reponse);
					cout << reponse;
				}
		    }
		}
		while (!exitMode);
		exit(0);
	}


	// initialize ui
	bool UISetupOk= BotWindowCtrl::getInstance().setup(argc, argv);
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
