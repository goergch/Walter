//============================================================================
// Name        : BotMain.cpp
// Author      : Jochen Alt
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "ActuatorCtrlInterface.h"
#include "Util.h"
#include "Kinematics.h"
#include "ui/BotWindowCtrl.h"
#include "MainBotController.h"

INITIALIZE_EASYLOGGINGPP


using namespace std;

void signalHandler(int s){
	printf("%d signal. Exiting.\n",s);
	exit(1);
}

bool setup(int argc, char *argv[]) {
	// catch SIGINT (ctrl-C)
    signal (SIGINT,signalHandler);

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

    // initialize kinematics
	Kinematics::getInstance().setup();

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
	cout << "usage: " << prg << " [-h] [-d \"<command>\"] [-i]" << endl
		 << "   [-h]                help" << endl
		 << "   [-d] \"<command>\"    send command to uC" << endl
	 	 << "   [-i]                direct console to uC" << endl
		 << "   <without par>       start engine and ui" << endl;

}

int main(int argc, char *argv[]) {

	bool ok = setup(argc, argv);
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
		bool exitMode = false;
		do {
			cout << "help for help" << endl << ">";
		    std::getline(cin, cmdStr);
		    if ((cmdStr.compare(0,4,"quit") == 0) || (cmdStr.compare(0,4,"exit") == 0))
		    	exitMode = true;
		    else {
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
		}
		while (!exitMode);
		exit(0);
	}

	// initialize main controller
	MainBotController::getInstance().setup();

	// initialize ui
	ok = botWindowCtrl.setup(argc, argv);
	if (!ok) {
		cerr << "ui initialization failed" << endl;
		exit(1);
	}

	while (true) {
		MainBotController::getInstance().loop();
	}

    cout << "no dwim running. Try -h" << endl;
	return 0;
}
