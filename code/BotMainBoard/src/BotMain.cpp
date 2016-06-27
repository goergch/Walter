//============================================================================
// Name        : BotMain.cpp
// Author      : Jochen Alt
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "MicroControllerInterface.h"
#include <conio.h>

using namespace std;

int main() {
	cout << "BotMain" << endl;
	MicroControllerInterface::getInstance().setup();

	string cmd;
	string echo;
	while (true) {

		if (kbhit()!=0) {
			char c = getch();
			cmd += c;
			if (c == 0x10) {
				string cmd;
				cout << "sending \"" << cmd << "\"";
				MicroControllerInterface::getInstance().sendString(cmd);
			}
		}

		int bytesRead = MicroControllerInterface::getInstance().receive(echo);
		if (bytesRead > 0) {
			cout << echo;
		}

	}
	return 0;
}
