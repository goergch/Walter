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

using namespace std;

bool setup() {
	cout << "Snorre" << endl;
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
