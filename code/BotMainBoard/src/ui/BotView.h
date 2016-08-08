/*
 * BotViewController.h
 *
 *  Created on: 07.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_BOTVIEW_H_
#define UI_BOTVIEW_H_

#include <string>
using namespace std;

class BotView {
public:
	BotView()  {windowHandle = 0;};
	int create(int mainWindow, string pTitle);
	void display();
	void paintBot(JointAngleType angles);

private:
	void setLights();
	void printSubWindowTitle(std::string text );
	void drawCoordSystem(bool withRaster );

	int windowHandle;
	string title;
};

#endif /* UI_BOTVIEW_H_ */
