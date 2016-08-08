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
	enum View { FRONT_VIEW, TOP_VIEW, RIGHT_VIEW, _3D_VIEW };

	BotView()  {windowHandle = 0;startupAnimationRatio = 0.0f;};
	int create(int mainWindow, string pTitle, View view);

	void display(JointAngleType angles);
	void setEyePosition(float* pEyePosition);
	void setStartupAnimationRatio(float ratio);
private:
	void setLights();
	void printSubWindowTitle(std::string text );
	void drawCoordSystem(bool withRaster );
	void paintBot(JointAngleType angles);
	void setWindowPerspective();
	float startupFactor(float start, float target);


	int windowHandle;
	string title;
	View view;
	float eyePosition[3] = { 0,0,0 };
	float startupAnimationRatio;

};

#endif /* UI_BOTVIEW_H_ */
