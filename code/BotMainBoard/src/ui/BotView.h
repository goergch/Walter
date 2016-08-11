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

	BotView()  {
		windowHandle = 0;
		startupAnimationRatio = 0.0f;
		currEyeDistance = ViewEyeDistance;
		baseAngle = -45;
		heightAngle = 0;
	};
	int create(int mainWindow, string pTitle, View view);

	void display();
	void reshape(int x,int y, int w, int h);
	void setEyePosition(float* pEyePosition);
	void setEyePosition(float currEyeDistance, float baseAngle, float heightAngle);
	void changeEyePosition(float currEyeDistance, float baseAngle, float heightAngle);

	void setAngles(JointAngleType pAngles);
	void setStartupAnimationRatio(float ratio);
	void hide();
	void show();
	void getTCPDot(GLint* &viewport, GLdouble* &modelview, GLdouble* &projmatrix);
private:
	void setLights();
	void printSubWindowTitle(std::string text );
	void drawCoordSystem(bool withRaster );
	void paintBot(const JointAngleType& angles);
	void setWindowPerspective();
	float startupFactor(float start, float target);


	int windowHandle;
	string title;
	View view;
	float eyePosition[3] = { 0,0,0 };
	float startupAnimationRatio;
	JointAngleType angles;

	float currEyeDistance = 0;
	float baseAngle = 0;
	float heightAngle = 0;

	GLint viewport[4];                  // Where The Viewport Values Will Be Stored
	GLdouble modelview[16];             // Where The 16 Doubles Of The Modelview Matrix Are To Be Stored
	GLdouble projection[16];

};

#endif /* UI_BOTVIEW_H_ */
