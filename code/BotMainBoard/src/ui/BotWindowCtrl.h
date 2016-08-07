/*
 * BotWindowCtrl.h
 *
 *  Created on: 05.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_BOTWINDOWCTRL_H_
#define UI_BOTWINDOWCTRL_H_

#include <windows.h>  // openGL windows
#include <thread>

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GL/Glui.h>

#include "spatial.h"
#include "Kinematics.h"

class BotWindowCtrl {
public:
	BotWindowCtrl() {
		anglesCallback = NULL;
		tcpCallback = NULL;
		eventLoopThread = NULL;
		uiReady = false;
	};
	bool setup(int argc, char** argv);

	void setAnglesCallback(void (* callback)( JointAngleType angles, Pose &pose, KinematicConfigurationType &config));
	void setTcpInputCallback(void (* callback)( Pose pose, KinematicConfigurationType config, JointAngleType &angles));

	void callbackChangedTCP();
	void callbackChangedAngles();
private:

	 void eventLoop();
	 int createBotSubWindow(int mainWindowHandle);
	 GLUI* createInteractiveWindow(int mainWindow);

	 void (*anglesCallback)( JointAngleType angles, Pose &pose, KinematicConfigurationType &config);
	 void (*tcpCallback)( Pose pose, KinematicConfigurationType config, JointAngleType &angles);
	 std::thread* eventLoopThread;
	 bool uiReady;

};

extern BotWindowCtrl botWindowCtrl;


#endif /* UI_BOTWINDOWCTRL_H_ */
