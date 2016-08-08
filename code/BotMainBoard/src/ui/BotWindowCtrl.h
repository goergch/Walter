/*
 * BotWindowCtrl.h
 *
 *  Created on: 05.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_BOTWINDOWCTRL_H_
#define UI_BOTWINDOWCTRL_H_

#if defined(_WIN32)
#include <windows.h>  // openGL windows
#endif
#include <thread>

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GL/Glui.h>

#include "spatial.h"
#include "Kinematics.h"
#include "BotView.h"

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
	void setTcpInputCallback(void (* callback)( Pose pose, KinematicConfigurationType &config, JointAngleType &angles, std::vector<KinematicConfigurationType>& validConfigurations));

	void callbackChangedTCP();
	void callbackChangedAngles();

	BotView topLeft;
	BotView topRight;
	BotView bottomLeft;
	BotView bottomRight;
private:

	 void eventLoop();
	 GLUI* createInteractiveWindow(int mainWindow);


	 void (*anglesCallback)( JointAngleType angles, Pose &pose, KinematicConfigurationType &config);
	 void (*tcpCallback)( Pose pose, KinematicConfigurationType &config, JointAngleType &angles,  std::vector<KinematicConfigurationType>& validConfigurations);
	 std::thread* eventLoopThread;
	 bool uiReady;



};

extern BotWindowCtrl botWindowCtrl;


#endif /* UI_BOTWINDOWCTRL_H_ */
