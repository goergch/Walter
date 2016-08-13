/*
 * TrajectoryView.h
 *
 *  Created on: 12.08.2016
 *      Author: SuperJochenAlt
 */

#ifndef UI_TRAJECTORYVIEW_H_
#define UI_TRAJECTORYVIEW_H_

#if defined(_WIN32)
#include <windows.h>  // openGL windows
#endif

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GL/Glui.h>

#include "spatial.h"
#include "Util.h"

using namespace std;


class TrajectoryView {
public:
	TrajectoryView();

	static TrajectoryView& getInstance() {
			static TrajectoryView instance;
			return instance;
	}

	void display();
	void create(GLUI *windowHandle, GLUI_Panel* interactivePanel);
	void fillTrajectoryListControl();
private:
	int windowHandle;
};

#endif /* UI_TRAJECTORYVIEW_H_ */
