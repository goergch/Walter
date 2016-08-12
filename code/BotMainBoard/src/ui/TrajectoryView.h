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

class TrajectoryNode {
public:
	TrajectoryNode() {
		smooth = 0;
		duration = 0;
	}
	TrajectoryNode(const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
	}
	void operator= (const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
	}

	Pose pose;
	bool smooth;
	float duration;
	std::string name;
};

class TrajectoryView {
public:
	TrajectoryView();

	void display();
	void create(GLUI *windowHandle, GLUI_Panel* interactivePanel);
private:
	int windowHandle;
	std::vector<TrajectoryNode> trajectory;
};

#endif /* UI_TRAJECTORYVIEW_H_ */
