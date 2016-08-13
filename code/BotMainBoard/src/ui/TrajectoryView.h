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
		pose = par.pose;
	}
	void operator= (const TrajectoryNode& par) {
		smooth = par.smooth;
		duration = par.duration;
		name = par.name;
		pose = par.pose;
	}

	string getText() {

		int par[7];
		int i = 0, j=0;
		par[j++] = pose.position[i++];
		par[j++] = pose.position[i++];
		par[j++] = pose.position[i++];
		i = 0;
		par[j++] = pose.orientation[i++];
		par[j++] = pose.orientation[i++];
		par[j++] = pose.orientation[i++];
		par[j++] = pose.gripperAngle;

		string text = string_format("%s (%i,%i,%i)(%i,%i,%i)(%i)",
							name.c_str(),
							par[0],par[1],par[2],
							par[3],par[4],par[5],
							par[6]);
		return text;
	}
	Pose pose;
	bool smooth;
	float duration;
	string name;
};

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

	vector<TrajectoryNode> trajectory;
private:
	int windowHandle;
};

#endif /* UI_TRAJECTORYVIEW_H_ */
