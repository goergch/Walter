/*
 * BotDrawer.cpp
 *
 *  Created on: 18.08.2016
 *      Author: SuperJochenAlt
 */

#include <ui/BotDrawer.h>

#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GL/Glui.h>

BotDrawer::BotDrawer() {
}

void BotDrawer::display(const JointAngleType& angles, const Pose& pose, const GLfloat* color, const GLfloat* accentColor) {
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();             // Reset the model-view matrix

		housing.display(accentColor,accentColor);

		GLfloat c[3];
		c[0] = color[0];
		c[1] = color[1];
		c[2] = color[2];

		glRotatef(degrees(angles[0]),0.0,1.0, 0.0);
		shoulder.display(c,accentColor);

		glTranslatef(0.0,HipHeight,0.0);
		glRotatef(degrees(angles[1]),1.0,0.0, 0.0);
		c[0] += 0.05;c[1] += 0.05;c[2] += 0.05;
		upperarm.display(c,accentColor);

		glTranslatef(0.0,UpperArmLength,0.0);
		glRotatef(degrees(angles[2]),1.0,0.0, 0.0);
		c[0] += 0.05;c[1] += 0.05;c[2] += 0.05;
		ellbow.display(c,accentColor);

		glTranslatef(0.0,0.0,EllbowLength);
		glRotatef(degrees(angles[3]),0.0,0.0, 1.0);
		forearm.display(c,accentColor);

		glTranslatef(0.0,0.0,ForearmLength);
		glRotatef(degrees(angles[4]),1.0,0.0, 0.0);
		wrist.display(c,accentColor);

		glTranslatef(0.0,0.0,HandLength);
		glRotatef(degrees(angles[5]),0.0,0.0, 1.0);
		c[0] += 0.05;c[1] += 0.05;c[2] += 0.05;
		hand.display(c,accentColor);

		const float gripperLeverRadius=5;
		float gripperAngleDeg = degrees(angles[GRIPPER]);
		glTranslatef(0.0,0.0,ForehandLength);

		// left gripper
		glPushMatrix();
			glTranslatef(gripperLeverRadius*2,0.0,0.0);
			glRotatef(gripperAngleDeg,0.0,1.0, 0.0);
			glutSolidCylinder(gripperLeverRadius, GripperLeverLength, 36, 1);
			glTranslatef(0,0.0,GripperLeverLength);
			glRotatef(-gripperAngleDeg,0.0,1.0, 0.0);
			gripper.display(color, accentColor);
		glPopMatrix();

		// right gripper
		glPushMatrix();
			glTranslatef(-gripperLeverRadius*2,0.0,0.0);
			glRotatef(-gripperAngleDeg,0.0,1.0, 0.0);
			glutSolidCylinder(gripperLeverRadius, GripperLeverLength, 36, 1);
			glTranslatef(0,0.0,GripperLeverLength);
			glRotatef(gripperAngleDeg,0.0,1.0, 0.0);
			glRotatef(180,0.0,0.0, 1.0);
			gripper.display(color, accentColor);
		glPopMatrix();
	glPopMatrix();
	glPopAttrib();
}


void BotDrawer::readFiles(string path) {
	housing.loadFile(path + "/housing.stl");
	shoulder.loadFile(path + "/shoulder.stl");
	upperarm.loadFile(path + "/upperarm.stl");
	ellbow.loadFile(path + "/ellbow.stl");
	forearm.loadFile(path + "/forearm.stl");
	wrist.loadFile(path + "/wrist.stl");
	hand.loadFile(path + "/hand.stl");
	gripper.loadFile(path + "/gripper.stl");

}
void BotDrawer::setup() {
	static bool setupDone = false;
	if (!setupDone) {
		// search for stl files
		if (fileExists("./stl/housing.stl")) {
			readFiles("./stl");
		} else {
			if (fileExists("./housing.stl"))
				readFiles("./");
			else
				readFiles("E:/Projects/Arm/cad/simplified");
		}
		setupDone = true;
	}
}
