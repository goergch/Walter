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

void BotDrawer::display(const JointAngleType& angles, const Pose& pose, GLfloat* color) {
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();             // Reset the model-view matrix

		housing.display(color);

		glRotatef(degrees(angles[0]),0.0,1.0, 0.0);
		shoulder.display(color);
		glTranslatef(0.0,HipHeight,0.0);
		glRotatef(degrees(angles[1]),1.0,0.0, 0.0);
		upperarm.display(color);

		glTranslatef(0.0,UpperArmLength,0.0);
		glRotatef(degrees(angles[2]),1.0,0.0, 0.0);
		ellbow.display(color);

		glTranslatef(0.0,0.0,EllbowLength);
		glRotatef(degrees(angles[3]),0.0,0.0, 1.0);
		forearm.display(color);

		glTranslatef(0.0,0.0,MyFormArmLength);
		glRotatef(degrees(angles[4]),1.0,0.0, 0.0);
		wrist.display(color);

		glTranslatef(0.0,0.0,HandLength);
		glRotatef(degrees(angles[5]),0.0,0.0, 1.0);
		hand.display(color);

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
			gripper.display(color);
		glPopMatrix();

		// right gripper
		glPushMatrix();
			glTranslatef(-gripperLeverRadius*2,0.0,0.0);
			glRotatef(-gripperAngleDeg,0.0,1.0, 0.0);
			glutSolidCylinder(gripperLeverRadius, GripperLeverLength, 36, 1);
			glTranslatef(0,0.0,GripperLeverLength);
			glRotatef(gripperAngleDeg,0.0,1.0, 0.0);
			glRotatef(180,0.0,0.0, 1.0);
			gripper.display(color);
		glPopMatrix();
	glPopMatrix();
	glPopAttrib();
}
void BotDrawer::setup() {
	static bool setupDone = false;
	if (!setupDone) {
		housing.loadFile("E:/Projects/Arm/cad/simplified/housing.stl");
		shoulder.loadFile("E:/Projects/Arm/cad/simplified/shoulder.stl");
		upperarm.loadFile("E:/Projects/Arm/cad/simplified/upperarm.stl");
		ellbow.loadFile("E:/Projects/Arm/cad/simplified/ellbow.stl");
		forearm.loadFile("E:/Projects/Arm/cad/simplified/forearm.stl");
		wrist.loadFile("E:/Projects/Arm/cad/simplified/wrist.stl");
		hand.loadFile("E:/Projects/Arm/cad/simplified/hand.stl");
		gripper.loadFile("E:/Projects/Arm/cad/simplified/gripper.stl");

		setupDone = true;
	}
}

