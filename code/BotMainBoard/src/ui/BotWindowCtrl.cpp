/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include "BotWindowCtrl.h"
#include "Util.h"

using namespace std;

BotWindowCtrl botWindowCtrl;

// Window size
int WindowWidth = 800;				// initial window size
int WindowHeight = 600;

int WindowGap=10;					// gap between subwindows
int SubWindowHeight = 10;			// initial height of a subwindow
int SubWindowWidth = 10;			// initial weight of a subwindow
int MainSubWindowHeight = 10;		// initial height of a subwindow
int MainSubWindowWidth = 10;		// initial weight of a subwindow
int InteractiveWindowWidth=220;		// initial width of the interactive window

// layout, we can do quad layout or single layout
enum LayoutType { SINGLE_LAYOUT = 0, QUAD_LAYOUT = 1, MIXED_LAYOUT=2 };
int layoutButtonSelection=QUAD_LAYOUT;		// live variable of radio group

static GLfloat glMainWindowColor[] 		= {1.0,1.0,1.0};
static GLfloat glSubWindowColor[] 		= {0.97,0.97,0.97};
static GLfloat glBotArmColor[] 			= { 1.0f, 0.3f, 0.2f };
static GLfloat glBotJointColor[] 		= { 0.5f, 0.6f, 0.6f };
static GLfloat glWindowTitleColor[] 	= { 1.0f, 1.0f, 1.0f };
static GLfloat glCoordSystemColor4v[] 	= { 0.03f, 0.27f, 0.32f,0.5f };
static GLfloat glRasterColor3v[] 		= { 0.73f, 0.77f, 0.82f };

// 3d moving window eye position
const float glEyeDistance = 1500.0f;	// distance of the eye to the bot
const float ViewHeight = 800.0f;		// height of the bot to be viewed
float currEyeAngle= -45;				// current eye position of moveable subwindow
float currEyeHeightAngle= 0;				// current eye position of moveable subwindow

float currEyeDistance = glEyeDistance;	// current eye distance of moveable subwindow
float eyePosition[3] = {currEyeDistance*sinf(radians(currEyeAngle)),ViewHeight,currEyeDistance*cosf(radians(currEyeAngle))};

// startup animation
float startUpDuration = 5000;			// duration of startup animation
float startupRatio= 0.0; 				// between 0 and 1, indicates the position within the startup animation

// handles of opengl windows and subwindows
int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;	// window handler of windows
GLUI *wInteractive = NULL;				// interactive window handler

GLUI_Panel* anglesPanel = NULL;
GLUI_Panel* buttonPanel = NULL;
GLUI_Spinner* angleSpinner[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
GLUI_Spinner* tcpCoordSpinner[] = {NULL,NULL,NULL};
GLUI_Button* layoutButton = NULL;

float botAngles[7] = {0.0,0.0,0.0,0.0,0.0,0.0,30.0 };
string angleName[] = { "hip","upperarm","forearm","ellbow", "wrist", "hand", "gripper" };
enum ActuatorType { HIP=0, UPPERARM = 1, FOREARM=2, ELLBOW = 3, WRIST=4, HAND=5,GRIPPER=6};
enum CoordType { X=0, Y=1, Z=2 };

float tcp[3] = {0,0,0 };
bool kinematicsHasChanged = false; 				// true, if something in kinematics has changed

void setLights()
{
  GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
  GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position0[] = {glEyeDistance, 3*glEyeDistance, glEyeDistance, 0.0};		// ceiling left
  GLfloat light_position1[] = {-glEyeDistance, 3*glEyeDistance, glEyeDistance, 0.0};	// ceiling right
  GLfloat light_position2[] = {0, 3*glEyeDistance, -glEyeDistance, 0.0};				// far away from the back

  GLfloat mat_ambient[] =  {0.6, 0.6, 0.6, 1.0};
  GLfloat mat_diffuse[] =  {0.4, 0.8, 0.4, 1.0};
  GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat mat_shinynes[] = {50.0};

  glMaterialfv(GL_LIGHT0, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_LIGHT0, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_LIGHT0, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_LIGHT0, GL_SPECULAR, mat_shinynes);
  glMaterialfv(GL_LIGHT1, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_LIGHT1, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_LIGHT1, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_LIGHT1, GL_SPECULAR, mat_shinynes);
  glMaterialfv(GL_LIGHT2, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_LIGHT2, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_LIGHT2, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_LIGHT2, GL_SPECULAR, mat_shinynes);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
  glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

  glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
  glLightfv(GL_LIGHT2, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT2, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT2, GL_POSITION, light_position2);

  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);

  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
}

void printSubWindowTitle(std::string text) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();                 // Reset the model-view matrix
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glWindowTitleColor);
	glRasterPos2f(-0.9,0.8);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) text.c_str());
}

void printKinematics() {
	static float lastBotAngle[7];
	for (int i = 0;i<7;i++) {
		float spinnerValue = botAngles[i];
		if (spinnerValue != lastBotAngle[i]) {
			angleSpinner[i]->set_float_val(spinnerValue); // set only when necessary, otherwise the cursor blinks
			lastBotAngle[i] = botAngles[i];
		}
	}
	static float lastTcp[6];

	for (int i = 0;i<3;i++) {
		float spinnerValue = tcp[i];
		if (spinnerValue != lastTcp[i]) {
			tcpCoordSpinner[i]->set_float_val(spinnerValue); // set only when necessary, otherwise the cursor blinks
			lastTcp[i] = spinnerValue;
		}
	}
}

void drawCoordSystem(bool withRaster) {
	// draw coordinate system
	const float axisLength = 500.0f;
	const float arrowLength = 20.0f;
	const float unitLength = 100.0f;
	const float rasterLineLength = axisLength*2;
	if (withRaster) {
		glPushAttrib(GL_LIGHTING_BIT);
		glBegin(GL_LINES);
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, glRasterColor3v);
			glColor3fv(glRasterColor3v);
			for (float i = -rasterLineLength;i<=rasterLineLength;i = i + unitLength ) {
				glVertex3f(i, 0.0, -rasterLineLength);glVertex3f(i,0.0f, rasterLineLength);
			}
			for (float i = -rasterLineLength;i<=rasterLineLength;i = i + unitLength ) {
				glVertex3f(-rasterLineLength, 0.0f, i);glVertex3f(rasterLineLength, 0.0f, i);
			}
		glEnd();
		glPopAttrib();
	}

	glBegin(GL_LINES);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glCoordSystemColor4v);
		glColor4fv(glCoordSystemColor4v);

		// robot's x-axis
		glVertex3f(0.0f, 0.0f, -arrowLength);glVertex3f(0.0f, 0.0f, axisLength);
		glVertex3f(0.0f, 0.0f, axisLength);glVertex3f(0.0f,+arrowLength/2, axisLength-arrowLength);
		glVertex3f(0.0f, 0.0f, axisLength);glVertex3f(0.0f,-arrowLength/2, axisLength-arrowLength);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(0.0f, -arrowLength/2, i);glVertex3f(0.0f,+arrowLength/2, i);
		}

		// robot's y-axis
		glVertex3f(-arrowLength, 0.0f, 0.0f);glVertex3f(axisLength, 0.0f, 0.0f);
		glVertex3f(axisLength, 0.0f, 0.0f);glVertex3f(axisLength-arrowLength, -arrowLength/2, 0.0f);
		glVertex3f(axisLength, 0.0f, 0.0f);glVertex3f(axisLength-arrowLength, arrowLength/2, 0.0f);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(i, -arrowLength/2, 0.0f);glVertex3f(i,+arrowLength/2, 0.0f);
		}

		// robot's z-axis
		glVertex3f(0.0f, -arrowLength, 0.0f);glVertex3f(0.0f, axisLength,0.0f);
		glVertex3f(0.0f, axisLength,0.0f);glVertex3f(+arrowLength/2,axisLength-arrowLength, 0.0f);
		glVertex3f(0.0f, axisLength,0.0f);glVertex3f(-arrowLength/2, axisLength-arrowLength,0.0f);
		for (float i = 0;i<axisLength;i = i + unitLength ) {
			glVertex3f(-arrowLength/2, i,0.0f);glVertex3f(+arrowLength/2, i,0.0f);
		}
	glEnd();

	glRasterPos3f(axisLength+arrowLength, 0.0f, 0.0f);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "y");
	glRasterPos3f(0.0f, 0.0f, axisLength+arrowLength);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "x");
	glRasterPos3f(0.0f, axisLength+arrowLength,0.0f);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "z");
}

void paintBot() {
	const float baseplateRadius= 140;
	const float baseplateHeight= 20;

	const float baseLength = 110;
	const float baseRadius = 60;
	const float baseJointRadius = 60;

	const float upperarmLength = 210;
	const float upperarmJointRadius= 45;
	const float upperarmRadius = 45;

	const float forearmLength = 240;
	const float forearmJointRadius= 35;
	const float forearmRadius = 35;

	const float handLength= 90;
	const float handJointRadius= 23;
	const float handRadius= 23;

	const float gripperLength= 70;
	const float gripperRadius=10;

	const float gripperLeverLength= 45;
	const float gripperLeverRadius=5;

	glMatrixMode(GL_MODELVIEW);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1],glSubWindowColor[2],0.0f); // Set background color to white and opaque

	// coord system
	drawCoordSystem(true);

	// base plate
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidCylinder(baseplateRadius, baseplateHeight, 36, 1);

	// shoulder
	glTranslatef(0.0, 0.0,baseplateHeight);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(baseRadius, baseLength, 36, 1);

	// shoulder joint
	glTranslatef(0.0,0.0,baseLength);  // Move right and into the screen
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(baseJointRadius, 36, 36);

	// upperarm
	glRotatef(botAngles[0],0.0,0.0, 1.0); // turn along angle
	glRotatef(botAngles[1],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(upperarmRadius, upperarmLength, 36, 1);

	// upperarm joint
	glTranslatef(0.0,0.0,upperarmLength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(upperarmJointRadius, 36, 36);

	// forearm
	glRotatef(90+botAngles[2],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(forearmRadius, forearmLength, 36, 1);

	// forearm joint
	glRotatef(botAngles[3],0.0,0.0, 1.0); // rotate along base angle
	glTranslatef(0.0,0.0,forearmLength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glPushMatrix(),
		glTranslatef(forearmJointRadius/2,0.0,0);  // move to its start height
		glutSolidSphere(forearmJointRadius, 36, 36);
	glPopMatrix();
	glPushMatrix(),
		glTranslatef(-forearmJointRadius/2,0.0,0);  // move to its start height
		glutSolidSphere(forearmJointRadius, 36, 36);
	glPopMatrix();
	glPushMatrix(),
		glTranslatef(-forearmJointRadius/2,0.0,0);  // move to its start height
		glRotatef(90,0.0,1.0, 0.0); // rotate along base angle
		glutSolidCylinder(forearmJointRadius, forearmJointRadius, 36, 1);
	glPopMatrix();

	// hand
	glRotatef(botAngles[4],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(handRadius, handLength, 36, 1);

	// hand joint
	glTranslatef(0.0,0.0,handLength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(handJointRadius, 36, 36);

	// hand
	glRotatef(botAngles[5],0.0,0.0, 1.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);

	glPushMatrix();
		glRotatef(botAngles[6],0.0,1.0, 0.0); // rotate along base angle
		glutSolidCylinder(gripperLeverRadius, gripperLeverLength, 36, 1);
		glTranslatef(0,0.0,gripperLeverLength);
		glRotatef(-botAngles[6],0.0,1.0, 0.0); // rotate along base angle
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
		glutSolidSphere(gripperRadius, 36, 36);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
		glutSolidCylinder(gripperRadius, gripperLength, 36, 1);
	glPopMatrix();
	glPushMatrix();
		glRotatef(-botAngles[6],0.0,1.0, 0.0); // rotate along base angle
		glutSolidCylinder(gripperLeverRadius, gripperLeverLength, 36, 1);
		glTranslatef(0,0.0,gripperLeverLength);
		glRotatef(botAngles[6],0.0,1.0, 0.0); // rotate along base angle
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
		glutSolidSphere(gripperRadius, 36, 36);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
		glutSolidCylinder(gripperRadius, gripperLength, 36, 1);
	glPopMatrix();

	glPopMatrix();
}

// compute a value floating from start to target during startup time
// (used for eye position to get a neat animation)
float startupFactor(float start, float target) {
	if (startupRatio < 1.0) {
		float myStartupRatio = 0.01;
		if (startupRatio >= 0.3)
			myStartupRatio = (startupRatio-0.3)/0.7;
		float distortedFactor = (1.0-(1.0-myStartupRatio)*(1.0-myStartupRatio));
		float startupFactorAngle = distortedFactor*PI/2.0;
		if (start == 0.0)
			return target*sin(startupFactorAngle);

		return target + (start-target)*cos(startupFactorAngle);
	}
	return target;
}

void setSubWindowBotView(int window) {
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset the model-view matrix

	// Enable perspective projection with fovy, aspect, zNear and zFar
	GLfloat aspectSubWindow = (GLfloat) SubWindowWidth / (GLfloat) SubWindowHeight;
	if (window == wBottomRight)
		aspectSubWindow = (GLfloat) MainSubWindowWidth / (GLfloat) MainSubWindowHeight;

	gluPerspective(45.0f, aspectSubWindow, 0.1f, 5000.0f);

	float startView[] = {-glEyeDistance,glEyeDistance, 0 };
	if (window == wTopLeft) {
		// view from top
		gluLookAt(startupFactor(startView[0],0), startupFactor(startView[1],glEyeDistance) ,startupFactor(startView[2],0),
				  0.0, 0.0, 0.0,
				  1.0, 0.0,	0.0);
	} else if (window == wTopRight) {
		// view from front
		gluLookAt(startupFactor(startView[0],0.0),startupFactor(startView[1],ViewHeight/2), startupFactor(startView[2],glEyeDistance) ,
				  0.0,startupFactor(0,ViewHeight/2), 0.0,
				  0.0, 1.0,	0.0);

	} else if (window == wBottomLeft) {
		// view from side
		gluLookAt(startupFactor(startView[0],-glEyeDistance), startupFactor(startView[1],ViewHeight/2) ,startupFactor(startView[2],0.0),
				  0.0,startupFactor(0,ViewHeight/2), 0.0,
				  0.0, 1.0,0.0);
	} else {
		// view in 3d movable window
		gluLookAt(startupFactor(startView[0], eyePosition[0]),startupFactor(startView[1],eyePosition[1]),startupFactor(startView[2], eyePosition[2]),
				0.0, startupFactor(0,ViewHeight/2), 0.0,
				0.0, 1.0, 0.0);
	}

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();                 // Reset the model-view matrix
	paintBot();
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void drawBotWindowsCallback() {
	glutSetWindow(wMain);
	glClearColor(glMainWindowColor[0], glMainWindowColor[1], glMainWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT);
	printKinematics();

	glutSetWindow(wTopLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printSubWindowTitle("top view");
	setSubWindowBotView(wTopLeft);

	glutSetWindow(wTopRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printSubWindowTitle("front view");
	setSubWindowBotView(wTopRight);

	glutSetWindow(wBottomLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printSubWindowTitle("right side");

	setSubWindowBotView(wBottomLeft);
	glutSetWindow(wBottomRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printSubWindowTitle("3D");
	setSubWindowBotView(wBottomRight);

	glFlush();  // Render now
}

/* Called back when timer expired [NEW] */
void StartupTimerCallback(int value) {
	static uint32_t startupTime_ms = millis();
	uint32_t timeSinceStart_ms = millis()-startupTime_ms;
	if (timeSinceStart_ms < startUpDuration) {
		startupRatio= ((float)(timeSinceStart_ms)/startUpDuration)*PI/2.0;
		// repainting is done in Idle Callback, checking the botModifed flag
		kinematicsHasChanged = true;
		glutTimerFunc(20, StartupTimerCallback, 0);
	}

	// startup procedure is done, done redraw
}

void vis(int visState) {
}

void reshape(int w, int h) {
	WindowWidth = w;
	WindowHeight = h;
	glViewport(0, 0, w, h);

	switch (layoutButtonSelection) {
	case QUAD_LAYOUT: {
		SubWindowWidth = (w -InteractiveWindowWidth - 3 * WindowGap) /2.0;
		MainSubWindowWidth = (w -InteractiveWindowWidth - 3 * WindowGap) / 2.0;
		SubWindowHeight = (h - 3 * WindowGap) /2.0;
		MainSubWindowHeight = (h - 3 * WindowGap) /2.0;

		glutSetWindow(wTopLeft);
		glutShowWindow();
		glutPositionWindow(WindowGap, WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wTopRight);
		glutShowWindow();
		glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wBottomLeft);
		glutShowWindow();
		glutPositionWindow(WindowGap, WindowGap + SubWindowHeight + WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wBottomRight);
		glutShowWindow();
		glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap + SubWindowHeight + WindowGap);
		glutReshapeWindow(MainSubWindowWidth, MainSubWindowHeight);
		glViewport(0, 0, MainSubWindowWidth, MainSubWindowHeight);

		break;
	}
	case MIXED_LAYOUT: {
		SubWindowWidth = (w -InteractiveWindowWidth - 4 * WindowGap) /3.0;
		MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap);
		SubWindowHeight = SubWindowWidth;
		MainSubWindowHeight = h - 2 * WindowGap - SubWindowHeight;

		glutSetWindow(wTopLeft);
		glutShowWindow();
		glutPositionWindow(WindowGap, WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wTopRight);
		glutShowWindow();
		glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wBottomLeft);
		glutShowWindow();
		glutPositionWindow(WindowGap + SubWindowHeight + WindowGap + SubWindowHeight + WindowGap, WindowGap);
		glutReshapeWindow(SubWindowWidth, SubWindowHeight);
		glViewport(0, 0, SubWindowWidth, SubWindowHeight);

		glutSetWindow(wBottomRight);
		glutShowWindow();
		glutPositionWindow(WindowGap, WindowGap + SubWindowHeight + WindowGap );
		glutReshapeWindow(MainSubWindowWidth, MainSubWindowHeight);
		glViewport(0, 0, MainSubWindowWidth, MainSubWindowHeight);

		break;
	}

	case SINGLE_LAYOUT: {
		SubWindowWidth = 0;
		MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap);
		SubWindowHeight = 0;
		MainSubWindowHeight = (h - 2 * WindowGap);

		glutSetWindow(wTopLeft);
		glutHideWindow();
		glutSetWindow(wTopRight);
		glutHideWindow();
		glutSetWindow(wBottomLeft);
		glutHideWindow();
		glutSetWindow(wBottomRight);
		glutShowWindow();
		glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap + SubWindowHeight + WindowGap);
		glutReshapeWindow(MainSubWindowWidth, MainSubWindowHeight);
		glViewport(0, 0, MainSubWindowWidth, MainSubWindowHeight);

		break;
	}
	} // switch
}

void GlutKeyboardCallback(unsigned char Key, int x, int y)
{
	switch(Key)
	{
		case 27:
		case 'q':
			exit(2);
			break;
	};

	glutPostRedisplay();
}

static int leftButtonMouseX,leftButtonMouseY;
static int lastMouseScroll;
static bool leftMouseButton;

void SubWindow3dMotionCallback(int x, int y) {
	if (leftMouseButton) {
		float viewAngle = (float) (x-leftButtonMouseX);
		float heightAngle = (float) (y-leftButtonMouseY);
		currEyeAngle -= viewAngle;
		currEyeHeightAngle -= heightAngle;
	}

	currEyeDistance -= 20*lastMouseScroll;
	lastMouseScroll = 0;
	currEyeDistance = constrain(currEyeDistance,glEyeDistance/3,glEyeDistance*3);
	currEyeHeightAngle = constrain(currEyeHeightAngle,-90.0f,45.0f);

	eyePosition[0] = currEyeDistance*( sin(radians(currEyeAngle)) * cos(radians(currEyeHeightAngle)));
	eyePosition[1] = ViewHeight - currEyeDistance*sin(radians(currEyeHeightAngle));
	eyePosition[2] = currEyeDistance*(cos(radians(currEyeAngle)) * cos(radians(currEyeHeightAngle)));

	if (leftMouseButton) {
		leftButtonMouseX = x;
		leftButtonMouseY = y;
	}

  glutPostRedisplay();
}


void SubWindows3DMouseCallback(int button, int button_state, int x, int y )
{
    leftMouseButton = false;

	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN ) {
	    leftButtonMouseX = x;
	    leftButtonMouseY = y;
	    leftMouseButton = true;
	}

	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) // It's a wheel event
	{
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (button_state != GLUT_UP) { // Disregard redundant GLUT_UP events
			if (button == 3)
				lastMouseScroll++;
			else
				lastMouseScroll--;
			SubWindow3dMotionCallback(x,y);
		}
	}
}

void GluiReshapeCallback( int x, int y )
{
	reshape(x,y);
	int tx, ty, tw, th;
	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );
	glutPostRedisplay();
}

// Idle Call back is used to check, whether anything has changed the
// bots position or view and it needs to be redrawn
void GlutIdleCallback( void )
{
	if ( glutGetWindow() != wMain)
		glutSetWindow(wMain);

	if (kinematicsHasChanged) {
		glutPostRedisplay();
		kinematicsHasChanged = false;
	} else
		delay(25); // otherwise we need 100% cpu, since this is called in a permanent loop
}

void AngleSpinnerCallback( int angleControlNumber )
{
	// spinner values are changed with live variables
	static float lastBotAngles[7];
	float lastValue = lastBotAngles[angleControlNumber];
	float value = botAngles[angleControlNumber];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;

	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;

	lastBotAngles[angleControlNumber] = roundedValue;
	angleSpinner[angleControlNumber]->set_float_val(roundedValue);
	kinematicsHasChanged = true;
	botWindowCtrl.callbackAngles();
}

void TcpSpinnerCallback( int tcpCoordId )
{
	// spinner values are changed with live variables
	static float lastTcp[7];

	float lastValue = lastTcp[tcpCoordId];
	float value = tcp[tcpCoordId];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;

	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;

	lastTcp[tcpCoordId] = roundedValue;
	tcpCoordSpinner[tcpCoordId ]->set_float_val(roundedValue);

	kinematicsHasChanged = true;
	botWindowCtrl.callbackTCP();
}


void layoutButtonCallback(int radioButtonNo) {
	reshape(WindowWidth, WindowHeight);
	glutPostRedisplay();
}


void BotWindowCtrl::callbackAngles() {
	if (anglesCallback != NULL)
		(*anglesCallback)(botAngles);
}

void BotWindowCtrl::callbackTCP() {
	if (tcpCallback != NULL)
		(*tcpCallback)(tcp);
}


int BotWindowCtrl::createBotSubWindow(int mainWindow) {
	int windowHandle = glutCreateSubWindow(mainWindow, WindowGap + SubWindowWidth + WindowGap,
						WindowGap + SubWindowHeight + WindowGap, SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(drawBotWindowsCallback);
	glutKeyboardFunc(GlutKeyboardCallback);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   							// Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    							// Set the type of depth-test
	glShadeModel(GL_SMOOTH);   							// Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); 	// Nice perspective corrections

	setLights();
	return windowHandle;
 }

GLUI* BotWindowCtrl::createInteractiveWindow(int mainWindow) {
	GLUI *windowHandle= GLUI_Master.create_glui_subwindow( wMain,  GLUI_SUBWINDOW_RIGHT );
	windowHandle->set_main_gfx_window( wMain );

	anglesPanel = new GLUI_Panel(windowHandle,"Kinematics", GLUI_PANEL_EMBOSSED);
	anglesPanel->set_alignment(GLUI_ALIGN_RIGHT);
	new GLUI_StaticText(anglesPanel,"                                    ");


	for (int i = 0;i<7;i++) {
		angleSpinner[i] = new GLUI_Spinner(anglesPanel,angleName[i].c_str(), GLUI_SPINNER_FLOAT,&botAngles[i],i, AngleSpinnerCallback);
	}
	angleSpinner[HIP]->set_float_limits(-180,180);
	angleSpinner[UPPERARM]->set_float_limits(-90,90);
	angleSpinner[FOREARM]->set_float_limits(-270,90);
	angleSpinner[ELLBOW]->set_float_limits(-180,180);
	angleSpinner[WRIST]->set_float_limits(-180,180);
	angleSpinner[HAND]->set_float_limits(-180,180);
	angleSpinner[GRIPPER]->set_float_limits(12,60);

	new GLUI_StaticText(anglesPanel,"");
	string coordName[3] = {"x","y","z" };
	for (int i = 0;i<3;i++) {
		tcpCoordSpinner[i]= new GLUI_Spinner(anglesPanel,coordName[i].c_str(), GLUI_SPINNER_FLOAT,&tcp[i],i, TcpSpinnerCallback);
	}
	tcpCoordSpinner[X]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Y]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Z]->set_float_limits(0,1000);

	GLUI_Panel* layoutPanel = new GLUI_Panel(windowHandle,"Layout", GLUI_PANEL_EMBOSSED);
	layoutPanel->set_alignment(GLUI_ALIGN_RIGHT);
	GLUI_RadioGroup *layoutRadioGroup= new GLUI_RadioGroup( layoutPanel,&layoutButtonSelection,4, layoutButtonCallback);
	new GLUI_StaticText(layoutRadioGroup,"                                    ");
	new GLUI_RadioButton( layoutRadioGroup, "single view" );
	new GLUI_RadioButton( layoutRadioGroup, "all side view" );
	new GLUI_RadioButton( layoutRadioGroup, "mixed view" );

	layoutRadioGroup->set_int_val(QUAD_LAYOUT);
	return windowHandle;
}


bool BotWindowCtrl::setup(int argc, char** argv) {
	glutInit(&argc, argv);

	eventLoopThread = new std::thread(&BotWindowCtrl::eventLoop, this);
	// wait until UI is ready (excluding the startup animation)
	unsigned long startTime  = millis();
	do { delay(10); }
	while ((millis() - startTime < 1000) && (!uiReady));

	return uiReady;
}

void BotWindowCtrl::eventLoop() {
	glutInitWindowSize(WindowWidth, WindowHeight);
    wMain = glutCreateWindow("Bad Robot"); // Create a window with the given title
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glutReshapeFunc(reshape);

	GLUI_Master.set_glutMouseFunc( SubWindows3DMouseCallback );
	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( GlutIdleCallback);

	wTopLeft = createBotSubWindow(wMain);
	wTopRight = createBotSubWindow(wMain);
	wBottomLeft = createBotSubWindow(wMain);
	wBottomRight = createBotSubWindow(wMain);

	wInteractive = createInteractiveWindow(wMain);
	glutSetWindow(wMain);
	GLUI_Master.set_glutKeyboardFunc( GlutKeyboardCallback );

	// 3D view can be rotated with mouse
	glutSetWindow(wBottomRight);
	glutMotionFunc( SubWindow3dMotionCallback);
	glutMouseFunc( SubWindows3DMouseCallback);

	glutTimerFunc(0, StartupTimerCallback, 0);	// timer that sets the view point of startup procedure

	uiReady = true; 							// stop waiting for ui initialization
	glutMainLoop();  							// Enter the infinitely event-processing loop
}


void BotWindowCtrl::setAngles(float pAngles[], float pTcp[]) {
	for (int i = 0;i<7;i++)
		botAngles[i] = pAngles[i];
	for (int i = 0;i<3;i++)
		tcp[i] = pTcp[i];
	kinematicsHasChanged = true; // redraw
}
void BotWindowCtrl::setAnglesCallback(void (* callback)( float[])) {
	anglesCallback = callback;
}
void BotWindowCtrl::setTcpCallback(void (* callback)( float[])) {
	tcpCallback = callback;

}

