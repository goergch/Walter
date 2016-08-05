/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include <windows.h>  // For MS Windows
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GL/Glui.h>

#include "Util.h"
#include "BotWindowCtrl.h"

using namespace std;

BotWindowCtrl botWindowCtrl;

// Window size
int WindowWidth = 800;				// initial window size
int WindowHeight = 600;

int WindowGap=10;					// gap between subwindows
int SubWindowHeight = 10;			// initial height of a subwindow
int SubWindowWidth = 10;			// initial weight of a subwindow
int InteractiveWindowWidth=220;		// initial width of the interactive window

static GLfloat glMainWindowColor[] 		= {1.0,1.0,1.0};
static GLfloat glSubWindowColor[] 		= {0.97,0.97,0.97};
static GLfloat glBotArmColor[] 			= { 1.0f, 0.3f, 0.2f };
static GLfloat glBotJointColor[] 		= { 0.5f, 0.6f, 0.6f };
static GLfloat glWindowTitleColor[] 	= { 1.0f, 1.0f, 1.0f };
static GLfloat glCoordSystemColor4v[] 	= { 0.33f, 0.37f, 0.42f,0.5f };

// 3d moving window eye position
const float glEyeDistance = 1500.0f;	// distance of the eye to the bot
const float ViewHeight = 800.0f;		// height of the bot to be viewed
float currEyeAngle= -45;					// current eye position of moveable subwindow
float currEyeDistance = glEyeDistance;	// current eye distance of moveable subwindow
float eyePosition[3] = {currEyeDistance*sinf(radians(currEyeAngle)),ViewHeight,currEyeDistance*cosf(radians(currEyeAngle))};
float startUpDuration = 5000;			// duration of startup animation
float startupRatio= 0.0; 				// between 0 and 1, indicates the position within the startup animation

// handles of opengl windows and subwindows
int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;	// window handler of windows
GLUI *wInteractive = NULL;				// interactive window handler

GLUI_Panel* anglesPanel = NULL;
GLUI_Spinner* angleSpinner[] = {NULL,NULL,NULL,NULL,NULL,NULL};
GLUI_Spinner* tcpCoordSpinner[] = {NULL,NULL,NULL};

float botAngles[6] = {0.0,0.0,0.0,0.0,0.0,0.0 };
string angleName[] = { "hip","shoulder","forearm","ellbow","upperarm", "wrist" };

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
	static float lastBotAngle[6];
	for (int i = 0;i<6;i++) {
		float spinnerValue = ((int)(botAngles[i]*10.0))/10.0f;
		if (spinnerValue != lastBotAngle[i]) {
			angleSpinner[i]->set_float_val(spinnerValue);
			lastBotAngle[i] = spinnerValue;
		}
	}
	static float lastTcp[6];

	for (int i = 0;i<3;i++) {
		float spinnerValue = ((int)(tcp[i]*10.0))/10.0f;
		if (spinnerValue != lastTcp[i]) {
			tcpCoordSpinner[i]->set_float_val(spinnerValue);
			lastTcp[i] = spinnerValue;
		}
	}

}

void drawCoordSystem() {
	// draw coordinate system
	const float axisLength = 500.0f;
	const float arrowLength = 20.0f;
	const float unitLength = 100.0f;
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
	const float baseHeight= 110;

	const float armlength = 240;
	const float armJointRadius= 30;
	const float armradius = 20;

	const float wristLength= 90;
	const float handJointRadius= 18;
	const float handRadius= 12;

	const float gripperLength= 70;
	const float gripperRadius=10;

	const float gripperLeverLength= 45;
	const float gripperLeverRadius=5;
	const float gripperAngle= 30;

	glMatrixMode(GL_MODELVIEW);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1],glSubWindowColor[2],0.0f); // Set background color to white and opaque

	// coord system
	drawCoordSystem();

	// base plate
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidCylinder(baseplateRadius, baseplateHeight, 36, 1);
	// glPopMatrix();

	// shoulder
	glTranslatef(0.0, 0.0,baseplateHeight);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(armradius, baseHeight, 36, 1);

	// shoulder joint
	glTranslatef(0.0,0.0,baseHeight);  // Move right and into the screen
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(armJointRadius, 36, 36);

	// upperarm
	glRotatef(botAngles[0],0.0,0.0, 1.0); // turn along angle
	glRotatef(botAngles[1],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(armradius, armlength, 36, 1);

	// upperarm joint
	glTranslatef(0.0,0.0,armlength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(armJointRadius, 36, 36);

	// forearm
	glRotatef(90+botAngles[2],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(armradius, armlength, 36, 1);

	// forearm joint
	glRotatef(botAngles[3],0.0,0.0, 1.0); // rotate along base angle
	glTranslatef(0.0,0.0,armlength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(armJointRadius, 36, 36);

	// wrist
	glRotatef(botAngles[4],1.0,0.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	glutSolidCylinder(handRadius, wristLength, 36, 1);

	// wrist joint
	glTranslatef(0.0,0.0,wristLength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
	glutSolidSphere(handJointRadius, 36, 36);

	// hand
	glRotatef(botAngles[5],0.0,0.0, 1.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
	// glutSolidCylinder(5, 90, 36, 1);
	// glTranslatef(0,0.0,90);

	glPushMatrix();
		glRotatef(gripperAngle,0.0,1.0, 0.0); // rotate along base angle
		glutSolidCylinder(gripperLeverRadius, gripperLeverLength, 36, 1);
		glTranslatef(0,0.0,gripperLeverLength);
		glRotatef(-gripperAngle,0.0,1.0, 0.0); // rotate along base angle
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotJointColor);
		glutSolidSphere(gripperRadius, 36, 36);
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotArmColor);
		glutSolidCylinder(gripperRadius, gripperLength, 36, 1);
	glPopMatrix();
	glPushMatrix();
		glRotatef(-gripperAngle,0.0,1.0, 0.0); // rotate along base angle
		glutSolidCylinder(gripperLeverRadius, gripperLeverLength, 36, 1);
		glTranslatef(0,0.0,gripperLeverLength);
		glRotatef(gripperAngle,0.0,1.0, 0.0); // rotate along base angle
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
	glLoadIdentity();                 // Reset the model-view matrix

	// Enable perspective projection with fovy, aspect, zNear and zFar
	GLfloat aspectSubWindow = (GLfloat) SubWindowWidth / (GLfloat) SubWindowHeight;

	gluPerspective(45.0f, aspectSubWindow, 0.1f, 5000.0f);
	static float par = 0.0;
	par += 0.02;

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
	printSubWindowTitle("side view");

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
	if (w > 50) {
		SubWindowWidth = (w -InteractiveWindowWidth - 3 * WindowGap) /2;
	} else {
		SubWindowWidth = WindowGap;
	}
	if (h > 50) {
		SubWindowHeight = (h - 3 * WindowGap) / 2;
	} else {
		SubWindowHeight = WindowGap;
	}

	if (SubWindowHeight == 0)
		SubWindowHeight = 1;                // To prevent divide by 0

	glutSetWindow(wTopLeft);
	glutPositionWindow(WindowGap, WindowGap);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

	glutSetWindow(wTopRight);
	glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

	glutSetWindow(wBottomLeft);
	glutPositionWindow(WindowGap, WindowGap + SubWindowHeight + WindowGap);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

	glutSetWindow(wBottomRight);
	glutPositionWindow(WindowGap + SubWindowWidth + WindowGap, WindowGap + SubWindowHeight + WindowGap);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);
}

void GlutKeyboardCallback(unsigned char Key, int x, int y)
{
	switch(Key)
	{
		case 27:
		case 'q':
			exit(0);
			break;
	};

	glutPostRedisplay();
}

static 	int lastMouseX,lastMouseY;

void SubWindows3DMouseCallback(int button, int button_state, int x, int y )
{
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN ) {
	    lastMouseX = x;
	    lastMouseY = y;
	}
}

void SubWindow3dMotionCallback(int x, int y) {
	float rotY = (float) (y - lastMouseY);
	float rotX = (float) (x - lastMouseX);

	currEyeDistance += 5*rotY;
	currEyeDistance = constrain(currEyeDistance,glEyeDistance/3,glEyeDistance*3);
	currEyeAngle -= rotX;
	eyePosition[0] = currEyeDistance*sin(radians(currEyeAngle));
	eyePosition[1] = ViewHeight;
	eyePosition[2] = currEyeDistance*cos(radians(currEyeAngle));

  lastMouseX = x;
  lastMouseY = y;

  glutPostRedisplay();
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
	if (kinematicsHasChanged) {
		if ( glutGetWindow() != wMain)
			glutSetWindow(wMain);

		glutPostRedisplay();
		kinematicsHasChanged = false;
	} else
		delay(25);
}

void AngleSpinnerCallback( int angleControlNumber )
{
	// float spinnerValue = ((int)(botAngles[angleControlNumber]*10.0))/10.0f;
	kinematicsHasChanged = true;
}

void TcpSpinnerCallback( int tcpCoordId )
{
	// float spinnerValue = ((int)(tcp[tcpCoordId]*10.0))/10.0f;
	kinematicsHasChanged = true;
}


void BotWindowCtrl::startBotUI(int argc, char** argv) {

	glutInit(&argc, argv);
	glutInitWindowSize(WindowWidth, WindowHeight);
	wMain = glutCreateWindow("Bad Robot"); // Create a window with the given title
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glutReshapeFunc(reshape);

	GLUI_Master.set_glutKeyboardFunc( GlutKeyboardCallback );
	GLUI_Master.set_glutMouseFunc( SubWindows3DMouseCallback );
	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( GlutIdleCallback);

	GLUI *wInteractive= GLUI_Master.create_glui_subwindow( wMain,  GLUI_SUBWINDOW_RIGHT );

	// GLUI_Master.set_main_gfx_window( wMain );
	anglesPanel = new GLUI_Panel(wInteractive,"Kinematics", GLUI_PANEL_EMBOSSED);

	for (int i = 0;i<6;i++) {
		angleSpinner[i] = new GLUI_Spinner(anglesPanel,angleName[i].c_str(), GLUI_SPINNER_FLOAT,&botAngles[i],i, AngleSpinnerCallback);
	}
	angleSpinner[0]->set_float_limits(-180,180);
	angleSpinner[1]->set_float_limits(-180,180);
	angleSpinner[2]->set_float_limits(-270,90);
	angleSpinner[3]->set_float_limits(-180,180);
	angleSpinner[4]->set_float_limits(-180,180);
	angleSpinner[5]->set_float_limits(-180,180);

	new GLUI_StaticText(anglesPanel,"");
	for (int i = 0;i<3;i++) {
		string coordName[3] = {"x","y","z" };
		tcpCoordSpinner[i]= new GLUI_Spinner(anglesPanel,coordName[i].c_str(), GLUI_SPINNER_FLOAT,&tcp[i],i, TcpSpinnerCallback);
	}
	tcpCoordSpinner[0]->set_float_limits(-1000,1000);
	tcpCoordSpinner[1]->set_float_limits(-1000,1000);
	tcpCoordSpinner[2]->set_float_limits(0,1000);


	wTopLeft = glutCreateSubWindow(wMain, WindowGap, WindowGap, SubWindowWidth,SubWindowHeight);
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	setLights();

	wTopRight = glutCreateSubWindow(wMain, WindowGap + SubWindowWidth + WindowGap, WindowGap,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	setLights();

	wBottomLeft = glutCreateSubWindow(wMain, WindowGap, WindowGap + SubWindowHeight + WindowGap,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	setLights();

	wBottomRight = glutCreateSubWindow(wMain, WindowGap + SubWindowWidth + WindowGap,
					WindowGap + SubWindowHeight + WindowGap, SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(drawBotWindowsCallback);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   	// Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    	// Set the type of depth-test
	glShadeModel(GL_SMOOTH);   	// Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	setLights();
	glutMotionFunc( SubWindow3dMotionCallback);
	glutMouseFunc( SubWindows3DMouseCallback);

	glutTimerFunc(0, StartupTimerCallback, 0);
	glutMainLoop();  // Enter the infinitely event-processing loop
}

