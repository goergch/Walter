/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include "BotWindowCtrl.h"
#include "Util.h"
#include "Kinematics.h"
#include "BotView.h"

using namespace std;

BotWindowCtrl botWindowCtrl;

int WindowWidth = 800;				// initial window size
int WindowHeight = 600;

int WindowGap=10;					// gap between subwindows
int SubWindowHeight = 10;			// initial height of a subwindow
int SubWindowWidth = 10;			// initial weight of a subwindow
int MainSubWindowHeight = 10;		// initial height of a subwindow
int MainSubWindowWidth = 10;		// initial weight of a subwindow
int InteractiveWindowWidth=250;		// initial width of the interactive window

// layout, we can do quad layout or single layout
enum LayoutType { SINGLE_LAYOUT = 0, QUAD_LAYOUT = 1, MIXED_LAYOUT=2 };
int layoutButtonSelection=QUAD_LAYOUT;		// live variable of radio group

static GLfloat glMainWindowColor[] 		= {1.0,1.0,1.0};
static GLfloat glSubWindowColor[] 		= {0.97,0.97,0.97};

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

GLUI_Spinner* angleSpinner[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
GLUI_Spinner* tcpCoordSpinner[] = {NULL,NULL,NULL, NULL, NULL, NULL};
float tcpSpinnerLiveVar[] = {0,0,0,0,0,0,0};
GLUI_Button* layoutButton = NULL;

float anglesLiveVar[7] = {0.0,0.0,0.0,0.0,0.0,0.0,30.0 };
JointAngleType angles = {0,0,0,0,0,0,30.0};

Pose tcp;										// current pose of the tool centre point
bool kinematicsHasChanged = false; 				// true, if something in kinematics has changed

KinematicConfigurationType currConfig;
int configDirectionLiveVar= 0;					// kinematics configuration, bot looks to the front or to the back
int configFlipLiveVar = 0;						// kinematics triangle flip
int configTurnLiveVar = 0;						// kinematics forearm flip


BotView botView;


void printKinematicsValuesInSubWindow() {
	static float lastAngle[7];
	for (int i = 0;i<7;i++) {
		float value = angles[i];
		if (value != lastAngle[i]) {
			angleSpinner[i]->set_float_val(value); // set only when necessary, otherwise the cursor blinks
			lastAngle[i] = anglesLiveVar[i];
		}
	}

	static float lastTcp[6];
	for (int i = 0;i<6;i++) {
		float value = (i<3)?tcp.position[i]:tcp.orientation[i-3];
		if (value != lastTcp[i]) {
			tcpCoordSpinner[i]->set_float_val(value); // set only when necessary, otherwise the cursor blinks
			lastTcp[i] = value;
		}
	}
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

void setSubWindowPerspective(int window) {
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
	botView.paintBot(angles);
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {
	glutSetWindow(wMain);
	glClearColor(glMainWindowColor[0], glMainWindowColor[1], glMainWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	printKinematicsValuesInSubWindow();

	glutSetWindow(wTopLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	botView.printSubWindowTitle("top view");
	setSubWindowPerspective(wTopLeft);

	glutSetWindow(wTopRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	botView.printSubWindowTitle("front view");
	setSubWindowPerspective(wTopRight);

	glutSetWindow(wBottomLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	botView.printSubWindowTitle("right side");
	setSubWindowPerspective(wBottomLeft);

	glutSetWindow(wBottomRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	botView.printSubWindowTitle("3D");
	setSubWindowPerspective(wBottomRight);

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
		MainSubWindowHeight = h - 3 * WindowGap - SubWindowHeight;

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
void idleCallback( void )
{
	if ( glutGetWindow() != wMain)
		glutSetWindow(wMain);

	if (kinematicsHasChanged) {
		glutPostRedisplay();
		kinematicsHasChanged = false;
	} else
		delay(25); // otherwise we needed 100% cpu, since idle callback is called in an infinite loop by glut
}

void angleSpinnerCallback( int angleControlNumber )
{
	// spinner values are changed with live variables
	static float lastSpinnerVal[7];
	float lastValue = lastSpinnerVal[angleControlNumber];
	float value = anglesLiveVar[angleControlNumber];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;

	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;

	lastSpinnerVal[angleControlNumber] = roundedValue;
	angleSpinner[angleControlNumber]->set_float_val(roundedValue);

	// copy live variables to main variable holding angles
	for (int i = 0;i<NumberOfActuators;i++)
		angles[i] = anglesLiveVar[i];

	// since angles have changed recompute kinematics. Call callback

	kinematicsHasChanged = true;
}

void TCPSpinnerCallback( int tcpCoordId )
{
	// spinner values are changed with live variables
	static float lastSpinnerValue[6];

	float lastValue = lastSpinnerValue[tcpCoordId];
	float value =tcpSpinnerLiveVar[tcpCoordId];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;

	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;

	lastSpinnerValue[tcpCoordId] = roundedValue;
	tcpCoordSpinner[tcpCoordId ]->set_float_val(roundedValue);

	// copy GLUI live vars into tcp
	if (tcpCoordId < 3)
		tcp.position[tcpCoordId] = tcpSpinnerLiveVar[tcpCoordId];
	else
		tcp.orientation[tcpCoordId] = tcpSpinnerLiveVar[tcpCoordId];

	// compute angles out of tcp pose
	botWindowCtrl.callbackChangedTCP();

	kinematicsHasChanged = true;
}

void BotWindowCtrl::callbackChangedTCP() {
	if (tcpCallback != NULL) {
		(*tcpCallback)(tcp, currConfig, angles);
	}
}

void BotWindowCtrl::callbackChangedAngles() {
	if (anglesCallback != NULL) {
		(*anglesCallback)(angles, tcp, currConfig);
	}
}

void layoutButtonCallback(int radioButtonNo) {
	reshape(WindowWidth, WindowHeight);
	glutPostRedisplay();
}


int BotWindowCtrl::createBotSubWindow(int mainWindow) {
	int windowHandle = glutCreateSubWindow(mainWindow, WindowGap + SubWindowWidth + WindowGap,
						WindowGap + SubWindowHeight + WindowGap, SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(display);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   							// Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    							// Set the type of depth-test
	glShadeModel(GL_SMOOTH);   							// Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); 	// Nice perspective corrections

	botView.setLights();
	return windowHandle;
 }

void PoseKonfigurationCallback(int ControlNo) {
	currConfig.poseDirection = (configDirectionLiveVar==0)?KinematicConfigurationType::FRONT:KinematicConfigurationType::BACK;
	currConfig.poseFlip = (configFlipLiveVar==0)?KinematicConfigurationType::FLIP:KinematicConfigurationType::NO_FLIP;
	currConfig.poseTurn = (configTurnLiveVar==0)?KinematicConfigurationType::UP:KinematicConfigurationType::DOWN;
}

GLUI* BotWindowCtrl::createInteractiveWindow(int mainWindow) {

	string emptyLine = "                                               ";

	GLUI *windowHandle= GLUI_Master.create_glui_subwindow( wMain,  GLUI_SUBWINDOW_RIGHT );
	windowHandle->set_main_gfx_window( wMain );

	GLUI_Panel* kinematicsPanel = new GLUI_Panel(windowHandle,"Kinematics", GLUI_PANEL_EMBOSSED);
	kinematicsPanel->set_alignment(GLUI_ALIGN_RIGHT);

	GLUI_Panel* AnglesPanel= new GLUI_Panel(kinematicsPanel,"Angles", GLUI_PANEL_RAISED);

	string angleName[] = { "hip"," upperarm","forearm","ellbow", "wrist", "hand", "gripper" };
	for (int i = 0;i<7;i++) {
		angleSpinner[i] = new GLUI_Spinner(AnglesPanel,angleName[i].c_str(), GLUI_SPINNER_FLOAT,&anglesLiveVar[i],i, angleSpinnerCallback);
	}
	angleSpinner[HIP]->set_float_limits(-180,180);
	angleSpinner[UPPERARM]->set_float_limits(-90,90);
	angleSpinner[FOREARM]->set_float_limits(-270,90);
	angleSpinner[ELLBOW]->set_float_limits(-180,180);
	angleSpinner[WRIST]->set_float_limits(-180,180);
	angleSpinner[HAND]->set_float_limits(-180,180);
	angleSpinner[GRIPPER]->set_float_limits(12,60);

	GLUI_Panel* TCPPanel= new GLUI_Panel(kinematicsPanel,"TCP", GLUI_PANEL_RAISED);
	string coordName[3] = {"x","y","z" };
	for (int i = 0;i<3;i++) {
		tcpCoordSpinner[i]= new GLUI_Spinner(TCPPanel,coordName[i].c_str(), GLUI_SPINNER_FLOAT,&tcpSpinnerLiveVar[i],i, TCPSpinnerCallback);
	}
	GLUI_Panel* PosePanel= new GLUI_Panel(kinematicsPanel,"Pose", GLUI_PANEL_RAISED);
	string rotName[3] = {"nick","yaw","roll" };
	for (int i = 0;i<3;i++) {
		tcpCoordSpinner[i+3]= new GLUI_Spinner(PosePanel,rotName[i].c_str(), GLUI_SPINNER_FLOAT,&tcpSpinnerLiveVar[i+3],i+3, TCPSpinnerCallback);
	}

	tcpCoordSpinner[X]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Y]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Z]->set_float_limits(0,1000);

	tcpCoordSpinner[3]->set_float_limits(-180, 180);
	tcpCoordSpinner[4]->set_float_limits(-180, 180);
	tcpCoordSpinner[5]->set_float_limits(-180, 180);

	GLUI_Panel* frontBackPanel= new GLUI_Panel(kinematicsPanel,"Configuration", GLUI_PANEL_RAISED);
	GLUI_RadioGroup *frontBackRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configDirectionLiveVar, 0, PoseKonfigurationCallback);
	new GLUI_RadioButton( frontBackRadioGroup,"front");
	new GLUI_RadioButton( frontBackRadioGroup, "back");
	frontBackRadioGroup->set_int_val(configDirectionLiveVar);
	windowHandle->add_column_to_panel(frontBackPanel, true);
	GLUI_RadioGroup *poseFlipRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configFlipLiveVar, 1, PoseKonfigurationCallback);
	new GLUI_RadioButton( poseFlipRadioGroup, "flip");
	new GLUI_RadioButton( poseFlipRadioGroup, "reg");
	poseFlipRadioGroup->set_int_val(configFlipLiveVar);
	windowHandle->add_column_to_panel(frontBackPanel, true);
	GLUI_RadioGroup *PoseForearmRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configTurnLiveVar, 2, PoseKonfigurationCallback);
	new GLUI_RadioButton( PoseForearmRadioGroup, "up");
	new GLUI_RadioButton( PoseForearmRadioGroup, "dn");
	PoseForearmRadioGroup->set_int_val(configTurnLiveVar);;

	GLUI_Panel* layoutPanel = new GLUI_Panel(windowHandle,"Layout", GLUI_PANEL_EMBOSSED);
	layoutPanel->set_alignment(GLUI_ALIGN_RIGHT);
	GLUI_RadioGroup *layoutRadioGroup= new GLUI_RadioGroup( layoutPanel,&layoutButtonSelection,4, layoutButtonCallback);
	new GLUI_StaticText(layoutRadioGroup,emptyLine.c_str());
	new GLUI_RadioButton( layoutRadioGroup, "single view" );
	new GLUI_RadioButton( layoutRadioGroup, "all side view" );
	new GLUI_RadioButton( layoutRadioGroup, "mixed view" );

	layoutRadioGroup->set_int_val(QUAD_LAYOUT);
	return windowHandle;
}


bool BotWindowCtrl::setup(int argc, char** argv) {
	glutInit(&argc, argv);

	// start the initialization in a thread so that this function returns
	// ( the thread runs the endless GLUT main loop)
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
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	GLUI_Master.set_glutMouseFunc( SubWindows3DMouseCallback );
	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( idleCallback);

	wTopLeft = createBotSubWindow(wMain);
	wTopRight = createBotSubWindow(wMain);
	wBottomLeft = createBotSubWindow(wMain);
	wBottomRight = createBotSubWindow(wMain);

	// 3D view can be rotated with mouse
	glutSetWindow(wBottomRight);
	glutMotionFunc( SubWindow3dMotionCallback);
	glutMouseFunc( SubWindows3DMouseCallback);

	createInteractiveWindow(wMain);
	glutSetWindow(wMain);

	glutTimerFunc(0, StartupTimerCallback, 0);	// timer that sets the view point of startup procedure

	uiReady = true; 							// stop waiting for ui initialization
	glutMainLoop();  							// Enter the infinitely event-processing loop
}



// set callback invoked whenever an angle is changed via ui
void BotWindowCtrl::setAnglesCallback(void (* callback)( JointAngleType angles, Pose &pose, KinematicConfigurationType &config)) {
	anglesCallback = callback;
}

// set callback invoked whenever the tcp or configuration is changed via ui
void BotWindowCtrl::setTcpInputCallback(void (* callback)( Pose pose, KinematicConfigurationType config, JointAngleType &angles)) {
	tcpCallback = callback;
}

