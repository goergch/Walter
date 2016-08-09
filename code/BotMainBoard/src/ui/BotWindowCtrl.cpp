/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include "BotWindowCtrl.h"
#include "Util.h"
#include "Kinematics.h"
#include "BotView.h"
#include "MainBotController.h"

using namespace std;

BotWindowCtrl botWindowCtrl;

int WindowWidth = 800;						// initial window size
int WindowHeight = 600;

int WindowGap=10;							// gap between subwindows
int InteractiveWindowWidth=250;				// initial width of the interactive window

enum LayoutType { SINGLE_LAYOUT = 0, QUAD_LAYOUT = 1, MIXED_LAYOUT=2 };// layout type for bot view
int layoutButtonSelection=QUAD_LAYOUT;		// live variable of radio group

static GLfloat glMainWindowColor[] 		= {1.0,1.0,1.0};

// startup animation
float startUpDuration = 5000;				// duration of startup animation

// handles of opengl windows and subwindows
int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;	// window handler of windows

// kinematics widget
GLUI_Spinner* tcpCoordSpinner[] = {NULL,NULL,NULL, NULL, NULL, NULL, NULL};
float tcpSpinnerLiveVar[] = {0,0,0,0,0,0,0};
GLUI_Spinner* angleSpinner[] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
float anglesLiveVar[7] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

bool kinematicsHasChanged = false; 				// true, if something in kinematics has changed


// configuration widget
std::vector<KinematicConfigurationType> validConfigurations;
KinematicConfigurationType currConfig;
GLUI_RadioGroup *frontBackRadioGroup= NULL;
GLUI_RadioGroup *poseFlipRadioGroup= NULL;
GLUI_RadioGroup *poseForearmRadioGroup= NULL;
int configDirectionLiveVar= 0;					// kinematics configuration, bot looks to the front or to the back
int configFlipLiveVar = 0;						// kinematics triangle flip
int configTurnLiveVar = 0;						// kinematics forearm flip

void copyAnglesToView() {
	static float lastAngle[NumberOfActuators];
	for (int i = 0;i<NumberOfActuators;i++) {
		float value = degrees(MainBotController::getInstance().getCurrentAngles()[i]);
		value = sgn(value)*((float)((int)(abs(value)*10.0+0.5)))/10.0;
		if (value != lastAngle[i]) {
			angleSpinner[i]->set_float_val(value); // set only when necessary, otherwise the cursor blinks
			lastAngle[i] = value;
		}
	}

	botWindowCtrl.topLeft.setAngles(MainBotController::getInstance().getCurrentAngles());
	botWindowCtrl.topRight.setAngles(MainBotController::getInstance().getCurrentAngles());
	botWindowCtrl.bottomLeft.setAngles(MainBotController::getInstance().getCurrentAngles());
	botWindowCtrl.bottomRight.setAngles(MainBotController::getInstance().getCurrentAngles());
}

JointAngleType getAnglesView() {
	JointAngleType angles = {0,0,0,0,0,0,0};
	for (int i = 0;i<NumberOfActuators;i++) {
		angles[i] = radians(anglesLiveVar[i]);
	}
	return angles;
}



void copyPoseToView() {
	static float lastTcp[Actuators];
	const Pose& tcp = MainBotController::getInstance().getCurrentTCP();
	for (int i = 0;i<Actuators;i++) {
		rational value;
		if (i<3)
			value = tcp.position[i];
		else
			value = degrees(tcp.orientation[i-3]);

		value = sgn(value)*((float)((int)(abs(value)*10.0+0.5)))/10.0;
		if (value != lastTcp[i]) {
			tcpCoordSpinner[i]->set_float_val(value); // set only when necessary, otherwise the cursor blinks
			lastTcp[i] = value;
		}
	}
	angleSpinner[GRIPPER]->set_float_val(degrees(tcp.gripperAngle));
}

Pose getPoseView() {
	Pose tcp;
	for (int i = 0;i<Actuators;i++) {
		rational value = tcpSpinnerLiveVar[i];
		if (i<3)
			tcp.position[i] = value;
		else
			tcp.orientation[i-3] = radians(value);
	}
	tcp.gripperAngle = radians(anglesLiveVar[GRIPPER]);

	return tcp;
}

KinematicConfigurationType getConfigurationView() {
	KinematicConfigurationType config;
	config.poseDirection = (KinematicConfigurationType::PoseDirectionType)poseFlipRadioGroup->get_int_val();
	config.poseFlip = (KinematicConfigurationType::PoseFlipType)poseFlipRadioGroup->get_int_val();
	config.poseTurn= (KinematicConfigurationType::PoseForearmType)poseForearmRadioGroup->get_int_val();
	return config;
}


void copyConfigurationToView() {
	KinematicConfigurationType config = MainBotController::getInstance().getCurrentConfiguration();
	frontBackRadioGroup->set_int_val(config.poseDirection);
	poseFlipRadioGroup->set_int_val(config.poseFlip);
	poseForearmRadioGroup->set_int_val(config.poseTurn);
	frontBackRadioGroup->disable();
	poseFlipRadioGroup->disable();
	poseForearmRadioGroup->disable();

	std::vector<KinematicConfigurationType> validConfigs = MainBotController::getInstance().getValidConfiguration();
	for (unsigned int i = 0;i<validConfigs.size();i--) {
		KinematicConfigurationType config = validConfigs[i];
		if (config.poseDirection != config.poseDirection)
			frontBackRadioGroup->enable();
		if (config.poseFlip != config.poseFlip)
			poseFlipRadioGroup->enable();
		if (config.poseTurn != config.poseTurn)
			poseForearmRadioGroup->enable();
	}
}


/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {
	glutSetWindow(wMain);
	glClearColor(glMainWindowColor[0], glMainWindowColor[1], glMainWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	botWindowCtrl.topLeft.display();
	botWindowCtrl.topRight.display();
	botWindowCtrl.bottomLeft.display();
	botWindowCtrl.bottomRight.display();

	glFlush();  // Render now
}

/* Called back when timer expired [NEW] */
void StartupTimerCallback(int value) {
	static uint32_t startupTime_ms = millis();
	uint32_t timeSinceStart_ms = millis()-startupTime_ms;
	if (timeSinceStart_ms < startUpDuration) {
		float startupRatio= ((float)(timeSinceStart_ms)/startUpDuration)*PI/2.0;
		botWindowCtrl.topLeft.setStartupAnimationRatio(startupRatio);
		botWindowCtrl.topRight.setStartupAnimationRatio(startupRatio);
		botWindowCtrl.bottomLeft.setStartupAnimationRatio(startupRatio);
		botWindowCtrl.bottomRight.setStartupAnimationRatio(startupRatio);

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
		int SubWindowWidth = (w -InteractiveWindowWidth - 3 * WindowGap) /2.0;
		int SubWindowHeight = (h - 3 * WindowGap) /2.0;

		botWindowCtrl.topLeft.reshape(WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.topRight.reshape(WindowGap + SubWindowWidth + WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.bottomLeft.reshape(WindowGap, WindowGap + SubWindowHeight + WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.bottomRight.reshape(WindowGap + SubWindowWidth + WindowGap, WindowGap + SubWindowHeight + WindowGap,SubWindowWidth, SubWindowHeight);

		break;
	}
	case MIXED_LAYOUT: {
		int SubWindowWidth = (w -InteractiveWindowWidth - 4 * WindowGap) /3.0;
		int SubWindowHeight = SubWindowWidth;
		int MainSubWindowHeight = h - 3 * WindowGap - SubWindowHeight;
		int MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap);

		botWindowCtrl.topLeft.reshape(WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.topRight.reshape(WindowGap + SubWindowWidth + WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.bottomLeft.reshape(WindowGap + SubWindowHeight + WindowGap + SubWindowHeight + WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
		botWindowCtrl.bottomRight.reshape(WindowGap, WindowGap + SubWindowHeight + WindowGap,MainSubWindowWidth, MainSubWindowHeight);
		break;
	}

	case SINGLE_LAYOUT: {
		int MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap);
		int MainSubWindowHeight = (h - 2 * WindowGap);

		botWindowCtrl.topLeft.hide();
		botWindowCtrl.topRight.hide();
		botWindowCtrl.bottomLeft.hide();
		botWindowCtrl.bottomRight.reshape(WindowGap, WindowGap,MainSubWindowWidth, MainSubWindowHeight);

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
		botWindowCtrl.bottomRight.changeEyePosition(0, -viewAngle, -heightAngle);
	}

	botWindowCtrl.bottomRight.changeEyePosition(-20*lastMouseScroll, 0,0);
	lastMouseScroll = 0;

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
	// spinner values are changed with live variables. Round it
	static float lastSpinnerVal[NumberOfActuators];
	float lastValue = lastSpinnerVal[angleControlNumber];
	float value = anglesLiveVar[angleControlNumber];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;
	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;
	lastSpinnerVal[angleControlNumber] = roundedValue;
	if (lastValue != roundedValue) {
		angleSpinner[angleControlNumber]->set_float_val(roundedValue);
		lastSpinnerVal[angleControlNumber] = roundedValue;
	}

	// since angles have changed recompute kinematics. Call callback
	botWindowCtrl.callbackChangedAngles();

	// indicate that something has changed. idle callback will redraw
	kinematicsHasChanged = true;
}

void TCPSpinnerCallback( int tcpCoordId )
{
	// spinner values are changed with live variables
	static float lastSpinnerValue[6];

	// get value from live var and round it
	float lastValue = lastSpinnerValue[tcpCoordId];
	float value =tcpSpinnerLiveVar[tcpCoordId];
	float roundedValue = sgn(value)*((int)(abs(value)*10.0+.5))/10.0f;
	if ((roundedValue == lastValue) && (roundedValue != value))
		roundedValue += sgn(value-lastValue)*0.1;
	if (lastValue != roundedValue) {
		lastSpinnerValue[tcpCoordId] = roundedValue;
		tcpCoordSpinner[tcpCoordId ]->set_float_val(roundedValue);
	}

	// compute angles out of tcp pose
	botWindowCtrl.callbackChangedTCP();

	kinematicsHasChanged = true;
}


void BotWindowCtrl::callbackChangedTCP() {
	if (tcpCallback != NULL) {
		JointAngleType angles = {0,0,0,0,0,0,0};
		Pose newPose = getPoseView();
		bool ok = (*tcpCallback)(newPose);
	}

	copyAnglesToView();
	copyPoseToView();
	copyConfigurationToView();
}


void BotWindowCtrl::callbackChangedAngles() {
	if (anglesCallback != NULL) {
		JointAngleType angles =getAnglesView();
		(*anglesCallback)(angles);
	}

	copyAnglesToView();
	copyPoseToView();
	copyConfigurationToView();
}

void layoutViewCallback(int radioButtonNo) {
	reshape(WindowWidth, WindowHeight);
	glutPostRedisplay();
}

void configurationViewCallback(int ControlNo) {
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
	new GLUI_StaticText(kinematicsPanel,emptyLine.c_str());

	GLUI_Panel* AnglesPanel= new GLUI_Panel(kinematicsPanel,"Angles", GLUI_PANEL_RAISED);

	string angleName[] = { "hip","upperarm","forearm","ellbow", "wrist", "hand", "gripper" };
	for (int i = 0;i<7;i++) {
		angleSpinner[i] = new GLUI_Spinner(AnglesPanel,angleName[i].c_str(), GLUI_SPINNER_FLOAT,&anglesLiveVar[i],i, angleSpinnerCallback);
		angleSpinner[i]->set_float_limits(degrees(actuatorLimits[i].minAngle),degrees(actuatorLimits[i].maxAngle));
		angleSpinner[i]->set_float_val(0.0);
	}


	GLUI_Panel* TCPPanel= new GLUI_Panel(kinematicsPanel,"TCP", GLUI_PANEL_RAISED);
	string coordName[3] = {"x","y","z" };
	for (int i = 0;i<3;i++) {
		tcpCoordSpinner[i]= new GLUI_Spinner(TCPPanel,coordName[i].c_str(), GLUI_SPINNER_FLOAT,&tcpSpinnerLiveVar[i],i, TCPSpinnerCallback);
	}
	GLUI_Panel* PosePanel= new GLUI_Panel(kinematicsPanel,"Pose", GLUI_PANEL_RAISED);
	string rotName[3] = {"yaw","nick","roll" };
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
	frontBackRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configDirectionLiveVar, 0, configurationViewCallback);
	new GLUI_RadioButton( frontBackRadioGroup,"front");
	new GLUI_RadioButton( frontBackRadioGroup, "back");
	frontBackRadioGroup->set_int_val(configDirectionLiveVar);
	windowHandle->add_column_to_panel(frontBackPanel, true);
	poseFlipRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configFlipLiveVar, 1, configurationViewCallback);
	new GLUI_RadioButton( poseFlipRadioGroup, "flip");
	new GLUI_RadioButton( poseFlipRadioGroup, "reg");
	poseFlipRadioGroup->set_int_val(configFlipLiveVar);
	windowHandle->add_column_to_panel(frontBackPanel, true);
	poseForearmRadioGroup= new GLUI_RadioGroup( frontBackPanel,&configTurnLiveVar, 2, configurationViewCallback);
	new GLUI_RadioButton( poseForearmRadioGroup, "up");
	new GLUI_RadioButton( poseForearmRadioGroup, "dn");
	poseForearmRadioGroup->set_int_val(configTurnLiveVar);;

	GLUI_Panel* layoutPanel = new GLUI_Panel(windowHandle,"Layout", GLUI_PANEL_EMBOSSED);
	layoutPanel->set_alignment(GLUI_ALIGN_RIGHT);
	GLUI_RadioGroup *layoutRadioGroup= new GLUI_RadioGroup( layoutPanel,&layoutButtonSelection,4, layoutViewCallback);
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
	while ((millis() - startTime < 20000) && (!uiReady));

	return uiReady;
}


void BotWindowCtrl::eventLoop() {
	LOG(DEBUG) << "BotWindowCtrl::eventLoop";
	glutInitWindowSize(WindowWidth, WindowHeight);
    wMain = glutCreateWindow("Bad Robot"); // Create a window with the given title
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	// GLUI_Master.set_glutMouseFunc( SubWindows3DMouseCallback );

	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( idleCallback);

	wTopLeft = topLeft.create(wMain,"top view", BotView::TOP_VIEW);
	glutDisplayFunc(display);
	wTopRight = topRight.create(wMain,"front view",BotView::FRONT_VIEW);
	glutDisplayFunc(display);
	wBottomLeft = bottomLeft.create(wMain,"right view", BotView::RIGHT_VIEW);
	glutDisplayFunc(display);
	wBottomRight= bottomRight.create(wMain,"3D", BotView::_3D_VIEW);
	glutDisplayFunc(display);

	// 3D view can be rotated with mouse
	glutSetWindow(wBottomRight);
	glutMotionFunc( SubWindow3dMotionCallback);
	glutMouseFunc( SubWindows3DMouseCallback);
	createInteractiveWindow(wMain);
	glutSetWindow(wMain);

	glutTimerFunc(0, StartupTimerCallback, 0);	// timer that sets the view point of startup procedure

	// set initial values of robot angles and position
	copyAnglesToView();
	copyConfigurationToView();
	copyPoseToView();

	uiReady = true; 							// tell calling thread to stop waiting for ui initialization
	LOG(DEBUG) << "starting GLUT main loop";
	glutMainLoop();  							// Enter the infinitely event-processing loop
}

// set callback invoked whenever an angle is changed via ui
void BotWindowCtrl::setAnglesCallback(void (* callback)( JointAngleType angles)) {
	anglesCallback = callback;
}

// set callback invoked whenever the tcp or configuration is changed via ui
void BotWindowCtrl::setTcpInputCallback(bool (* callback)( const Pose& pose)) {
	tcpCallback = callback;
}

