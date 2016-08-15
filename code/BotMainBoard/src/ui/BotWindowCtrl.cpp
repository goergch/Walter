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


int WindowWidth = 1000;						// initial window size
int WindowHeight = 700;
int WindowGap=10;							// gap between subwindows
int InteractiveWindowWidth=390;				// initial width of the interactive window

enum LayoutType { SINGLE_LAYOUT = 0, MIXED_LAYOUT=1 };// layout type for bot view
int layoutButtonSelection=MIXED_LAYOUT;		// live variable of radio group

static GLfloat glMainWindowColor[] 		= {1.0,1.0,1.0};

// startup animation
float startUpDuration = 5000;				// duration of startup animation

// handles of opengl windows and subwindows
int wMain, wMainBotView, wSideBotView, wFrontBotView, wTopBotView;	// window handler of windows

// kinematics widget
GLUI_Spinner* tcpCoordSpinner[7] = {NULL,NULL,NULL, NULL, NULL, NULL, NULL};
bool tcpCoordSpinnerINT[7] = {false, false, false, false, false, false, true };
float tcpSpinnerLiveVar[7] = {0,0,0,
							 0,0,0,
							 0};

GLUI_Spinner* angleSpinner[NumberOfActuators] = {NULL,NULL,NULL,NULL,NULL,NULL,NULL};
float anglesLiveVar[NumberOfActuators] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
bool angleSpinnerINT[NumberOfActuators] = {false, false, false, false, false, false, true };

bool kinematicsHasChanged = false; 				// true, if something in kinematics has changed

// configuration widget
GLUI_Checkbox *confDirectionCheckbox= NULL;
GLUI_Checkbox  *confgFlipCheckbox= NULL;
GLUI_Checkbox  *configTurnCheckbox= NULL;
int configDirectionLiveVar= 0;					// kinematics configuration, bot looks to the front or to the back
int configFlipLiveVar = 0;						// kinematics triangle flip
int configTurnLiveVar = 0;						// kinematics forearm flip

// each mouse motion call requires a display() call before doing the next mouse motion call
// (without that, we have so many motion calls that rendering is bumpy)
volatile static bool mouseMotionDisplayMutex = true;
volatile static bool controllerDisplayMutex = true;

bool BotWindowCtrl::readyForControllerEvent() {
	if (controllerDisplayMutex) {
		controllerDisplayMutex = false;
		return false;
	} else
		return true;// wait for display first
}

void postRedisplay() {
	int saveWindow = glutGetWindow();
	glutSetWindow(wMain);
	glutPostRedisplay();
	glutSetWindow(saveWindow );
}


void copyAnglesToView() {
	static float lastAngle[NumberOfActuators];
	for (int i = 0;i<NumberOfActuators;i++) {
		float value = degrees(MainBotController::getInstance().getCurrentAngles()[i]);
		value = roundValue(value);
		if (value != lastAngle[i]) {
			angleSpinner[i]->set_float_val(value);
			lastAngle[i] = value;
		}
	}

	BotWindowCtrl::getInstance().topBotView.setAngles(MainBotController::getInstance().getCurrentAngles(), MainBotController::getInstance().getCurrentPose());
	BotWindowCtrl::getInstance().frontBotView.setAngles(MainBotController::getInstance().getCurrentAngles(), MainBotController::getInstance().getCurrentPose());
	BotWindowCtrl::getInstance().sideBotView.setAngles(MainBotController::getInstance().getCurrentAngles(), MainBotController::getInstance().getCurrentPose());
	BotWindowCtrl::getInstance().mainBotView.setAngles(MainBotController::getInstance().getCurrentAngles(), MainBotController::getInstance().getCurrentPose());
}

JointAngleType getAnglesView() {
	JointAngleType angles = {0,0,0,0,0,0,0};
	for (int i = 0;i<NumberOfActuators;i++) {
		angles[i] = radians(anglesLiveVar[i]);
	}
	return angles;
}



void copyPoseToView() {
	static float lastTcp[7];
	const Pose& tcp = MainBotController::getInstance().getCurrentPose();
	for (int i = 0;i<7;i++) {
		rational value;
		if (i<3)
			value = tcp.position[i];
		else
			if (i<6)
				value = degrees(tcp.orientation[i-3]);
			else
				value = degrees(tcp.gripperAngle);

		value = roundValue(value);
		if (value != lastTcp[i]) {
			tcpCoordSpinner[i]->set_float_val(value); // set only when necessary, otherwise the cursor blinks
			lastTcp[i] = value;
		}
	}

	// synchronize pose to angle
	angleSpinner[GRIPPER]->set_float_val(degrees(tcp.gripperAngle));
}

Pose getPoseView() {
	Pose tcp;
	for (int i = 0;i<7;i++) {
		float value = tcpSpinnerLiveVar[i];
		if (i<3)
			tcp.position[i] = value;
		else
			if (i<6)
				tcp.orientation[i-3] = radians(value);
			else
				tcp.gripperAngle = radians(value);
	}

	return tcp;
}

KinematicConfigurationType getConfigurationView() {
	KinematicConfigurationType config;
	config.poseDirection = (KinematicConfigurationType::PoseDirectionType)confgFlipCheckbox->get_int_val();
	config.poseFlip = (KinematicConfigurationType::PoseFlipType)confgFlipCheckbox->get_int_val();
	config.poseTurn= (KinematicConfigurationType::PoseForearmType)configTurnCheckbox->get_int_val();

	return config;
}


void copyConfigurationToView() {
	KinematicConfigurationType config = MainBotController::getInstance().getCurrentConfiguration();
	confDirectionCheckbox->set_int_val(config.poseDirection);
	confgFlipCheckbox->set_int_val(config.poseFlip);
	configTurnCheckbox->set_int_val(config.poseTurn);
	confDirectionCheckbox->disable();
	confgFlipCheckbox->disable();
	configTurnCheckbox->disable();

	std::vector<KinematicsSolutionType> validSolutions = MainBotController::getInstance().getPossibleSolutions();
	for (unsigned int i = 0;i<validSolutions.size();i++) {
		KinematicConfigurationType possibleConfig = validSolutions[i].config;
		if (possibleConfig.poseDirection != config.poseDirection)
			confDirectionCheckbox->enable();
		if (possibleConfig.poseFlip != config.poseFlip)
			confgFlipCheckbox->enable();
		if (possibleConfig.poseTurn != config.poseTurn)
			configTurnCheckbox->enable();
	}
}


/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void display() {
	glutSetWindow(wMain);
	glClearColor(glMainWindowColor[0], glMainWindowColor[1], glMainWindowColor[2], 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	copyAnglesToView();
	copyPoseToView();
	copyConfigurationToView();



	if (layoutButtonSelection == MIXED_LAYOUT) {
		BotWindowCtrl::getInstance().topBotView.display();
		BotWindowCtrl::getInstance().frontBotView.display();
		BotWindowCtrl::getInstance().sideBotView.display();
	}
	BotWindowCtrl::getInstance().mainBotView.display();

	glFlush();  // Render now
	mouseMotionDisplayMutex = true;
	controllerDisplayMutex = true;
}

/* Called back when timer expired [NEW] */
void StartupTimerCallback(int value) {
	static uint32_t startupTime_ms = millis();
	uint32_t timeSinceStart_ms = millis()-startupTime_ms;
	if (timeSinceStart_ms < startUpDuration) {
		float startupRatio= ((float)(timeSinceStart_ms)/startUpDuration)*PI/2.0;
		BotWindowCtrl::getInstance().topBotView.setStartupAnimationRatio(startupRatio);
		BotWindowCtrl::getInstance().frontBotView.setStartupAnimationRatio(startupRatio);
		BotWindowCtrl::getInstance().sideBotView.setStartupAnimationRatio(startupRatio);
		BotWindowCtrl::getInstance().mainBotView.setStartupAnimationRatio(startupRatio);

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
		case MIXED_LAYOUT: {
			int SubWindowHeight = (h - 4 * WindowGap)/3;
			int SubWindowWidth = SubWindowHeight;
			int MainSubWindowHeight = h - 2*WindowGap;
			int MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap - SubWindowWidth);

			BotWindowCtrl::getInstance().topBotView.reshape(WindowGap, WindowGap,SubWindowWidth, SubWindowHeight);
			BotWindowCtrl::getInstance().frontBotView.reshape(WindowGap, 2*WindowGap + SubWindowHeight,SubWindowWidth, SubWindowHeight);
			BotWindowCtrl::getInstance().sideBotView.reshape(WindowGap, 3*WindowGap + 2*SubWindowHeight, SubWindowWidth, SubWindowHeight);
			BotWindowCtrl::getInstance().mainBotView.reshape(2*WindowGap + SubWindowWidth, WindowGap ,MainSubWindowWidth, MainSubWindowHeight);
			break;
		}

		case SINGLE_LAYOUT: {
			int MainSubWindowWidth = (w -InteractiveWindowWidth - 2 * WindowGap);
			int MainSubWindowHeight = (h - 2 * WindowGap);

			BotWindowCtrl::getInstance().topBotView.hide();
			BotWindowCtrl::getInstance().frontBotView.hide();
			BotWindowCtrl::getInstance().sideBotView.hide();
			BotWindowCtrl::getInstance().mainBotView.reshape(WindowGap, WindowGap,MainSubWindowWidth, MainSubWindowHeight);

			break;
		}
	} // switch
}

static int lastMouseX = 0;
static int lastMouseY = 0;
static int lastMouseScroll = 0;
static bool mouseViewPane = false;

static bool mouseBotXZPane = false;
static bool mouseBotYZPane = false;
static bool mouseBotOrientationXYPane = false;
static bool mouseBotOrientationYZPane = false;


void SubWindow3dMotionCallback(int x, int y) {
	// if display has not yet been called after the last motion, dont execute
	// this one, but wait for
	if (mouseMotionDisplayMutex) {
		mouseMotionDisplayMutex = false;
	} else
		return; // wait for display first

	const float slowDownPositionFactor = 1.0/3.0;
	const float slowDownOrientationFactor = 1.0/4.0;

	float diffX = (float) (x-lastMouseX);
	float diffY = (float) (y-lastMouseY);
	if (mouseViewPane) {
		BotWindowCtrl::getInstance().mainBotView.changeEyePosition(0, -diffX, -diffY);
	} else
	if (mouseBotXZPane) {
		tcpCoordSpinner[1]->set_float_val(roundValue(tcpSpinnerLiveVar[1] + diffX*slowDownPositionFactor));
		tcpCoordSpinner[2]->set_float_val(roundValue(tcpSpinnerLiveVar[2] - diffY*slowDownPositionFactor));
		BotWindowCtrl::getInstance().changedPoseCallback();
	} else
	if (mouseBotYZPane) {
		tcpCoordSpinner[0]->set_float_val(roundValue(tcpSpinnerLiveVar[0] + diffX*slowDownPositionFactor));
		tcpCoordSpinner[2]->set_float_val(roundValue(tcpSpinnerLiveVar[2] - diffY*slowDownPositionFactor));
		BotWindowCtrl::getInstance().changedPoseCallback();
	} else
	if (mouseBotOrientationYZPane) {
		tcpCoordSpinner[3]->set_float_val(roundValue(tcpSpinnerLiveVar[3] + diffX*slowDownOrientationFactor));
		tcpCoordSpinner[4]->set_float_val(roundValue(tcpSpinnerLiveVar[4] + diffY*slowDownOrientationFactor));
		BotWindowCtrl::getInstance().changedPoseCallback();
	} else
	if (mouseBotOrientationXYPane) {
		tcpCoordSpinner[5]->set_float_val(roundValue(tcpSpinnerLiveVar[5] + diffX*slowDownOrientationFactor));
		tcpCoordSpinner[4]->set_float_val(roundValue(tcpSpinnerLiveVar[4] + diffY*slowDownOrientationFactor));
		BotWindowCtrl::getInstance().changedPoseCallback();
	} else
		if (lastMouseScroll != 0) {
			BotWindowCtrl::getInstance().mainBotView.changeEyePosition(-20*lastMouseScroll, 0,0);
			kinematicsHasChanged = true;
			lastMouseScroll = 0;
		}

	if (mouseViewPane || mouseBotOrientationYZPane ||  mouseBotOrientationXYPane || mouseBotYZPane || mouseBotXZPane) {
		lastMouseX = x;
		lastMouseY = y;
		postRedisplay();	}

}


void SubWindows3DMouseCallback(int button, int button_state, int x, int y )
{

	/*
	GLfloat winX, winY, winZ;         // Holds Our X, Y and Z Coordinates
	winX = (float)x;                  // Holds The Mouse X Coordinate
	winY = (float)y;                  // Holds The Mouse Y Coordinate
	GLint* viewport;					// Where The Viewport Values Will Be Stored
	GLdouble* modelview;				// Where The 16 Doubles Of The Modelview Matrix Are To Be Stored
	GLdouble* projection;
	botWindowCtrl.bottomRight.getTCPDot(viewport, modelview, projection);

	winY = (float)viewport[3] - winY; // Subtract The Current Mouse Y Coordinate From The Screen Height.
	glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);


	GLdouble posX, posY, posZ;
	bool success = gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
	*/

	mouseViewPane = false;
	mouseBotXZPane = false;
	mouseBotYZPane = false;
	mouseBotOrientationXYPane = false;
	mouseBotOrientationYZPane = false;

	bool withShift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
	bool withCtrl= glutGetModifiers() & GLUT_ACTIVE_CTRL;

	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && !withCtrl) {
	    mouseViewPane = true;
	} else
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && withShift && !withCtrl) {
	    mouseBotXZPane = true;
	}else
	if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && withShift && !withCtrl) {
	    mouseBotYZPane = true;
	}else
	if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN && !withShift && withCtrl) {
	    mouseBotOrientationXYPane = true;
	} else
	if ( button == GLUT_RIGHT_BUTTON && button_state == GLUT_DOWN && !withShift && withCtrl) {
	    mouseBotOrientationYZPane = true;
	} else
	// Wheel reports as button 3(scroll up) and button 4(scroll down)
	if ((button == 3) || (button == 4)) // It's a wheel event
	{
		// Each wheel event reports like a button click, GLUT_DOWN then GLUT_UP
		if (button_state != GLUT_UP) { // Disregard redundant GLUT_UP events
			if (button == 3)
				lastMouseScroll++;
			else
				lastMouseScroll--;
			SubWindow3dMotionCallback(x,y); // scroll wheel does not trigger a glut call of MotionCallback, so we do it manually
		}
	}

	if (mouseViewPane || mouseBotOrientationYZPane ||  mouseBotOrientationXYPane || mouseBotYZPane || mouseBotXZPane) {
	    lastMouseX = x;
	    lastMouseY = y;
		// SubWindow3dMotionCallback(x,y);
	}
}


void GluiReshapeCallback( int x, int y )
{
	reshape(x,y);
	int tx, ty, tw, th;
	int saveWindow = glutGetWindow();
	glutSetWindow(wMain);
	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );
	glutSetWindow(saveWindow);
	postRedisplay();
}

// Idle Call back is used to check, whether anything has changed the
// bots position or view and it needs to be redrawn
void idleCallback( void )
{
	int saveWindow = glutGetWindow();

	if ( glutGetWindow() != wMain) {
		glutSetWindow(wMain);
	}

	if (kinematicsHasChanged) {
		postRedisplay();
		kinematicsHasChanged = false;
	} else
		delay(25); // otherwise we needed 100% cpu, since idle callback is called in an infinite loop by glut
	glutSetWindow(saveWindow);
}

void layoutReset(int buttonNo) {
	for (int i = 0;i<NumberOfActuators;i++) {
		angleSpinner[i]->set_float_val(0);
	}

	// since angles have changed recompute kinematics. Call callback
	BotWindowCtrl::getInstance().changedAnglesCallback();
}
void angleSpinnerCallback( int angleControlNumber )
{
	// spinner values are changed with live variables. Round it
	static float lastSpinnerVal[NumberOfActuators];
	float lastValue = lastSpinnerVal[angleControlNumber];
	float value = anglesLiveVar[angleControlNumber];
	float roundedValue = roundValue(value);
	bool isIntType = angleSpinnerINT[angleControlNumber];
	if (isIntType)
		roundedValue = sgn(value)*((int)(abs(value)+0.5));
	if ((roundedValue == lastValue) && (roundedValue != value)) {
		if (isIntType)
			roundedValue += sgn(value-lastValue);
		else
			roundedValue += sgn(value-lastValue)*0.1;
	}
	lastSpinnerVal[angleControlNumber] = roundedValue;
	if (lastValue != roundedValue) {
		angleSpinner[angleControlNumber]->set_float_val(roundedValue);
		lastSpinnerVal[angleControlNumber] = roundedValue;
	}

	// since angles have changed recompute kinematics. Call callback
	BotWindowCtrl::getInstance().changedAnglesCallback();
}

void poseSpinnerCallback( int tcpCoordId )
{
	// spinner values are changed with live variables
	static float lastSpinnerValue[7] = {0,0,0,0,0,0,0};

	// get value from live var and round it
	float lastValue = lastSpinnerValue[tcpCoordId];
	float value =tcpSpinnerLiveVar[tcpCoordId];
	float roundedValue = roundValue(value);
	bool isIntType = tcpCoordSpinnerINT[tcpCoordId];
	if (isIntType)
		roundedValue = sgn(value)*((int)(abs(value)+0.5));

	if ((roundedValue == lastValue) && (roundedValue != value)) {
		if (isIntType)
			roundedValue += sgn(value-lastValue);
		else
			roundedValue += sgn(value-lastValue)*0.1;
	}
	if (lastValue != roundedValue) {
		tcpCoordSpinner[tcpCoordId ]->set_float_val(roundedValue);
		lastSpinnerValue[tcpCoordId] = roundedValue;
	}

	// compute angles out of tcp pose
	BotWindowCtrl::getInstance().changedPoseCallback();
}

void configurationViewCallback(int ControlNo) {
	KinematicConfigurationType config;
	config.poseDirection 	= (configDirectionLiveVar==0)?KinematicConfigurationType::FRONT:KinematicConfigurationType::BACK;
	config.poseFlip 		= (configFlipLiveVar==0)?KinematicConfigurationType::FLIP:KinematicConfigurationType::NO_FLIP;
	config.poseTurn 		= (configTurnLiveVar==0)?KinematicConfigurationType::UP:KinematicConfigurationType::DOWN;

	const std::vector<KinematicsSolutionType>& solutions = MainBotController::getInstance().getPossibleSolutions();
	int changeConfigurationTries = 0;
	int changeConfigurationControl = ControlNo;
	do {
		changeConfigurationTries++;
		for (unsigned int i = 0;i<solutions.size();i++) {
			KinematicsSolutionType sol = solutions[i];
			if ((sol.config.poseDirection == config.poseDirection) &&
				(sol.config.poseFlip == config.poseFlip) &&
				(sol.config.poseTurn == config.poseTurn))
			{
				// key in angles manually and initiate forward kinematics
				for (unsigned int i = 0;i<NumberOfActuators;i++) {
					float value = degrees(sol.angles[i]);
					float roundedValue = roundValue(value);
					bool isIntType = angleSpinnerINT[i];
					if (isIntType)
						roundedValue = sgn(value)*((int)(abs(value)+0.5));
					angleSpinner[i]->set_float_val(roundedValue);
				}

				BotWindowCtrl::getInstance().changedAnglesCallback();
				return; // solution is found, quit
			}
		}

		// we did not found a valid solution, we need to change another parameter to get a valid one
		changeConfigurationControl = (changeConfigurationControl+1) % 3;
		if (changeConfigurationControl == ControlNo)
			changeConfigurationControl = (changeConfigurationControl+1) % 3;

		switch (changeConfigurationControl) {
		case 0:
			config.poseDirection = (config.poseDirection==KinematicConfigurationType::BACK)?KinematicConfigurationType::FRONT:KinematicConfigurationType::BACK;
			break;
		case 1:
			config.poseFlip = (config.poseFlip==KinematicConfigurationType::NO_FLIP)?KinematicConfigurationType::FLIP:KinematicConfigurationType::NO_FLIP;
			break;
		case 2:
			config.poseTurn = (config.poseTurn==KinematicConfigurationType::DOWN)?KinematicConfigurationType::UP:KinematicConfigurationType::DOWN;
			break;
		default:
			LOG(ERROR) << "configuration invalid";
		}
	} while (changeConfigurationTries <= 3); // we have three configuration dimensions, so try two other ones max
	LOG(ERROR) << "valid configuration not found";
}


void BotWindowCtrl::notifyNewBotData() {
	kinematicsHasChanged = true;
}



void BotWindowCtrl::changedPoseCallback() {
	if (tcpCallback != NULL) {
		Pose newPose = getPoseView();
		(*tcpCallback)(newPose);
	}

	notifyNewBotData();
}



void BotWindowCtrl::changedAnglesCallback() {
	if (anglesCallback != NULL) {
		JointAngleType angles =getAnglesView();
		(*anglesCallback)(angles);
	}

	notifyNewBotData();
}

void layoutViewCallback(int radioButtonNo) {
	reshape(WindowWidth, WindowHeight);
	postRedisplay();
}


GLUI* BotWindowCtrl::createInteractiveWindow(int mainWindow) {

	string emptyLine = "                                               ";

	GLUI *windowHandle= GLUI_Master.create_glui_subwindow( wMain,  GLUI_SUBWINDOW_RIGHT );
	windowHandle->set_main_gfx_window( wMain );

	GLUI_Panel* interactivePanel = new GLUI_Panel(windowHandle,"interactive panel", GLUI_PANEL_NONE);
	GLUI_Panel* kinematicsPanel = new GLUI_Panel(interactivePanel,"kinematics panel", GLUI_PANEL_NONE);
	kinematicsPanel->set_alignment(GLUI_ALIGN_RIGHT);
	// new GLUI_StaticText(kinematicsPanel,emptyLine.c_str());

	GLUI_Panel* AnglesPanel= new GLUI_Panel(kinematicsPanel,"angles panel", GLUI_PANEL_RAISED);
	// new GLUI_StaticText(AnglesPanel, "forward kinematics");

	string angleName[] = { "hip","upperarm","forearm","ellbow", "wrist", "hand", "finger" };
	for (int i = 0;i<7;i++) {
		angleSpinner[i] = new GLUI_Spinner(AnglesPanel,angleName[i].c_str(), GLUI_SPINNER_FLOAT,&anglesLiveVar[i],i, angleSpinnerCallback);
		// angleSpinner[i]->set_alignment(GLUI_ALIGN_RIGHT);
		angleSpinner[i]->set_float_limits(degrees(actuatorLimits[i].minAngle),degrees(actuatorLimits[i].maxAngle));
		angleSpinner[i]->set_float_val(0.0);
	}

	windowHandle->add_column_to_panel(kinematicsPanel, false);

	GLUI_Panel* TCPPanel= new GLUI_Panel(kinematicsPanel,"IK panel", GLUI_PANEL_RAISED);

	string coordName[7] = {"x","y","z","roll","nick","yaw", "finger" };
	for (int i = 0;i<7;i++) {
		tcpCoordSpinner[i]= new GLUI_Spinner(TCPPanel,coordName[i].c_str(), GLUI_SPINNER_FLOAT,&tcpSpinnerLiveVar[i],i, poseSpinnerCallback);
	}
	tcpCoordSpinner[X]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Y]->set_float_limits(-1000,1000);
	tcpCoordSpinner[Z]->set_float_limits(0,1000);

	tcpCoordSpinner[3]->set_float_limits(-360, 360);
	tcpCoordSpinner[4]->set_float_limits(-360, 360);
	tcpCoordSpinner[5]->set_float_limits(-360, 360);
	tcpCoordSpinner[6]->set_float_limits(degrees(actuatorLimits[GRIPPER].minAngle),degrees(actuatorLimits[GRIPPER].maxAngle));


	GLUI_Panel* configurationPanel= new GLUI_Panel(interactivePanel,"configuration", GLUI_PANEL_RAISED);

	confDirectionCheckbox = new GLUI_Checkbox( configurationPanel,"direction",&configDirectionLiveVar, 0, configurationViewCallback);
	windowHandle->add_column_to_panel(configurationPanel, false);
	confgFlipCheckbox = new GLUI_Checkbox( configurationPanel, "flip    ", &configFlipLiveVar, 1, configurationViewCallback);
	windowHandle->add_column_to_panel(configurationPanel, false);
	configTurnCheckbox = new GLUI_Checkbox( configurationPanel, "turn    ",&configTurnLiveVar, 2, configurationViewCallback);
	windowHandle->add_column_to_panel(configurationPanel, false);
	new GLUI_Button(configurationPanel, "reset", 0, layoutReset);

	trajectoryView.create(windowHandle, interactivePanel);

	GLUI_Panel* layoutPanel = new GLUI_Panel(interactivePanel,"Layout", GLUI_PANEL_RAISED);
	new GLUI_StaticText(layoutPanel,"                                    layout                                   ");
	GLUI_RadioGroup *layoutRadioGroup= new GLUI_RadioGroup( layoutPanel,&layoutButtonSelection,4, layoutViewCallback);
	new GLUI_RadioButton( layoutRadioGroup, "single view" );
	new GLUI_RadioButton( layoutRadioGroup, "mixed view" );
	layoutRadioGroup->set_int_val(MIXED_LAYOUT);

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
	glutInitWindowPosition(20, 20); // Position the window's initial top-left corner
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	// GLUI_Master.set_glutMouseFunc( SubWindows3DMouseCallback );

	GLUI_Master.set_glutReshapeFunc( GluiReshapeCallback );
	GLUI_Master.set_glutIdleFunc( idleCallback);

	wTopBotView = topBotView.create(wMain,"top view", BotView::TOP_VIEW, false);
	glutDisplayFunc(display);
	wFrontBotView = frontBotView.create(wMain,"front view",BotView::FRONT_VIEW, false);
	glutDisplayFunc(display);
	wSideBotView = sideBotView.create(wMain,"right view", BotView::RIGHT_VIEW, false);
	glutDisplayFunc(display);
	wMainBotView= mainBotView.create(wMain,"", BotView::_3D_VIEW, true);
	glutDisplayFunc(display);

	// Main view has comprehensive mouse motion
	glutSetWindow(wMainBotView);
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
void BotWindowCtrl::setAnglesCallback(void (* callback)( const JointAngleType& angles)) {
	anglesCallback = callback;
}

// set callback invoked whenever the tcp or configuration is changed via ui
void BotWindowCtrl::setTcpInputCallback(bool (* callback)( const Pose& pose)) {
	tcpCallback = callback;
}

