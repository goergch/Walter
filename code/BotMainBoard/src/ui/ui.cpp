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


using namespace std;



// Window size
int WindowWidth = 800;
int WindowHeight = 600;

int WindowGap=10;
int SubWindowHeight = 10;
int SubWindowWidth = 10;
int InteractiveWindowWidth=150;

static GLfloat glMainWindowColor[] = {1.0,1.0,1.0};
static GLfloat glSubWindowColor[] = {0.95,0.95,0.95};
static GLfloat glBotColor[] = { 1.0f, 0.2f, 0.0f };
static GLfloat glWindowTitleColor[] = { 1.0f, 1.0f, 1.0f };

#define glEyeDistance 1000.0f

int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;



float botAngles[6] = {0,0,0,0,0,0 };
void initializeLighting()
{
  GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
  GLfloat light_diffuse[] =  {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position0[] = {glEyeDistance*3, glEyeDistance*3, glEyeDistance*3, 0.0};

  GLfloat mat_ambient[] =  {0.6, 0.6, 0.6, 1.0};
  GLfloat mat_diffuse[] =  {0.4, 0.8, 0.4, 1.0};
  GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat mat_shinynes[] = {50.0};

  glMaterialfv(GL_LIGHT0, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_LIGHT0, GL_DIFFUSE, mat_diffuse);
  glMaterialfv(GL_LIGHT0, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_LIGHT0, GL_SPECULAR, mat_shinynes);

  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

  glEnable(GL_LIGHT0);
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

void printBotInfo() {



	glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);

	float linePos = 0.9;
	float lineDistance = 0.06;
	glRasterPos2f(0.7,linePos);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "________________");
	linePos -= lineDistance;



	string angleName[] = { "hip","shoulder","forearm","ellbow","upperarm", "wrist" };
	for (int i = 0;i<6;i++) {
		glRasterPos2f(0.7,linePos);
		glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) angleName[i].c_str());
		glRasterPos2f(0.85,linePos);
		glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) string_format("= % 4.1f °",botAngles[i]).c_str());

		linePos -= lineDistance;
	}
	glRasterPos2f(0.7,linePos);
	glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*) "________________");
	linePos -= lineDistance;

}


void paintBot() {

	const float baseplateHeight= 20;
	const float armlength = 120;
	const float jointradius= 30;
	const float armradius = 20;

	glMatrixMode(GL_MODELVIEW);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1],glSubWindowColor[2],0.0f); // Set background color to white and opaque

	// base plate
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidCylinder(150.0, baseplateHeight, 36, 1);
	glPopMatrix();

	// shoulder
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0);
	glTranslatef(0.0, 0.0,baseplateHeight);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidCylinder(armradius, armlength, 36, 1);
	glPopMatrix();

	// shoulder joint
	glPushMatrix();
	glTranslatef(0.0,armlength + baseplateHeight,0.0);  // Move right and into the screen
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidSphere(jointradius, 36, 36);
	glPopMatrix();

	// upperarm
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0); // turn along z axis
	glTranslatef(0.0,0.0,baseplateHeight+armlength);  // move to its start height
	glRotatef(botAngles[1],0.0,0.0, 1.0); // turn along angle
	glRotatef(botAngles[0],0.0,1.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidCylinder(20.0, armlength, 36, 1);
	glPopMatrix();

	// upperarm joint
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0); // turn along z axis
	glTranslatef(0.0,0.0,baseplateHeight+armlength);  // move to its start height
	glRotatef(botAngles[1],0.0,0.0, 1.0); // turn along angle
	glRotatef(botAngles[0],0.0,1.0, 0.0); // rotate along base angle
	glTranslatef(0.0,0.0,armlength);  // move to its start height
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidSphere(jointradius, 36, 36);
	glPopMatrix();

}

void setSubWindowBotView(int window) {
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();                 // Reset the model-view matrix

	// Enable perspective projection with fovy, aspect, zNear and zFar
	GLfloat aspectSubWindow = (GLfloat) SubWindowWidth / (GLfloat) SubWindowHeight;

	gluPerspective(45.0f, aspectSubWindow, 0.1f, 5000.0f);
	static float par = 0.0;
	par += 0.02;

	if (window == wTopLeft) {
		// view from top
		gluLookAt(0, glEyeDistance,0,
				  0.0, 0.0, 0.0,
				  1.0, 0.0,	0.0);
	} else if (window == wTopRight) {
		// view from front
		gluLookAt(0.0,0.0, glEyeDistance ,
			      0.0,  0.0, 0.0,
				  0.0, 1.0,	0.0);

	} else if (window == wBottomLeft) {
		// view from side
		gluLookAt(-glEyeDistance, 0.0 ,0.0,
				  0.0, 0.0, 0.0,
				  0.0, 1.0,0.0);
	} else {
		gluLookAt(glEyeDistance* sin(par), glEyeDistance, glEyeDistance * cos(par),
				0.0, 0.0, 0.0,
				0.0, 1.0, 0.0);
	}


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();                 // Reset the model-view matrix


	paintBot();
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void repaintBotWindow() {
	glutSetWindow(wMain);
	glClearColor(glMainWindowColor[0], glMainWindowColor[1], glMainWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
	printBotInfo();

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
void timer(int value) {
	static float par = 0.0;
	par += 0.05;
	botAngles[0] =20*sin(par*3);
	botAngles[1] =20*cos(par);

	glutPostRedisplay();      // Post re-paint request to activate display()
	glutTimerFunc(100, timer, 0); // next timer call milliseconds later
}

void vis(int visState) {
	printf("VIS: win=%d, v=%d\n", glutGetWindow(), visState);
}

void reshape(int w, int h) {

	WindowWidth = w;
	WindowHeight = h;
	glViewport(0, 0, w, h);
	if (w > 50) {
		SubWindowWidth = (w -InteractiveWindowWidth - 3 * WindowGap) / 2;
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
	glViewport(0, 0, SubWindowWidth, SubWindowHeight); // Set the viewport to cover the new window

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

void myGlutKeyboard(unsigned char Key, int x, int y)
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

void myGlutMouse(int button, int button_state, int x, int y )
{
}

void myGlutReshape( int x, int y )
{
  int tx, ty, tw, th;
  GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
  glViewport( tx, ty, tw, th );

  float xy_aspect = (float)tw / (float)th;
  reshape(x,y);
  glutPostRedisplay();
}

int startBotUI(int argc, char** argv) {

	glutInit(&argc, argv);                 			// Initialize GLUT
	// glutInitDisplayMode(GLUT_DOUBLE); 				// Enable double buffered mode
	glutInitWindowSize(WindowWidth, WindowHeight);
	wMain = glutCreateWindow("Bad Robot"); // Create a window with the given title
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glutReshapeFunc(reshape);

	GLUI_Master.set_glutKeyboardFunc( myGlutKeyboard );
	GLUI_Master.set_glutMouseFunc( myGlutMouse );
	GLUI_Master.set_glutReshapeFunc( myGlutReshape );


	wTopLeft = glutCreateSubWindow(wMain, WindowGap, WindowGap, SubWindowWidth,
			SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	wTopRight = glutCreateSubWindow(wMain, WindowGap + SubWindowWidth + WindowGap, WindowGap,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	wBottomLeft = glutCreateSubWindow(wMain, WindowGap, WindowGap + SubWindowHeight + WindowGap,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	wBottomRight = glutCreateSubWindow(wMain, WindowGap + SubWindowWidth + WindowGap,
					WindowGap + SubWindowHeight + WindowGap, SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	glutTimerFunc(0, timer, 0);
	glutMainLoop();           	// Enter the infinitely event-processing loop

	// Suppress warning
	return 0;
}

