/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include <windows.h>  // For MS Windows
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GLUI2/glui2.h>

using namespace std;

// Global Glui2 Handle
Glui2* GluiHandle = NULL;
// Window size
const int WindowWidth = 800;
const int WindowHeight = 600;

#define GAP 10
int SubWindowHeight = 100;
int SubWindowWidth = 100;

#define glSubWindowColor 0.95,0.95,0.95
#define glEyeDistance 10.0f

int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;

void printText(float x, float y, std::string text) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();                 // Reset the model-view matrix
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glColor3f(0.1f, 0.1f, 0.1f);
	glRasterPos2f(x, y);

	glutBitmapString(GLUT_BITMAP_HELVETICA_12,
			(const unsigned char*) text.c_str());
}

void paintBot() {

	glMatrixMode(GL_MODELVIEW);

	glClearColor(glSubWindowColor, 0.0f); // Set background color to white and opaque
	//  glTranslatef(0.f, 0.9f, -0.0f);  // Move right and into the screen

	glBegin(GL_QUADS);              // Begin drawing the color cube with 6 quads
	// Top face (y = 1.0f)
	// Define vertices in counter-clockwise (CCW) order with normal pointing out
	glColor3f(0.0f, 1.0f, 0.0f);     // Green
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);

	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);

	// Back face (z = -1.0f)
	glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);

	// Right face (x = 1.0f)
	glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();  // End of drawing color-cube
}

void setSubWindowBotView(int window) {
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();                 // Reset the model-view matrix

	// Enable perspective projection with fovy, aspect, zNear and zFar
	GLfloat aspectSubWindow = (GLfloat) SubWindowWidth / (GLfloat) SubWindowHeight;

	gluPerspective(45.0f, aspectSubWindow, 0.1f, 100.0f);
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

	glClearColor(1.0f, 1.0f, 1.0f, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer

	glutSetWindow(wTopLeft);
	glClearColor(glSubWindowColor, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "top view");
	setSubWindowBotView(wTopLeft);

	glutSetWindow(wTopRight);
	glClearColor(glSubWindowColor, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "front view");
	setSubWindowBotView(wTopRight);

	glutSetWindow(wBottomLeft);
	glClearColor(glSubWindowColor, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "side view");

	setSubWindowBotView(wBottomLeft);
	glutSetWindow(wBottomRight);
	glClearColor(glSubWindowColor, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "3D");
	setSubWindowBotView(wBottomRight);


	glFlush();  // Render now
}

/* Called back when timer expired [NEW] */
void timer(int value) {
	glutPostRedisplay();      // Post re-paint request to activate display()
	glutTimerFunc(100, timer, 0); // next timer call milliseconds later
}

void vis(int visState) {
	printf("VIS: win=%d, v=%d\n", glutGetWindow(), visState);
}
void reshape(int w, int h) {

	glViewport(0, 0, w, h);
	if (w > 50) {
		SubWindowWidth = (w - 3 * GAP) / 2;
	} else {
		SubWindowWidth = GAP;
	}
	if (h > 50) {
		SubWindowHeight = (h - 3 * GAP) / 2;
	} else {
		SubWindowHeight = GAP;
	}

	if (SubWindowHeight == 0)
		SubWindowHeight = 1;                // To prevent divide by 0

	glutSetWindow(wTopLeft);
	glutPositionWindow(GAP, GAP);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight); // Set the viewport to cover the new window

	glutSetWindow(wTopRight);
	glutPositionWindow(GAP + SubWindowWidth + GAP, GAP);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

	glutSetWindow(wBottomLeft);
	glutPositionWindow(GAP, GAP + SubWindowHeight + GAP);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

	glutSetWindow(wBottomRight);
	glutPositionWindow(GAP + SubWindowWidth + GAP, GAP + SubWindowHeight + GAP);
	glutReshapeWindow(SubWindowWidth, SubWindowHeight);
	glViewport(0, 0, SubWindowWidth, SubWindowHeight);

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

	wTopLeft = glutCreateSubWindow(wMain, GAP, GAP, SubWindowWidth,
			SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections

	wTopRight = glutCreateSubWindow(wMain, GAP + SubWindowWidth + GAP, GAP,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections

	wBottomLeft = glutCreateSubWindow(wMain, GAP, GAP + SubWindowHeight + GAP,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections

	wBottomRight = glutCreateSubWindow(wMain, GAP + SubWindowWidth + GAP,
					GAP + SubWindowHeight + GAP, SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections

	glutTimerFunc(0, timer, 0);
	glutMainLoop();           	// Enter the infinitely event-processing loop

	// Suppress warning
	return 0;
}

