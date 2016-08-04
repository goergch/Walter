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

static GLfloat glSubWindowColor[] = {0.95,0.95,0.95};
static GLfloat glBotColor[] = { 1.0f, 0.2f, 0.0f };

#define glEyeDistance 1000.0f

int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;


void initializeLighting()
{
  GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
  GLfloat light_diffuse[] =  {0.8, 0.8, 0.8, 1.0};
  GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat light_position0[] = {3000.0, 3000.0, 3000.0, 0.0};


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

void highlightBegin3f(GLfloat r, GLfloat g, GLfloat b)
{
  static GLfloat red[4] =
  {r, g, b, 1.0 };

  glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, red);
  glColor3fv(red);
}

void highlightEnd(void)
{
  glPopAttrib();
}

void printText(float x, float y, std::string text) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();                 // Reset the model-view matrix
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	// glColor3f(0.1f, 0.1f, 0.1f);
	GLfloat black[] =  { 1.0f, 1.0f, 1.0f };

	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, black);

	glRasterPos2f(x, y);

	glutBitmapString(GLUT_BITMAP_HELVETICA_12,
			(const unsigned char*) text.c_str());
}

void paintCube() {

	glMatrixMode(GL_MODELVIEW);
	glClearColor(glSubWindowColor[0],glSubWindowColor[1],glSubWindowColor[0],0.0f); // Set background color to white and opaque
	//  glTranslatef(0.f, 0.9f, -0.0f);  // Move right and into the screen

	glBegin(GL_QUADS);              // Begin drawing the color cube with 6 quads
	// Top face (y = 1.0f)
	// Define vertices in counter-clockwise (CCW) order with normal pointing out

	GLfloat green[] =  { 0.0f, 1.0f, 0.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, green);
	// glColor3fv(green);
	glVertex3f(100.0f, 100.0f, -100.0f);
	glVertex3f(-100.0f, 100.0f, -100.0f);
	glVertex3f(-100.0f, 100.0f, 100.0f);
	glVertex3f(100.0f, 100.0f, 100.0f);

	// Bottom face (y = -1.0f)
	GLfloat orange[] =  { 1.0f, 0.5f, 0.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, orange);
	// glColor3fv(orange);     // Orange
	glVertex3f(100.0f, -100.0f, 100.0f);
	glVertex3f(-100.0f, -100.0f, 100.0f);
	glVertex3f(-100.0f, -100.0f, -100.0f);
	glVertex3f(100.0f, -100.0f, -100.0f);

	// Front face  (z = 1.0f)
	GLfloat red[] =  { 1.0f, 0.0f, 0.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, red);
	// glColor3fv(red);     // Red
	glVertex3f(100.0f, 100.0f, 100.0f);
	glVertex3f(-100.0f, 100.0f, 100.0f);
	glVertex3f(-100.0f, -100.0f, 100.0f);
	glVertex3f(100.0f, -100.0f, 100.0f);

	// Back face (z = -1.0f)
	GLfloat yellow[] =  { 1.0f, 1.0f, 0.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, yellow);
	// glColor3fv(yellow);     // Yellow

	glVertex3f(100.0f, -100.0f, -100.0f);
	glVertex3f(-100.0f, -100.0f, -100.0f);
	glVertex3f(-100.0f, 100.0f, -100.0f);
	glVertex3f(100.0f, 100.0f, -100.0f);

	// Left face (x = -1.0f)
	GLfloat blue[] =  { 0.0f, 0.0f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, blue);
	// glColor3fv(blue);     // Blue
	glVertex3f(-100.0f, 100.0f, 100.0f);
	glVertex3f(-100.0f, 100.0f, -100.0f);
	glVertex3f(-100.0f, -100.0f, -100.0f);
	glVertex3f(-100.0f, -100.0f, 100.0f);

	// Right face (x = 1.0f)
	GLfloat magenta[] =  { 1.0f, 0.0f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, magenta);
	// glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	glVertex3f(100.0f, 100.0f, -100.0f);
	glVertex3f(100.0f, 100.0f, 100.0f);
	glVertex3f(100.0f, -100.0f, 100.0f);
	glVertex3f(100.0f, -100.0f, -100.0f);

	glEnd();  // End of drawing color-cube
}


void paintBot(float angle1, float angle2, float angle3, float angle4,float angle5,float angle6) {

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
	glTranslatef(0.0,armlength + baseplateHeight +jointradius);  // Move right and into the screen
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidSphere(jointradius, 36, 36);
	glPopMatrix();

	// upperarm
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0); // turn along z axis
	glTranslatef(0.0,0.0,baseplateHeight+armlength);  // move to its start height
	glRotatef(angle2,0.0,0.0, 1.0); // turn along angle
	glRotatef(angle1,0.0,1.0, 0.0); // rotate along base angle
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, glBotColor);
	glutSolidCylinder(20.0, armlength, 36, 1);
	glPopMatrix();

	// upperarm joint
	glPushMatrix();
	glRotatef(-90.0,1.0,0.0, 0.0); // turn along z axis
	glTranslatef(0.0,0.0,baseplateHeight+armlength);  // move to its start height
	glRotatef(angle2,0.0,0.0, 1.0); // turn along angle
	glRotatef(angle1,0.0,1.0, 0.0); // rotate along base angle
	glTranslatef(0.0,0.0,armlength+jointradius);  // move to its start height
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

	paintCube();
	float angle1= 20*sin(par/2);
	float angle2= 20*sin(par);

	paintBot(angle1,angle2,0,0,0,0);
}

/* Handler for window-repaint event. Call back when the window first appears and
 whenever the window needs to be re-painted. */
void repaintBotWindow() {
	glutSetWindow(wMain);

	glClearColor(1.0f, 1.0f, 1.0f, 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer

	glutSetWindow(wTopLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "top view");
	setSubWindowBotView(wTopLeft);

	glutSetWindow(wTopRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "front view");
	setSubWindowBotView(wTopRight);

	glutSetWindow(wBottomLeft);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	printText(-0.9, 0.8, "side view");

	setSubWindowBotView(wBottomLeft);
	glutSetWindow(wBottomRight);
	glClearColor(glSubWindowColor[0], glSubWindowColor[1], glSubWindowColor[2], 0.0f); // Set background color to white and opaque
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
	initializeLighting();

	wTopRight = glutCreateSubWindow(wMain, GAP + SubWindowWidth + GAP, GAP,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	wBottomLeft = glutCreateSubWindow(wMain, GAP, GAP + SubWindowHeight + GAP,
			SubWindowWidth, SubWindowHeight);
	glutDisplayFunc(repaintBotWindow);
	glutVisibilityFunc(vis);
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
	initializeLighting();

	wBottomRight = glutCreateSubWindow(wMain, GAP + SubWindowWidth + GAP,
					GAP + SubWindowHeight + GAP, SubWindowWidth, SubWindowHeight);
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

