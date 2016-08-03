/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 */
#include <stdio.h>

#include <windows.h>  // For MS Windows
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glut.h>  // GLUT, includes glu.h and gl.h
#include <GLUI2/glui2.h>

// Global Glui2 Handle
Glui2* GluiHandle = NULL;
// Window size
const int WindowWidth = 800;
const int WindowHeight = 600;


#define GAP 10
int SubWindowHeight= 100;
int SubWindowWidth= 100;

int wMain, wBottomRight, wBottomLeft, wTopRight, wTopLeft;


void printText(float x,float y, std::string text) {
    glColor4f(0.0f, 0.0f, 0.0f, 0.0f);
    // gluOrtho2D(0, SubWindowWidth, 0, SubWindowHeight);
    glRasterPos2f(x,y);
    glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)text.c_str());

}

void paintBot() {
	   glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
	      // Top face (y = 1.0f)
	      // Define vertices in counter-clockwise (CCW) order with normal pointing out
	      glColor3f(0.0f, 1.0f, 0.0f);     // Green
	      glVertex3f( 1.0f, 1.0f, -1.0f);
	      glVertex3f(-1.0f, 1.0f, -1.0f);
	      glVertex3f(-1.0f, 1.0f,  1.0f);
	      glVertex3f( 1.0f, 1.0f,  1.0f);

	      // Bottom face (y = -1.0f)
	      glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	      glVertex3f( 1.0f, -1.0f,  1.0f);
	      glVertex3f(-1.0f, -1.0f,  1.0f);
	      glVertex3f(-1.0f, -1.0f, -1.0f);
	      glVertex3f( 1.0f, -1.0f, -1.0f);

	      // Front face  (z = 1.0f)
	      glColor3f(1.0f, 0.0f, 0.0f);     // Red
	      glVertex3f( 1.0f,  1.0f, 1.0f);
	      glVertex3f(-1.0f,  1.0f, 1.0f);
	      glVertex3f(-1.0f, -1.0f, 1.0f);
	      glVertex3f( 1.0f, -1.0f, 1.0f);

	      // Back face (z = -1.0f)
	      glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	      glVertex3f( 1.0f, -1.0f, -1.0f);
	      glVertex3f(-1.0f, -1.0f, -1.0f);
	      glVertex3f(-1.0f,  1.0f, -1.0f);
	      glVertex3f( 1.0f,  1.0f, -1.0f);

	      // Left face (x = -1.0f)
	      glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	      glVertex3f(-1.0f,  1.0f,  1.0f);
	      glVertex3f(-1.0f,  1.0f, -1.0f);
	      glVertex3f(-1.0f, -1.0f, -1.0f);
	      glVertex3f(-1.0f, -1.0f,  1.0f);

	      // Right face (x = 1.0f)
	      glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	      glVertex3f(1.0f,  1.0f, -1.0f);
	      glVertex3f(1.0f,  1.0f,  1.0f);
	      glVertex3f(1.0f, -1.0f,  1.0f);
	      glVertex3f(1.0f, -1.0f, -1.0f);
	   glEnd();  // End of drawing color-cube
}

/* Handler for window-repaint event. Call back when the window first appears and
   whenever the window needs to be re-painted. */
void repaintBotWindow() {
   glutSetWindow(wMain);

   glClearColor(1.0f, 1.0f, 1.0f, 0.0f); // Set background color to white and opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer

   glutSetWindow(wTopLeft);
   glClearColor(0.9f, 0.9f, 0.9f, 0.0f); // Set background color to white and opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
   printText(-0.9,  0.8 , "top view");

   // paintBot();

   glutSetWindow(wTopRight);
   glClearColor(0.9f, 0.9f, 0.9f, 0.0f); // Set background color to white and opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
   printText(-0.9,  0.8, "front view");

   // paintBot();

   glutSetWindow(wBottomLeft);
   glClearColor(0.9f, 0.9f, 0.9f, 0.0f); // Set background color to white and opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
   printText(-0.9,  0.8, "side view");

   // paintBot();

   glutSetWindow(wBottomRight);
   glClearColor(0.9f, 0.9f, 0.9f, 0.0f); // Set background color to white and opaque
   glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
   printText(-0.9,  0.8, "3D");


   // paintBot();


   glFlush();  // Render now
}


void
vis(int visState)
{
  printf("VIS: win=%d, v=%d\n", glutGetWindow(), visState);
}
void
reshape(int w, int h)
{

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
  glutSetWindow(wTopLeft);
  glutPositionWindow(GAP, GAP);
  glutReshapeWindow(SubWindowWidth, SubWindowHeight);
  glViewport(0, 0, SubWindowWidth, SubWindowHeight);

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



int startBotUI(int argc, char** argv)
{

	glutInit(&argc, argv);                 			// Initialize GLUT
	// glutInitDisplayMode(GLUT_DOUBLE); 				// Enable double buffered mode

    wMain = glutCreateWindow("Bad Robot"); 					// Create a window with the given title

    glutDisplayFunc(repaintBotWindow);
    glutVisibilityFunc(vis);
    glutReshapeFunc(reshape);


    wTopLeft = glutCreateSubWindow(wMain, GAP, GAP, SubWindowWidth, SubWindowHeight);
    glutDisplayFunc(repaintBotWindow);
    glutVisibilityFunc(vis);
    wTopRight = glutCreateSubWindow(wMain, GAP+SubWindowWidth+GAP, GAP, SubWindowWidth, SubWindowHeight);
    glutDisplayFunc(repaintBotWindow);
    glutVisibilityFunc(vis);
    wBottomLeft = glutCreateSubWindow(wMain, GAP, GAP+SubWindowHeight+GAP, SubWindowWidth, SubWindowHeight);
    glutDisplayFunc(repaintBotWindow);
    glutVisibilityFunc(vis);
    wBottomRight = glutCreateSubWindow(wMain, GAP+SubWindowWidth+GAP, GAP+SubWindowHeight+GAP, SubWindowWidth, SubWindowHeight);
    glutDisplayFunc(repaintBotWindow);
    glutVisibilityFunc(vis);

    // glutInitDisplayMode(GLUT_RGB);
    // glutInitWindowSize(WindowWidth, WindowHeight); 	// Set the window's initial width & height
	glutInitWindowPosition(50, 50); 				// Position the window's initial top-left corner
	// glutDisplayFunc(repaintBotWindow); 				// Register display callback handler for window re-paint
    // glutVisibilityFunc(vis);						// Register visibility callback handler

	glutMainLoop();           						// Enter the infinitely event-processing loop

	// Suppress warning
    return 0;
}

