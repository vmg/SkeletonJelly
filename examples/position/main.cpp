#include <list>
#include "GLUT/glut.h"
#include "OpenGL/gl.h"
#include "../../src/skeletonjelly.hpp"

#define WINDOW_X 800
#define WINDOW_Y 600

#define ROOM_X 4.0f // m
#define ROOM_Y 3.0f // m

#define GRID_SIZE 0.25f // 25 cm

#define SCALE(x) ((x) / 1000.0f)

bool g_running = false;
Kinect g_kinect;

char g_message[64] = {0};
char g_coords[64] = {0};
char g_leftHand[64] = {0};
char g_rightHand[64] = {0};

const KinectUser *g_userData = NULL;

static const char *MESSAGES[] =
{
	"Found user",
	"Lost user",
	"Pose detected",
	"Calibration started...",
	"TRACKING",
	"Calibration failed"
};

void kinect_status(Kinect *k, Kinect::CallbackType cb_type, XnUserID id, void *data)
{
	snprintf(g_message, 64, "User [%d]: %s", id, MESSAGES[cb_type]);
	printf("%s\n", g_message);

	if (cb_type == Kinect::CB_NEW_USER && id == 1)
	{
		g_userData = k->getUserData(id);
	}
}

void glPrintString(void *font, char *str)
{
	int i,l = strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}

void drawTracking()
{
	glOrtho(0, 1.0, 1.0, 0.0f, -1.0, 1.0);

	glPointSize(16.0f);
	glLineWidth(8.0f);

	if (g_userData)
	{
		const XnPoint3D *com = &g_userData->centerOfMass;

    	glColor3f(0.66, 0.33, 0.33);
		if (g_userData->status & Kinect::USER_TRACKING)
        	glColor3f(0.33, 0.66, 0.33);

		glBegin(GL_POINTS);
			glVertex3f(com->X, com->Y, 0.1f);
		glEnd();

		snprintf(g_coords, 64, "CoM: (%0.4f, %0.4f, %0.4f)\n", com->X, com->Y, com->Z);

        if (g_kinect.userStatus() & Kinect::USER_TRACKING)
		{
			const KinectUser::Hand *left = &g_userData->left;
			const KinectUser::Hand *right = &g_userData->right;

			if (left->tracked)
			{
				snprintf(g_leftHand, 64, "Left: (%0.4f, %0.4f, %s) %f %s\n",
						left->pos.X, left->pos.Y, 
						left->pos.Z >= 0.8f ? "PUSH" : "-",
						left->variance, 
						left->variance <= 0.1f ? "IDLE" : "");
				
				glColor3f(1.0f, 1.0f, 1.0f);
				glBegin(GL_POINTS);
					glVertex3f(left->pos.X, left->pos.Y, 0.1f);
				glEnd();

				glColor4f(0.81, 0.72, 0.66, 0.33);
				glLineWidth(4.0f);
				std::list<XnPoint3D>::const_iterator it;

				glBegin(GL_LINE_STRIP);
				for (it = left->history.begin(); it != left->history.end(); ++it)
					glVertex3f((*it).X, (*it).Y, 0.1f);
				glEnd();
			}
			else
			{
				snprintf(g_leftHand, 64, "Left: --");
			}
			
			if (right->tracked)
			{
				snprintf(g_rightHand, 64, "Right: (%0.4f, %0.4f, %s) %f %s\n", 
						right->pos.X, right->pos.Y, 
						right->pos.Z >= 0.8f ? "PUSH" : "-",
						right->variance,
						right->variance <= 0.1f ? "IDLE" : "");

				glColor3f(1.0f, 1.0f, 1.0f);
				glBegin(GL_POINTS);
					glVertex3f(right->pos.X, right->pos.Y, 0.1f);
				glEnd();

				glColor4f(0.81, 0.72, 0.66, 0.33);
				glLineWidth(4.0f);
				std::list<XnPoint3D>::const_iterator it;

				glBegin(GL_LINE_STRIP);
				for (it = right->history.begin(); it != right->history.end(); ++it)
					glVertex3f((*it).X, (*it).Y, 0.1f);
				glEnd();
			}
			else
			{
				snprintf(g_rightHand, 64, "Right: --");
			}
		}
	}
}

void drawHUD()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0, WINDOW_X, 0, WINDOW_Y, -1.0, 1.0);
	glDisable(GL_DEPTH_TEST); 

	glColor3f(1, 1, 1);

	glRasterPos2i(10, 10);
	glPrintString(GLUT_BITMAP_HELVETICA_18, g_message);

	glRasterPos2i(10, 30);
	glPrintString(GLUT_BITMAP_HELVETICA_18, g_coords);

	glRasterPos2i(10, 50);
	glPrintString(GLUT_BITMAP_HELVETICA_18, g_leftHand);

	glRasterPos2i(10, 70);
	glPrintString(GLUT_BITMAP_HELVETICA_18, g_rightHand);

	glEnable(GL_DEPTH_TEST); 
	glPopMatrix();
}


// this function is called each frame
void glutDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST); 
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	drawTracking();
	drawHUD();

	glutSwapBuffers();
}

void glutIdle()
{
	static int time = 0;

	int now = glutGet(GLUT_ELAPSED_TIME);

	g_kinect.tick(now - time);
	glutPostRedisplay();

	time = now;
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
	}
}
void glInit (int *pargc, char **argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_X, WINDOW_Y);
	glutCreateWindow ("SkeletonJelly Debug");
	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);

	glClearColor(1.0f, 153.0f / 255.0f, 0.0f, 1.0);

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

int main(int argc, char **argv)
{
	g_kinect.setEventCallback(kinect_status, NULL);
	g_kinect.setTicksPerSecond(30);
	g_kinect.init();

	glInit(&argc, argv);
	glutMainLoop();
}
