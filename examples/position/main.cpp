#include <list>
#include "GL/glut.h"
#include "Gl/gl.h"
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

const Kinect_UserData *g_userData = NULL;

#define TRACE_LENGTH 16
std::list<XnPoint3D> g_traceLeft;
std::list<XnPoint3D> g_traceRight;

enum Scene
{
	SCENE_POSITION = 0,
	SCENE_ARMS,
	SCENE_MAX
};

Scene g_drawScene = SCENE_POSITION;

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
	sprintf_s(g_message, 64, "User [%d]: %s", id, MESSAGES[cb_type]);
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

void drawPosition()
{
	glOrtho(-ROOM_X / 2.0f, +ROOM_X / 2.0f, ROOM_Y, 0.0f, -1.0, 1.0);
	glColor3f(1.0f, 1.0f, 1.0f);
	glLineWidth(1.0f);

	glBegin(GL_LINES);

	float x;

	int rows = (ROOM_Y) / GRID_SIZE;
	x = 0.0f;
	for (int i = 0; i < rows; ++i) 
	{
		if (i & 1)
			glColor4f(0.65, 0.65, 0.65, 0.5);
		else
        	glColor3f(1.0f, 1.0f, 1.0f);

		glVertex2f(-ROOM_X, x);
		glVertex2f(+ROOM_Y, x);
		x += GRID_SIZE;
	}

	int cols = (ROOM_X) / GRID_SIZE;
	x = -ROOM_X / 2;
	for (int i = 0; i < cols; ++i) 
	{
		if (i & 1)
			glColor4f(0.65, 0.65, 0.65, 0.5);
		else
        	glColor3f(1.0f, 1.0f, 1.0f);

		glVertex2f(x, 0.0f);
		glVertex2f(x, +ROOM_Y);
		x += GRID_SIZE;
	}

	glEnd();

	glColor3f(0.33, 0.33, 0.33);
	glTranslatef(0.0f, 0.0f, 0.0f);
	glutSolidSphere(0.2f, 16, 4);

	if (g_userData)
	{
		const XnPoint3D *p = &g_userData->world.centerOfMass;

		glTranslatef(SCALE(p->X), SCALE(p->Z), 0.0f);

    	glColor3f(0.66, 0.33, 0.33);
		if (g_userData->status & Kinect::USER_TRACKING)
        	glColor3f(0.33, 0.66, 0.33);

    	glutSolidSphere(0.1f, 16, 4);
		sprintf_s(g_coords, 64, "(%0.4f, %0.4f, %0.4f)\n", p->X, p->Y, p->Z);
	}
}

void drawArms()
{
	XnUInt32XYPair resolution = g_kinect.getDepthResolution();
	glOrtho(0, resolution.X, resolution.Y, 0.0f, -1.0, 1.0);
	glPointSize(8.0f);

	if (g_userData)
	{
		const XnPoint3D *com = &g_userData->screen.centerOfMass;

    	glColor3f(0.66, 0.33, 0.33);
		if (g_userData->status & Kinect::USER_TRACKING)
        	glColor3f(0.33, 0.66, 0.33);

		glBegin(GL_POINTS);
			glVertex3f(com->X, com->Y, 0.1f);
		glEnd();

		sprintf_s(g_coords, 64, "(%0.4f, %0.4f, %0.4f)\n", com->X, com->Y, com->Z);

        if (g_kinect.userStatus() & Kinect::USER_TRACKING)
		{
			const XnPoint3D *joints = g_userData->screen.joints;
			const XnPoint3D *leftHand = &g_userData->screen.joints[XN_SKEL_LEFT_HAND];
			const XnPoint3D *rightHand = &g_userData->screen.joints[XN_SKEL_RIGHT_HAND];

        	glColor3f(1.0f, 1.0f, 1.0f);
        	glLineWidth(8.0f);
			glBegin(GL_LINES);
				glVertex3f(joints[XN_SKEL_LEFT_SHOULDER].X, joints[XN_SKEL_LEFT_SHOULDER].Y, 0.1f);
				glVertex3f(joints[XN_SKEL_LEFT_ELBOW].X, joints[XN_SKEL_LEFT_ELBOW].Y, 0.1f);

				glVertex3f(joints[XN_SKEL_LEFT_ELBOW].X, joints[XN_SKEL_LEFT_ELBOW].Y, 0.1f);
				glVertex3f(joints[XN_SKEL_LEFT_HAND].X, joints[XN_SKEL_LEFT_HAND].Y, 0.1f);


				glVertex3f(joints[XN_SKEL_RIGHT_SHOULDER].X, joints[XN_SKEL_RIGHT_SHOULDER].Y, 0.1f);
				glVertex3f(joints[XN_SKEL_RIGHT_ELBOW].X, joints[XN_SKEL_RIGHT_ELBOW].Y, 0.1f);

				glVertex3f(joints[XN_SKEL_RIGHT_ELBOW].X, joints[XN_SKEL_RIGHT_ELBOW].Y, 0.1f);
				glVertex3f(joints[XN_SKEL_RIGHT_HAND].X, joints[XN_SKEL_RIGHT_HAND].Y, 0.1f);
			glEnd();

			if (g_traceLeft.empty() || g_traceLeft.back().X != leftHand->X || g_traceLeft.back().Y != leftHand->Y)
			{
				g_traceLeft.push_back(*leftHand);
				if (g_traceLeft.size() > TRACE_LENGTH)
					g_traceLeft.pop_front();
			}

			if (g_traceRight.empty() || g_traceRight.back().X != rightHand->X || g_traceRight.back().Y != rightHand->Y)
			{
				g_traceRight.push_back(*rightHand);
				if (g_traceRight.size() > TRACE_LENGTH)
					g_traceRight.pop_front();
			}

			glColor4f(0.81, 0.72, 0.66, 0.33);
        	glLineWidth(4.0f);
			std::list<XnPoint3D>::iterator it;

			glBegin(GL_LINE_STRIP);
			for (it = g_traceLeft.begin(); it != g_traceLeft.end(); ++it)
				glVertex3f((*it).X, (*it).Y, 0.1f);
			glEnd();

			glBegin(GL_LINE_STRIP);
			for (it = g_traceRight.begin(); it != g_traceRight.end(); ++it)
				glVertex3f((*it).X, (*it).Y, 0.1f);
			glEnd();
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

	switch (g_drawScene)
	{
	case SCENE_POSITION:
    	drawPosition();
		break;

	case SCENE_ARMS:
		drawArms();
		break;

	default:
		break;
	}

	drawHUD();

	glutSwapBuffers();
}

void glutIdle()
{
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);

	case ' ':
		g_drawScene = (Scene)(((int)g_drawScene + 1) % (int)SCENE_MAX);
		break;
	}
}
void glInit (int *pargc, char **argv)
{
	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(WINDOW_X, WINDOW_Y);
	glutCreateWindow ("Positioning Sample");
	//glutFullScreen();
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
	g_kinect.init();
	g_kinect.runThreaded();

	glInit(&argc, argv);
	glutMainLoop();
}
