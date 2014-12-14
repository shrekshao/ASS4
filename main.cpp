#include<iostream>
#include "SegmentBallJoint.h"
#include "SegmentDisplayManager.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif


using namespace std;

int window_w = 600;
int window_h = 600;

float zoom = 1.5;
//float camera_dist = 10;
//Eigen::Vector3f pos_camera(0,0,-10);
////for mouse
//int lastx=0;
//int lasty=0;


float t = 0;
float t_step = 0.005;
Eigen::Vector3f goal(0,1.5,0);


void initRendering() 
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);
	
	//glShadeModel(GL_FLAT);
	
	glShadeModel(GL_SMOOTH);
	glDisable(GL_COLOR_MATERIAL);



	GLfloat ambientLight[] = {0.2f, 0.2f, 0.2f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);
	
	GLfloat lightColord[] = {0.7f, 0.7f, 0.7f, 1.0f};
	GLfloat lightColors[] = {1.0f, 1.0f, 1.0f, 1.0f};
	GLfloat lightPos[] = {1000, 1000, 1000, 1.0f};
	//Diffuse (non-shiny) light component
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColord);
	//Specular (shiny) light component
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColors);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	
}


void handleResize(int w, int h) 
{
	window_h = h;
	window_w = w;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0 *zoom, (float)w / (float)h, 1.0, 200.0);
}




void drawGoal()
{
	float specularity = 1;
	float emissivity = 0;
	float shininess = 12;

	GLfloat materialColor[] = {1.0, 1.0, 0.2, 1.0};
	//The specular (shiny) component of the material
	GLfloat materialSpecular[] = {specularity, specularity, specularity, 1.0f};
	//The color emitted by the material
	GLfloat materialEmission[] = {emissivity, emissivity, emissivity, 1.0f};

	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);
	glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
	glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	//draw goal
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(goal.x(),goal.y(),goal.z());
	GLUquadricObj * pointTool;
	// create the rendering tool
	pointTool=gluNewQuadric();
	gluQuadricDrawStyle(pointTool,GLU_POINT);
	gluSphere(pointTool, 0.05, 200, 200);
	// delete the rendering tool
	gluDeleteQuadric(pointTool);
	glPopMatrix();
}






extern "C"
void drawScene() 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	
	//center = getCenter(teapot);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0 * zoom, (float)window_w / (float)window_h, 1.0, 200.0);
	//gluPerspective(45.0 * zoom, (float)window_w / (float)window_h, -3*center.z(), 3*center.z());
	
	

	//gluLookAt(pos_camera.x(),pos_camera.y(),pos_camera.z()
	//	,center.x(),center.y(),center.z(),0,-1,0);

	gluLookAt(0,0,5
		,0,0,0,0,1,0);



	
	drawGoal();
	
	pSegmentDisplayManager->draw();
	//mng -> draw();



	glFlush();
	glutSwapBuffers();
}





void parameterEquation()
{
	//2c,1s,1s is a successful case
	goal.x() = 2 * cos(t);
	goal.y() = 1 * sin(t);
	goal.z() = 1 * sin(t);
	//goal.x() = 3.0 *cos(t);
	//goal.y() = 3.0 * sin(t);
	//goal.z() = 0;
}

//Called every 25 milliseconds
void myFrameMove() 
{
#ifdef _WIN32
  Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif

  parameterEquation();
  t += t_step;

  pSegmentDisplayManager->update(goal);

  
  

  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}



unsigned char Buttons[3] = {0};
void Mouse(int b,int s,int x,int y)
{
	/*lastx=x;
	lasty=y;
	switch(b)
	{
	case GLUT_LEFT_BUTTON:
		Buttons[0] = ((GLUT_DOWN==s)?1:0);
		break;
	case GLUT_MIDDLE_BUTTON:
		Buttons[1] = ((GLUT_DOWN==s)?1:0);
		break;
	case GLUT_RIGHT_BUTTON:
		Buttons[2] = ((GLUT_DOWN==s)?1:0);
		break;
	default:
		break;		
	}
	glutPostRedisplay();*/
}

void Motion(int x,int y)
{
	/*int diffx=x-lastx;
	int diffy=y-lasty;
	lastx=x;
	lasty=y;

	if( Buttons[0] && Buttons[1] )
	{
		zoom -= (float) 0.05f * diffx;
	}
	else
	{
		
	}
	glutPostRedisplay();*/
}

void handleKeypress(unsigned char key, int x, int y) {
	switch (key) 
	{
		case 27: //Escape key
			exit(0);
			break;
		
		default:
			break;
	}
}


void SpecialKeys(int key, int x, int y)
{
	int state;
	state=glutGetModifiers();
	if (state != GLUT_ACTIVE_SHIFT)
	{
		if (key == GLUT_KEY_LEFT)
		{
			
			//glutPostRedisplay();
		}
		if (key == GLUT_KEY_RIGHT)
		{
			
		}
		if (key == GLUT_KEY_UP)
		{
			
		}
		if (key == GLUT_KEY_DOWN)
		{
			
		}
	} 
	else 
	{
		
	}
}



int main(int argc, char* argv[])
{

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(window_w, window_h);
	
	glutCreateWindow("ASS4");
	initRendering();
	

	//init
	//test: create a chain
	SegmentBallJoint* root;
	SegmentBallJoint* cur;
	SegmentBallJoint* next;
	root = new SegmentBallJoint(1.0f);
	cur = root;

	for (int i = 0; i < 3; i++)
	{
		next = new SegmentBallJoint(0.8 - i*0.2);
		cur -> addChildrenSegment(next);
		cur = next;
		next = NULL;
	}

	root->initTotalDOF();
	pSegmentDisplayManager->addSegment(root);







	//pSegmentDisplayManager->setupDrawCallback();
	glutDisplayFunc(drawScene);
	glutSpecialFunc(SpecialKeys);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);

	glutMouseFunc(Mouse);
	glutMotionFunc(Motion);

	glutIdleFunc(myFrameMove);


	//test
	/*SegmentBallJoint* root;
	root = new SegmentBallJoint(Eigen::Vector3f(0,0,0),1.0f);

	root -> addChildrenSegment(new SegmentBallJoint(1.0f) );

	delete root;*/

	glutMainLoop();


	
	return 0;
}


