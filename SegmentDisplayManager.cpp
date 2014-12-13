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

SegmentDisplayManager::~SegmentDisplayManager()
{
	for(Segment* iter : vec_segment)
	{
		if(iter != NULL)
		{
			delete iter;
		}
	}
}


SegmentDisplayManager* SegmentDisplayManager::Instance()
{
	static SegmentDisplayManager instance;
	return &instance;
}

void SegmentDisplayManager::addSegment(Segment* segment)
{
	vec_segment.push_back(segment);
}




//SegmentDisplayManager* g_CurrentInstance;
//extern "C"
//void drawCallback()
//{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	
//	
//	//center = getCenter(teapot);
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	gluPerspective(45.0 /* * zoom*/, 1, 1.0, 200.0);
//	//gluPerspective(45.0 * zoom, (float)window_w / (float)window_h, -3*center.z(), 3*center.z());
//	glutPostRedisplay();
//	
//
//	//gluLookAt(pos_camera.x(),pos_camera.y(),pos_camera.z()
//	//	,center.x(),center.y(),center.z(),0,-1,0);
//
//	gluLookAt(0,0,-1
//		,0,0,0,0,-1,0);
//
//
//
//
//	g_CurrentInstance -> draw();
//
//
//
//
//	glutSwapBuffers();
//	
//}
//
//void SegmentDisplayManager::setupDrawCallback()
//{
//  ::g_CurrentInstance = this;
//  ::glutDisplayFunc(::drawCallback);
//}


void SegmentDisplayManager::draw()
{
	//TODO

	//test
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();


	float specularity = 1;
	float emissivity = 0;
	float shininess = 12;

	GLfloat materialColor[] = {1.0, 0.2, 0.2, 1.0};
	//The specular (shiny) component of the material
	GLfloat materialSpecular[] = {specularity, specularity, specularity, 1.0f};
	//The color emitted by the material
	GLfloat materialEmission[] = {emissivity, emissivity, emissivity, 1.0f};

	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, materialColor);
	glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
	glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
	glMaterialf(GL_FRONT, GL_SHININESS, shininess);


	/*glBegin(GL_POLYGON);
	glNormal3f(0, 0, -1);
	glVertex3f(0, 1, 0);
	glNormal3f(0, 0, -1);
	glVertex3f(0.5, -0.5, 0);
	glNormal3f(0, 0, -1);
	glVertex3f(-0.5, -0.5, 0);
	glEnd();*/

	for(Segment* iter : vec_segment)
	{
		Eigen::Matrix3f R(Eigen::Matrix3f::Identity());
		Eigen::Vector3f v3_origin(0,0,0);	//temp, should be iter->v3_base()
		//iter -> draw(R,v3_origin);
		iter -> draw();
	}


	glPopMatrix();
}



void SegmentDisplayManager::update(const Eigen::Vector3f & g)
{
	for(Segment* iter : vec_segment)
	{
		iter -> rootUpdate(g);
	}
}