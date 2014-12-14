#include <iostream>
#include "SegmentBallJoint.h"

#include<math.h>

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




SegmentBallJoint::SegmentBallJoint(float seg_length)
	:Segment(seg_length)
	//,thetax(0),thetay(0),thetaz(0)
	,Rotation(Eigen::Matrix3f::Identity())
{
	/*Eigen::Matrix3f R;

	R = 
		Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());

	v3_end = R * v3_end;*/
	v3_end = Rotation * v3_end;
}


SegmentBallJoint::SegmentBallJoint(const Eigen::Vector3f & basepos, float seg_length)
	:Segment(basepos,seg_length)
	//,thetax(0),thetay(0),thetaz(0)
	,Rotation(Eigen::Matrix3f::Identity())
{
	/*Eigen::Matrix3f R;

	R = 
		Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());

	v3_end = R * v3_end;*/
	v3_end = Rotation * v3_end;
}






Eigen::Matrix3f SegmentBallJoint::getRotationMatrix()
{
	//this coordinate to next coordiate
	//j -> j+1

	//no translate

	/*Eigen::Matrix3f R;



	R = 
		Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());

	


	return R;*/

	return Rotation;
}



Eigen::Vector3f SegmentBallJoint::getChildrenJointPosition()const
{
	//Eigen::Affine3f R;

	//R = 
	//	Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
	//	* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
	//	* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());


	//Eigen::Vector3f p(length,0,0);	//init position of end joint of this segment
	//return (R * p);




	return v3_end;
}


int SegmentBallJoint::getDOF()const
{
	return BALLJOINT_DOF;
}




void SegmentBallJoint::update(const Eigen::MatrixXf & dtheta, int & cid, const Eigen::Vector3f & v3_parent_end)
{
	//update position info
	v3_base = v3_parent_end;

	/*thetax += dtheta(cid);
	thetay += dtheta(cid+1);
	thetaz += dtheta(cid+2);*/
	float rx = dtheta(cid);
	float ry = dtheta(cid+1);
	float rz = dtheta(cid+2);

	/*if(rx > 0)
	{
		rx = ( rx<BALLJOINT_ROTATE_EPSILON ) ? rx : BALLJOINT_ROTATE_EPSILON;
	}
	else
	{
		rx = ( rx>(-BALLJOINT_ROTATE_EPSILON) ) ? rx : (-BALLJOINT_ROTATE_EPSILON);
	}

	if(ry > 0)
	{
		ry = ( ry<BALLJOINT_ROTATE_EPSILON ) ? ry : BALLJOINT_ROTATE_EPSILON;
	}
	else
	{
		ry = ( ry>(-BALLJOINT_ROTATE_EPSILON) ) ? ry : (-BALLJOINT_ROTATE_EPSILON);
	}
	
	if(rz > 0)
	{
		rz = ( rz<BALLJOINT_ROTATE_EPSILON ) ? rz : BALLJOINT_ROTATE_EPSILON;
	}
	else
	{
		rz = ( rz>(-BALLJOINT_ROTATE_EPSILON) ) ? rz : (-BALLJOINT_ROTATE_EPSILON);
	}*/


	//2
	/*const float gap = 0.1;
	if(rx > gap)
	{
		rx = 0;
	}
	if(ry > gap)
	{
		ry = 0;
	}
	if(rz > gap)
	{
		rz = 0;
	}*/



	//thetax += rx;
	//thetay += ry;
	//thetaz += rz;


	//
	//Eigen::Matrix3f R;

	//R = 
	//	Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
	//	* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
	//	* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());


	//Eigen::Vector3f p(length,0,0);	//init position of end joint of this segment
	//v3_end = (R * p) ;

	Eigen::Matrix3f Rd;	//delta

	Rd = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(ry,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());

	v3_end = Rd * v3_end;

	Rotation = Rd * Rotation;
	



	//test
	//cout<<v3_end<<endl<<endl;


	//update children

	cid += getDOF();
	//dtheta should be in prefix order
	for ( Segment* iter : vec_children )
	{
		iter->update(dtheta,cid,v3_end);
	}
}


void SegmentBallJoint::getJ(Eigen::Vector3f & p,Eigen::Matrix3f R, Eigen::MatrixXf & J,int cid)
{
	//p parent ask children
	//R children ask parent

	//traverse children first
	if(vec_children.size() > 0)
	{
		//TODO: multiple end effectors
		for ( Segment* iter : vec_children)
		{
			iter -> getJ(p, R*Rotation.transpose() , J, cid+getDOF());
			p = Rotation * p + v3_end;
		}
	}
	else
	{
		//leaf
		p = v3_end;
	}

	//no children, fill in J
	Eigen::Matrix3f Ji;

	Ji(0,0) = 0;
	Ji(1,0) = - p.z();
	Ji(2,0) = p.y();
	Ji(0,1) = p.z();
	Ji(1,1) = 0;
	Ji(2,1) = - p.x();
	Ji(0,2) = - p.y();
	Ji(1,2) = p.x();
	Ji(2,2) = 0;


	Ji = R * Ji;


	int ei = cid + getDOF();
	int ej = getDOF();
	for (int i = cid; i<ei ; i++)
	{
		for (int j = 0; j<ej; j++)
		{
			J(j,i) = Ji(j,i-cid);
		}
	}


	
}


void SegmentBallJoint::rootGetJ(Eigen::MatrixXf & J)
{
	int cid = 0;
	Eigen::Vector3f pp(0,0,0);	//?base
	
	getJ(pp,Eigen::Matrix3f::Identity(),J,cid);
}







void SegmentBallJoint::getEndEffector(Eigen::Vector3f & p, Eigen::Matrix3f & R)
{
	p += R * getChildrenJointPosition();

	
	if(vec_children.size() > 0)
	{
		/*R = 
			R * Eigen::AngleAxisf(-thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(-thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-thetaz, Eigen::Vector3f::UnitZ());*/
		/*R = 
			R * Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());*/
		R = R * Rotation;

		
		if(vec_children.size() == 1)
		{
			vec_children.at(0) -> getEndEffector(p,R);
		}
		else
		{
			//>1 tree structure

			//TODO (currently only support one end effector)
			for(Segment* iter : vec_children)
			{
				Eigen::Matrix3f Rn(R);
				iter -> getEndEffector(p,Rn);
			}
		}
		
	}
}





void SegmentBallJoint::draw()
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	//joint
	GLUquadricObj * pointTool;
	// create the rendering tool
	pointTool=gluNewQuadric();
	gluQuadricDrawStyle(pointTool,GLU_POINT);
	gluSphere(pointTool, 0.1, 200, 200);
	// delete the rendering tool
	gluDeleteQuadric(pointTool);


	//segment (line currently)
	glBegin(GL_LINE_LOOP);
	glNormal3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glNormal3f(0, 0, 1);
	glVertex3f(v3_end.x(), v3_end.y(), v3_end.z());
	glEnd();


	//draw children
	glTranslatef(v3_end.x(),v3_end.y(),v3_end.z());
	//glRotatef(thetax/M_PI*180,1,0,0);
	//glRotatef(thetay/M_PI*180,0,1,0);
	//glRotatef(thetaz/M_PI*180,0,0,1);
	const GLfloat Rm[] = {Rotation(0,0)
							,Rotation(1,0)
							,Rotation(2,0)
							,0
							,Rotation(0,1)
							,Rotation(1,1)
							,Rotation(2,1)
							,0
							,Rotation(0,2)
							,Rotation(1,2)
							,Rotation(2,2)
							,0

							,0,0,0,1
							};
	glMultMatrixf(Rm);
	
	for(Segment* iter : vec_children)
	{
		iter -> draw();
	}




	glPopMatrix();
}