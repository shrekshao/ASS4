#include <iostream>
#include "Segment.h"
#include "Eigen/SVD"





Segment::Segment(float seg_length)
	:totalDOF(0)
	,length(seg_length)
	,v3_base(0,0,0)
	,v3_end(seg_length,0,0)
{

}



Segment::Segment(const Eigen::Vector3f & basepos, float seg_length)
	:totalDOF(0)
	,length(seg_length)
	,v3_base(basepos)
	,v3_end(seg_length,0,0)
{
}

Segment::~Segment()
{
	for(Segment* iter : vec_children)
	{
		if(iter != NULL)
		{
			delete iter;
		}
	}
}



bool Segment::rootUpdate(const Eigen::Vector3f & g)
{
	//Eigen::Vector3f dis = g - v3_base;
	

	Eigen::Vector3f p(0,0,0);	//current end effector
	{
		Eigen::Matrix3f R(Eigen::Matrix3f::Identity());
		getEndEffector(p,R);
	}
	
	//test
	//cout<<p<<endl;


	//TODO: if can't reach the goal
	
	//?? p + root.v3_base => world coordinate
	Eigen::Vector3f dp = g - p;

	if(dp.norm() > SEGMENT_EPSILON)
	{
		Eigen::MatrixXf J(3,totalDOF);
		//Eigen::Matrix3f R(Eigen::Matrix3f::Identity());

		
		//getJ(pp,Rotation,J,cid);
		rootGetJ(J);
		
		//test
		//cout<<J<<endl<<endl;

		Eigen::JacobiSVD<Eigen::MatrixXf> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

		//dp = J dtheta
		Eigen::VectorXf dtheta(totalDOF);
		//?
		float dplength = dp.norm();
		if(dplength > SEGMENT_MOVE_EPSILON)
		{
			dp = dp / dplength * SEGMENT_MOVE_EPSILON;
		}
		//dp = dp.normalized() * SEGMENT_MOVE_EPSILON;

		//12-12:the second dtheta is wrong
		dtheta = svd.solve(dp);

		//test
		//cout<<dtheta<<endl<<endl;



		int cid = 0;
		update(dtheta, cid, v3_base);



		return false;
	}
	

	
	return true;
}


//int Segment::getChildrenNum()
//{
//	if ( vec_children.size() == 0)
//	{
//		return 1;
//	}
//	else
//	{
//		int num_children = 0;
//		for(Segment* iter : vec_children)
//		{
//			num_children += (iter -> getChildrenNum());
//		}
//		return (num_children + 1);
//	}
//}


int Segment::initTotalDOF()
{
	if ( vec_children.size() == 0)
	{
		totalDOF = getDOF();
		return totalDOF;
	}
	else
	{
		totalDOF = 0;
		for(Segment* iter : vec_children)
		{
			totalDOF += (iter -> initTotalDOF());
		}
		totalDOF += getDOF();
		return totalDOF;
	}
}







//Eigen::MatrixXf Segment::rootGetJ()
//{
//	//totalDOF should have been initalized
//	//before calling this function
//
//	Eigen::MatrixXf J(3,totalDOF);
//
//	Eigen::Matrix3f R(Eigen::Matrix3f::Identity());
//
//	Eigen::Vector3f p;	//current end effector
//
//	getEndEffector(p);
//
//
//	getJ(p,R,J,0);
//
//
//	return J;
//
//}



void Segment::addChildrenSegment(Segment* seg)
{
	vec_children.push_back(seg);
	seg->setBasePosition(this->v3_end);
}

void Segment::setBasePosition(const Eigen::Vector3f v)
{
	v3_base = v;
}




//void Segment::draw(Eigen::Matrix3f & R, Eigen::Vector3f & v3_origin_in_world)
//{
//	//draw itself
//	
//	glMatrixMode(GL_MODELVIEW);
//	glPushMatrix();
//	glLoadIdentity();
//	
//	glTranslatef(v3_origin_in_world.x(), v3_origin_in_world.y(), v3_origin_in_world.z());
//
//	GLUquadricObj * pointTool;
//	// create the rendering tool
//	pointTool=gluNewQuadric();
//	gluQuadricDrawStyle(pointTool,GLU_POINT);
//	gluSphere(pointTool, 0.1, 200, 200);
//	// delete the rendering tool
//	gluDeleteQuadric(pointTool);
//	
//	glPopMatrix();
//	
//	
//	glBegin(GL_LINE_LOOP);
//	glNormal3f(0, 0, 1);
//	glVertex3f(v3_origin_in_world.x(), v3_origin_in_world.y(), v3_origin_in_world.z());
//	glNormal3f(0, 0, 1);
//	v3_origin_in_world = v3_origin_in_world + (R * v3_end);
//	glVertex3f(v3_origin_in_world.x(), v3_origin_in_world.y(), v3_origin_in_world.z());
//	glEnd();
//
//	
//	
//
//
//	//draw children
//	int num_children = vec_children.size();
//	if(num_children == 1)
//	{
//		R = (getRotationMatrix()) * R;
//		for(Segment* iter : vec_children)
//		{
//			iter -> draw(R,v3_origin_in_world);
//		}
//	}
//	else if(num_children > 1)
//	{
//		for(Segment* iter : vec_children)
//		{
//			Eigen::Matrix3f Rn(R);
//			Eigen::Vector3f v3_endeffector(v3_origin_in_world);
//			iter -> draw(Rn,v3_endeffector);
//		}
//	}
//}





