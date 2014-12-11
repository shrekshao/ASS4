#include "Segment.h"
#include "Eigen/SVD"

Segment::Segment(const Eigen::Vector3f & basepos, float seg_length)
	:totalDOF(0)
	,length(seg_length)
	,v3_base(basepos)
	,v3_end(length,0,0)
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
	

	Eigen::Vector3f p;	//current end effector
	{
		Eigen::Matrix3f R(Eigen::Matrix3f::Identity());
		getEndEffector(p,R);
	}
	
	//TODO: if can't reach the goal
	
	Eigen::Vector3f dp = g - p;

	if(dp.norm() > SEGMENT_EPSILON)
	{
		Eigen::MatrixXf J(3,totalDOF);
		Eigen::Matrix3f R(Eigen::Matrix3f::Identity());

		int cid = 0;
		getJ(p,R,J,cid);
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);

		//dp = J dtheta
		Eigen::VectorXf dtheta(totalDOF);
		dtheta = svd.solve(dp);

		cid = 0;
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