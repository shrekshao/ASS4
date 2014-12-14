#pragma once


#include "Segment.h"

#define BALLJOINT_DOF (3)

#define BALLJOINT_ROTATE_EPSILON (0.2)


class SegmentBallJoint
	:public Segment
{
protected:
	//rotation, radian
	//float thetax;
	//float thetay;
	//float thetaz;

	Eigen::Matrix3f Rotation;

public:
	SegmentBallJoint(float seg_length);
	SegmentBallJoint(const Eigen::Vector3f & basepos, float seg_length);

	virtual Eigen::Matrix3f getRotationMatrix();
	virtual Eigen::Vector3f getChildrenJointPosition()const;
	virtual int getDOF()const;

	virtual void update(const Eigen::MatrixXf & dtheta, int & cid, const Eigen::Vector3f & v3_parent_end); 

	virtual void getJ(Eigen::Vector3f & p,Eigen::Matrix3f R, Eigen::MatrixXf & J,int cid);
	virtual void rootGetJ(Eigen::MatrixXf & J);

	virtual void getEndEffector(Eigen::Vector3f & p, Eigen::Matrix3f & R);

	virtual void draw();
};