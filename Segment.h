#pragma once

#include "Eigen/Dense"
#include <vector>

using namespace std;

#define SEGMENT_EPSILON (0.01)
#define SEGMENT_MOVE_EPSILON (0.01)


class Segment
{
protected:
	//shape info

	//joint type
	//TODO
	//subclass


	//position

	//root joint for this segment, parent segment coordinate
	Eigen::Vector3f v3_base;	//12-11 seems to be useless?
								// the base point should always be at
								// the origin of the current coordinate

	//end point pos, local coordinate
	Eigen::Vector3f v3_end;	//?


	float length;

	int totalDOF;		//decide the size of Jacobian Matrix

	//children segment pointer
	//tree structure
	vector<Segment*> vec_children;




public:
	Segment(float seg_length);
	Segment(const Eigen::Vector3f & basepos, float seg_length);
	~Segment();


	virtual Eigen::Matrix3f getRotationMatrix() = 0;
	virtual Eigen::Vector3f getChildrenJointPosition()const = 0;
	virtual int getDOF()const = 0;

	virtual void update(const Eigen::MatrixXf & dtheta, int & cid, const Eigen::Vector3f & v3_parent_end) = 0;
	
	virtual void getJ(const Eigen::Vector3f & p, Eigen::Matrix3f & R, Eigen::MatrixXf & J,int & cid) = 0;






	bool rootUpdate(const Eigen::Vector3f & g);
	//Eigen::MatrixXf rootGetJ();
	//int getChildrenNum();
	int initTotalDOF();

	//TODO: tree structure needs vector<Vector3f> p
	virtual void getEndEffector(Eigen::Vector3f & p,Eigen::Matrix3f & R) = 0;





	void addChildrenSegment(Segment* seg);
	void setBasePosition(const Eigen::Vector3f v);


	//intuitive
	//void draw(Eigen::Matrix3f & R, Eigen::Vector3f & v3_origin_in_world);

	//use opengl transformation
	virtual void draw() = 0;
};