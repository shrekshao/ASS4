#include "SegmentBallJoint.h"

SegmentBallJoint::SegmentBallJoint(const Eigen::Vector3f & basepos, float seg_length)
	:Segment(basepos,seg_length)
	,thetax(0),thetay(0),thetaz(0)
{
}






Eigen::Affine3f SegmentBallJoint::getTransformMatrix()
{
	//X(j-1) <- j

	Eigen::Affine3f X;


	//translate
	X = Eigen::Translation3f(-v3_base);

	X *= 
		Eigen::AngleAxisf(-thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(-thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-thetaz, Eigen::Vector3f::UnitZ());




	return X;
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

	thetax += dtheta(0,cid);
	thetay += dtheta(1,cid+1);
	thetaz += dtheta(2,cid+2);



	
	Eigen::Matrix3f R;

	R = 
		Eigen::AngleAxisf(thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(thetaz, Eigen::Vector3f::UnitZ());


	Eigen::Vector3f p(length,0,0);	//init position of end joint of this segment
	v3_end = (R * p) ;




	//update children

	cid += getDOF();
	//dtheta should be in prefix order
	for ( Segment* iter : vec_children )
	{
		iter->update(dtheta,cid,v3_end);
	}
}


void SegmentBallJoint::getJ(const Eigen::Vector3f & p,Eigen::Matrix3f & R, Eigen::MatrixXf & J,int & cid)
{


	Eigen::Matrix3f Ji;

	Ji(0,0) = 0;
	Ji(1,0) = - p.z();
	Ji(2,0) = p.y();
	Ji(0,1) = p.z();
	Ji(1,1) = 0;
	Ji(2,1) = - p.x();
	Ji(0,1) = - p.y();
	Ji(1,1) = p.x();
	Ji(2,1) = 0;


	Ji *= R;


	int ei = cid + getDOF();
	int ej = getDOF();
	for (int i = cid; i<ei ; i++)
	{
		for (int j = 0; j<ej; j++)
		{
			J(j,i) = Ji(j,i-cid);
		}
	}


	cid += getDOF();
	R =	
		R * Eigen::AngleAxisf(-thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(-thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-thetaz, Eigen::Vector3f::UnitZ());

	


	for ( Segment* iter : vec_children)
	{
		iter -> getJ(p, R , J,cid);
	}
	
}


void SegmentBallJoint::getEndEffector(Eigen::Vector3f & p, Eigen::Matrix3f & R)
{
	p += R * getChildrenJointPosition();

	
	if(vec_children.size() > 0)
	{
		//R(j)<-(j+1)
		R = 
			R * Eigen::AngleAxisf(-thetax, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(-thetay,  Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-thetaz, Eigen::Vector3f::UnitZ());

		
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