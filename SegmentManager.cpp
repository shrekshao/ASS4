#include "SegmentManager.h"
#include "SegmentBallJoint.h"

SegmentManager::SegmentManager()
{
	//temp
	//root_segment = new SegmentBallJoint();
}




SegmentManager::~SegmentManager()
{
	if(root_segment != NULL)
	{
		delete root_segment;
	}
}



void SegmentManager::init()
{

}