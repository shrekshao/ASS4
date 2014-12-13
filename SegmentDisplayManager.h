#pragma once

#include "Segment.h"

//singleton

#define pSegmentDisplayManager SegmentDisplayManager::Instance()

class SegmentDisplayManager
{
private:
	vector<Segment*> vec_segment;
	
	
public:
	static SegmentDisplayManager* Instance();

	//SegmentDisplayManager();
	~SegmentDisplayManager();


	void addSegment(Segment* segment);
	void draw();
	void update(const Eigen::Vector3f & g);
	//void setupDrawCallback();
};