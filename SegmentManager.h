#pragma once

#include "Segment.h"

class SegmentManager
{
protected:
	//vector<Segment*> vec_segments;

	Segment* root_segment;

	int num_total_seg;
	int max_level;
public:
	SegmentManager();
	~SegmentManager();

	void init();
};