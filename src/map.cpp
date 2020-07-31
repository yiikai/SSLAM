#include "map.h"

namespace MySlam
{
	map::map()
	{}

	map::~map()
	{}
		
	void map::insertPoints(mappoint::ptr point)
	{
		m_activeMapPoints.push_back(point);
	}
	
	void map::insertKeyFrame(frame::ptr frame)
	{
		m_activeFrames.push_back(frame);
	} 
}
