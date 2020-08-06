#include "slammap.h"

namespace MySlam
{
	SLAMMap::SLAMMap()
	{}

	SLAMMap::~SLAMMap()
	{}
		
	void SLAMMap::insertPoints(mappoint::ptr point)
	{
		m_activeMapPoints.push_back(point);
	}
	
	void SLAMMap::insertKeyFrame(frame::ptr frame)
	{
		m_activeFrames.push_back(frame);
	}

 	const vector<mappoint::ptr>& SLAMMap::getActiveMapPoints()
	{
		return m_activeMapPoints;
	}
	
	const vector<frame::ptr>& SLAMMap::getActiveFrames()
	{
		return m_activeFrames;
	}

}
