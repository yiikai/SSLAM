#include "slammap.h"

namespace MySlam
{
	SLAMMap::SLAMMap()
	{}

	SLAMMap::~SLAMMap()
	{}
		
	void SLAMMap::insertPoints(mappoint::ptr point)
	{
		m_activeMapPoints.insert({point->getID(),point});
	}
	
	void SLAMMap::insertKeyFrame(frame::ptr frame)
	{
		m_activeFrames.insert({frame->getKeyID(),frame});
	}

 	const map<unsigned long,mappoint::ptr>& SLAMMap::getActiveMapPoints()
	{
		return m_activeMapPoints;
	}
	
	const map<unsigned long,frame::ptr>& SLAMMap::getActiveFrames()
	{
		return m_activeFrames;
	}

}
