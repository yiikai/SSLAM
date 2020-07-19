#include "map.h"

namespace MySlam
{
	map::map()
	{}

	map::~map()
	{}
		
	void map::insertPoints(mappoint::ptr point)
	{
		m_3dpoints.push_back(point);
	}
}
