#include "mappoint.h"

namespace MySlam
{

	long mappoint::totoalID = 0;
	mappoint::mappoint(cv::Mat pose):m_pose(pose)
	{
		m_id = totoalID++;	
	}

	mappoint::~mappoint()
	{

	}
	cv::Mat mappoint::getPose()
	{
		return m_pose;
	}

	void mappoint::addObservation(feature::ptr feat)
	{
		m_observations.push_back(feat);
	}

	void mappoint::removeObservation(feature::ptr feat)
	{
		list<feature::ptr>::iterator itr;
		for(itr = m_observations.begin(); itr != m_observations.end();  itr++)
		{
			if(itr->get() == feat.get())
			{
				m_observations.erase(itr);
			}
		}		
	}
		
	bool mappoint::isOutLier()
	{
		return m_outlier ? true:false;
	}
}

