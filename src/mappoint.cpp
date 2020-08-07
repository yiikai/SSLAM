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
	
	bool mappoint::isOutLier()
	{
		return m_outlier ? true:false;
	}
}

