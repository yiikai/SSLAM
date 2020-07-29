#include "mappoint.h"

namespace MySlam
{
	mappoint::mappoint(cv::Mat pose):m_pose(pose)
	{
		
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
}

