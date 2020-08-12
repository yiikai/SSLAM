#ifndef DATA_SETS_H
#define DATA_SETS_H

#include <iostream>
#include <string>
#include "camera.h"
#include "frame.h"

using namespace std;

namespace MySlam
{
	class DataSets
	{
		public:
			DataSets() = default;
			DataSets(string& as_dp);
			~DataSets();
			bool init();
			frame::ptr nextFrame();				
			string 	ms_DataPath;
			std::vector<Camera> mv_cameras;  //left and right
			Camera m_leftcamera;
			Camera m_rightcamera;
			int     m_imgcount = {0};
	};
}

#endif
