#ifndef MAPPOINT_H
#define MAPPOINT_H
#include <utility>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <memory>
#include <list>
#include "feature.h"
using namespace std; 
namespace MySlam
{
	class mappoint
	{
		public:
			using ptr = shared_ptr<mappoint>;
			mappoint(cv::Mat pose);
			~mappoint();
			cv::Mat getPose();
			Eigen::Matrix<double,3,1> getEigenPose()
			{
				Eigen::Matrix<double,3,1> pose;
				cv::cv2eigen(m_pose,pose);
				return pose;	
			}
			void addObservation(feature::ptr feat);
		private:
			cv::Mat m_pose;  //world corrdinate
			list<feature::ptr> m_observations;	
	};
}

#endif
