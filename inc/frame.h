#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include "feature.h"
#include "camera.h"
#include <sophus/se3.hpp>
namespace MySlam
{
	class frame
	{
		public:
			typedef std::shared_ptr<frame> ptr; 
			frame(){}
			~frame(){}

			inline void showFrameWithKeyPoint(std::vector<cv::KeyPoint>& points)
			{
				//covert keypoint to point2f
				std::vector<cv::Point2f> lv_2fpoints;
				cv::KeyPoint::convert(points, lv_2fpoints);
				for(auto point:lv_2fpoints)
				{
					cv::circle(m_leftImg,point,2,cv::Scalar(0,255, 255),-1);
				}
				imshow( "Display window", m_leftImg);
			}

			void setPose(const Sophus::SE3d& pose)
			{
				m_pose = pose;
			}

			Sophus::SE3d getPose()
			{
				return m_pose;
			}
	
			cv::Mat m_leftImg;
			std::vector<feature::ptr> m_leftKPs;
			cv::Mat m_rightImg; 	
			std::vector<feature::ptr> m_rightKPs;
		private:
			Sophus::SE3d m_pose;
	};
}
#endif
