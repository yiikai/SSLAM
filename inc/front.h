#ifndef FRONT_H
#define FRONT_H

#include "DataSets.h"
#include "frame.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "map.h"
namespace MySlam
{

	class frontEnd
	{
		public:
			frontEnd(DataSets& a_sets);
			~frontEnd(){}
			void addFrame(frame::ptr newframe);
		private:
			typedef enum
			{
				E_INITING,
				E_TRACKING,
				E_RESET
			}E_STATUS;

			void detectedFeature();
			void findFeatureInRight(); //find match point in right camera img.
			void calcMapPoint();
			int EstimateCurrentPose();		
			void addObservationToMapPoint();
			void trackingLastFrame();
			Eigen::Matrix<double,2,1> toVec2(const cv::Point2f& val){
	return Eigen::Matrix<double,2,1>(val.x, val.y);
}
			void insertKeyFrome();
		private:
			frame::ptr m_currentFrame;
			frame::ptr m_lastFrame;
			frontEnd::E_STATUS m_status = {E_INITING};
			cv::Ptr<cv::GFTTDetector> mp_detector;
			DataSets m_sets;
			std::vector<cv::Point2f> m_matchL;
			std::vector<cv::Point2f> m_matchR; 
			Sophus::SE3d m_relative_motion;   //假设的当前frame相对于上一个frame的位姿
			map m_map;  //slam的最终地图数据
			int m_tracking_inlier = {0};
			int m_num_feature_tracking = {20};
			int m_num_feature_need_for_keyframe = {80};
	};
}
#endif
