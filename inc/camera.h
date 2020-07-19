#ifndef CAMERA_H
#define CAMERA_H
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include "sophus/se3.hpp"
using namespace std;

namespace MySlam
{
	class Camera
	{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;	
			using ptr = std::shared_ptr<Camera>;
			Camera(){}
			~Camera(){}
			/********************************
				as_rotation: matrix (SO3 , t)
			*/
			void initCamera(double ad_fx, double ad_fy, double ad_cx, double ad_cy, double baseline, Sophus::SE3d& as_rotation )
			{
				m_fx = ad_fx;
				m_fy = ad_fy;
				m_cx = ad_cx;
				m_cy = ad_cy;
		 		m_baseline = baseline;
				m_pose = as_rotation;		
			}
			
			Eigen::Matrix<double,3,3> getK()
			{
				Eigen::Matrix<double,3,3> K;
				K << m_fx, 0, m_cx, 0 , m_fy, m_cy, 0,0,1;
				return K;
			}
			
			Eigen::Matrix<double, 3, 1> world2camera(const Eigen::Matrix<double, 3, 1>&w_point, const Sophus::SE3d& T_C_W);
			Eigen::Matrix<double, 2,1> camera2pixel(const Eigen::Matrix<double,3,1>& p_c);	
			Eigen::Matrix<double, 2, 1>  world2pixel(const Eigen::Matrix<double,3,1>& p_w, const Sophus::SE3d& T_c_w);

			string name;
			double m_fx = 0;
			double m_fy = 0;
			double m_cx = 0;
			double m_cy = 0;
			double m_baseline = 0;
			Sophus::SE3d   m_pose;
					
	};
}
#endif
