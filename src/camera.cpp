#include "camera.h"

namespace MySlam
{
	Eigen::Matrix<double,3,1> Camera::world2camera(const Eigen::Matrix<double, 3, 1>& w_point, const Sophus::SE3d& T_C_W)
	{
		return m_pose * T_C_W * w_point;
	}

	Eigen::Matrix<double, 2,1> Camera::camera2pixel(const Eigen::Matrix<double,3,1>& p_c)
	{
		return Eigen::Matrix<double, 2, 1>(
					m_fx*p_c(0,0)/ p_c(2,0) + m_cx,
					m_fy*p_c(1,0)/p_c(2,0) + m_cy
				);
	}

	Eigen::Matrix<double, 2, 1>  Camera::world2pixel(const Eigen::Matrix<double,3,1>& p_w, const Sophus::SE3d& T_c_w)
	{
		return camera2pixel(world2camera(p_w,T_c_w));
	}

	
	Eigen::Matrix<double,3,1> Camera::pixel2camera(const Eigen::Matrix<double,2,1>& p_p, double depth)
	{
		return Eigen::Matrix<double,3,1>(
				(p_p(0,0) - m_cx) * depth / m_fx,
				(p_p(1,0) - m_cy) * depth / m_fy,
				depth
				); 
	}
}

