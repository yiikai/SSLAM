#ifndef FEATURE_H
#define FEATURE_H
#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
namespace MySlam
{
class frame;
class mappoint;
class feature
{
	public:
		using ptr = shared_ptr<feature>;
		feature(cv::Point2f& pt);
		~feature();
		cv::Point2f getPoint2f() //get pixel point
		{
			return m_pt2f;
		}

		Eigen::Matrix<double,2,1> getEigenPts()
		{
			Eigen::Matrix<double,2,1> x(m_pt2f.x,m_pt2f.y);
			return x;
		}
 
		std::weak_ptr<frame> m_frame;
		std::weak_ptr<mappoint> m_mapPt; //pixel in world corrdinate
		bool m_inlier = {true};
		bool m_isOnLeftImg = {false};
	private:
		cv::Point2f m_pt2f; //pixel points 2D
};
}

#endif
