#ifndef FEATURE_H
#define FEATURE_H
#include <memory>
#include <opencv2/opencv.hpp>

using namespace std;
namespace MySlam
{
class mappoint;
class feature
{
	public:
		using ptr = shared_ptr<feature>;
		feature(cv::Point2f& pt);
		~feature();
		cv::Point2f getPts() //get pixel point
		{
			return m_pt;
		}

		std::weak_ptr<mappoint> m_mapPt; //pixel in world corrdinate
		bool m_inlier = {true};
	private:
		cv::Point2f m_pt; //pixel points 2D
};
}

#endif
