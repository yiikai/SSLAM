#include <iostream>
#include <opencv2/opencv.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_impl.h>
#include <opencv2/features2d/features2d.hpp>
#include "sophus/se3.hpp"
#include "inc/DataSets.h"
#include "front.h"
#include "slammap.h"
using namespace MySlam;
using namespace cv;
int main(int argc, char* * argv)
{

    string img1path = "/home/yiikai/Develop/MySlam/resource/21/image_0/000000.png";
    string img2path = "/home/yiikai/Develop/MySlam/resource/21/image_0/000001.png";
    cv::Mat img1 = cv::imread(img1path);
    cv::Mat img2 = cv::imread(img2path);
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1,keypoints2;
    cv::Mat descriptors1,descriptors2;
    orb->detectAndCompute(img1,cv::Mat(),keypoints1,descriptors1);
    orb->detectAndCompute(img2,cv::Mat(),keypoints2,descriptors2);
    cv::BFMatcher matcher(NORM_HAMMING);
    std::vector<DMatch> mathces;
	matcher.match(descriptors1, descriptors2, mathces);
    //drawKeypoints(img1, keypoints, img1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    double min_dist = 10000, max_dist = 0;
    for(int i = 0; i < descriptors1.rows; i++)
    {
        if(mathces[i].distance < min_dist)
            min_dist = mathces[i].distance;
        if(mathces[i].distance > max_dist)
            max_dist = mathces[i].distance;
    }
    cout<<"max distance: "<<max_dist<<endl;
    cout<<"min distance: "<<min_dist<<endl;
    std::vector<DMatch> filterMathces;
    for(int i = 0; i < descriptors1.rows; i++)
    {
        if(mathces[i].distance <= max(2 * min_dist, 100.0))
            filterMathces.push_back(mathces[i]);
    }
	Mat matchMat;
	drawMatches(img1, keypoints1, img2, keypoints2, filterMathces, matchMat);
    imshow("match feature",matchMat);
    cv::waitKey(0);
#if 0
	string path = "/home/yiikai/Develop/MySlam/resource/21";
	DataSets sets(path);
	sets.init();
	SLAMMap::ptr lp_newmap = make_shared<SLAMMap>();
	becken::ptr l_becken = make_shared<becken>(sets);
    Viewer::ptr l_viewer = make_shared<Viewer>();
	frontEnd l_front(sets, lp_newmap);
	l_front.addBecken(l_becken);
    l_viewer->SetMap(lp_newmap);
    l_front.setViewer(l_viewer);
	while(1)
	{
        frame::ptr lf = sets.nextFrame();
        if(!lf)
        {
            cout<<"no frame any more"<<endl;
            l_becken->stop();
            break;
        }
		l_front.addFrame(lf);
	}
    cout<<"THE END"<<endl;	
#endif
    
	return 0;
}


