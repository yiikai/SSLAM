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
const float SCALE_FACTOR = 1.2f;
float getScaleFactor(int level)
{
    float scale = 1.0f;
    for(int i = 1; i < level; i++)
    {
        scale*=SCALE_FACTOR;
    }
    return scale;
}

std::vector<Mat> mvImagePyramidL(8);
std::vector<Mat> mvImagePyramidR(8);
void ComputePyramid(cv::Mat image, const int nlevels, std::vector<Mat>& mvImagePyramid)
{
    for (int level = 0; level < nlevels; ++level)
    {
        float scale = 1.0f/getScaleFactor(level);
        Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101+BORDER_ISOLATED);
        }
        else
        {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           BORDER_REFLECT_101);
        }
    }
#if 0 /*show ORB prymaid img*/
    for(int i =0 ; i < 8 ; i++)
    {
        imshow("match feature",mvImagePyramid[i]);
        cv::waitKey(0);
    }
#endif

}


int main(int argc, char* * argv)
{
#if 0
    string img1path = "/home/yiikai/Develop/MySlam/resource/21/image_0/000000.png";
    string img2path = "/home/yiikai/Develop/MySlam/resource/21/image_0/000001.png";
    cv::Mat img1 = cv::imread(img1path);
    cv::Mat img2 = cv::imread(img2path);
    cvtColor(img1,img1,COLOR_BGR2GRAY);
    cvtColor(img2,img2,COLOR_BGR2GRAY);
    cout<<img1.depth()<<", "<<img1.channels()<<endl;
    cout<<img2.depth()<<", "<<img2.channels()<<endl;
         

    //计算ORB的金字塔图层的每一层的size
    ComputePyramid(img1,8,mvImagePyramidL);
    ComputePyramid(img2,8,mvImagePyramidR);

    cv::Ptr<cv::ORB> orbleft = cv::ORB::create(500,SCALE_FACTOR,8,31,0,2,ORB::FAST_SCORE);
    cv::Ptr<cv::ORB> orbright = cv::ORB::create(500,SCALE_FACTOR,8,31,0,2,ORB::FAST_SCORE);

    std::vector<cv::KeyPoint> keypointsL,keypointsR;
    cv::Mat descriptorsL,descriptorsR;
    orbleft->detectAndCompute(img1,cv::Mat(),keypointsL,descriptorsL);
    orbright->detectAndCompute(img2,cv::Mat(),keypointsR,descriptorsR);
    //cv::BFMatcher matcher(NORM_HAMMING);
    //std::vector<DMatch> mathces;
	//matcher.match(descriptorsL, descriptorsR, mathces);
    
    std::vector<std::vector<int>> vRowIndices(keypointsR.size(), std::vector<int>());
    for(int i = 0 ; i < keypointsR.size(); i++)
    {
        vRowIndices[i].reserve(200);
    }

    const int rows = img1.rows;
    for(int iR = 0; iR < keypointsR.size(); iR++)
    {
        cv::KeyPoint& kp = keypointsR[iR];
        const float y = kp.pt.y;
        const float x = kp.pt.x;
        const float r = 2.0f * getScaleFactor(kp.octave);
        const float maxRange = ceil(y + r);
        const float minRange = floor(y - r);
        for(int i = minRange; i < maxRange; i++)
        {
            vRowIndices[i].push_back(iR);
        }
    }
    std::vector<pair<int,int>> vPair(keypointsL.size());
    //对左图的每一个特征点寻找一个匹配的右图的特征点
    for(int iL = 0; iL < keypointsL.size(); iL++)
    {
        cv::KeyPoint kpL = keypointsL[iL];
        const float y = kpL.pt.y;
        const float x = kpL.pt.x;
        int levelL = kpL.octave;
        auto candicates = vRowIndices[y];
        if(candicates.empty())
            continue;

        float mindist = 1000;
        int bestiR = 0;
        for(auto& iR:candicates)
        {
            cv::KeyPoint kpR = keypointsR[iR];
            int levelR = kpR.octave;
            if(levelR < levelL -1 && levelR > levelL +1) //特征点金字塔级别太大的不考虑
                continue;
            const cv::Mat dL = descriptorsL.row(iL);
            const cv::Mat dR = descriptorsR.row(iR);
            cv::BFMatcher matcher(NORM_HAMMING);
            std::vector<DMatch> mathces;
            matcher.match(dL, dR, mathces);
            //从匹配的matches中选出好的那个匹配对
            for(auto& match:mathces)
            {
                if(match.distance < mindist)
                {
                    mindist = match.distance;
                    bestiR = iR;
                }
            }
        }
    
        const int orbTH = 80;
        if(mindist > 80)
            continue;
        
        //SAD 进一步优化匹配的特征点的精度
        float scalefactorL =  getScaleFactor(levelL);
        float scalefactorInvL = 1/getScaleFactor(levelL);
        int xL = round(x*scalefactorInvL);
        int yL = round(y*scalefactorInvL);
        int xR = round(keypointsR[bestiR].pt.x * scalefactorInvL);
       
        int w = 5;
        int steps = 5;
        int bestDist = 10000;
        int beststep = 0;
        std::vector<float> vdist(2*steps + 1);
         
        cv::Mat IL = mvImagePyramidL[levelL].rowRange(yL-w, yL+w+1).colRange(xL-w, xL+w+1); //定义sad slide window
        //归一化
        IL.convertTo(IL,CV_32F);
        IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);
        const float iniu = xR + steps - w;
        const float endu = xR + steps + w + 1;
        if(iniu <  0 || endu > mvImagePyramidR[levelL].cols)
            continue;
         
        for(int i = -steps; i <= steps; i++)
        {
            cv::Mat IR = mvImagePyramidR[levelL].rowRange(yL-w,yL+w+1).colRange(xR+i-w, xR+i+w+1);
            IR.convertTo(IR,CV_32F);
            IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);
            
            float dist = cv::norm(IL,IR,cv::NORM_L1);
            if(dist < bestDist)
            {
                bestDist = dist;
                beststep = i;
            }
            vdist[i + w] = dist;
        }        

        if(beststep == -steps || beststep == steps)  //根据sad原理，最佳值再所有距离组成的抛物线低端，不可能是再两个头的
            continue; 
        const float dist1 = vdist[steps + bestDist - 1];
        const float dist2 = vdist[steps + bestDist];
        const float dist3 = vdist[steps + bestDist + 1];

        const float deltaR = (dist1 - dist2)/(2.0f*(dist1 + dist3 - 2.0f*dist2));
        if(deltaR < -1 || deltaR > 1)
            continue;
        
        cout<<"delta R:"<<deltaR<<endl;
        float bestUR = scalefactorL * static_cast<float>(bestiR + beststep + deltaR);
         
    }
#endif
#if 0 //比较粗糙的特征匹配筛选
    //drawKeypoints(img1, keypoints, img1, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
    double min_dist = 10000, max_dist = 0;
    for(int i = 0; i < descriptors1.rows; i++)
    {
        if(mathces[i].distance < min_dist)
            min_dist = mathces[i].distance;
        if(mathces[i].distance > max_dist)
            max_dist = mathces[i].distance;
    }
    c
    out<<"max distance: "<<max_dist<<endl;
    cout<<"min distance: "<<min_dist<<endl;
    std::vector<DMatch> filterMathces;
    for(int i = 0; i < descriptors1.rows; i++)
    {
        if(mathces[i].distance <= max(2 * min_dist, 100.0))
            filterMathces.push_back(mathces[i]);
    }
#endif 
	//Mat matchMat;
	//drawMatches(img1, keypoints1, img2, keypoints2, filterMathces, matchMat);
    //imshow("match feature",matchMat);
    //cv::waitKey(0);
#if 1
	string path = "/home/yiikai/Develop/MySlam/resource/21";
	DataSets sets(path);
	sets.init();
	SLAMMap::ptr lp_newmap = make_shared<SLAMMap>();
	becken::ptr l_becken = make_shared<becken>(sets);
    //Viewer::ptr l_viewer = make_shared<Viewer>();
	frontEnd l_front(sets, lp_newmap);
	l_front.addBecken(l_becken);
    //l_viewer->SetMap(lp_newmap);
    //l_front.setViewer(l_viewer);
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


