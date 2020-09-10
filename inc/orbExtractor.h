#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"

/*using cv::ORB implemented*/
namespace MySlam
{
#define EDGE_THRESHOLD 19
class orbExtractor
{
    public:
        using ptr = std::shared_ptr<orbExtractor>;
        orbExtractor(cv::Mat img, const int fnum, const float scale, const int levels);
        ~orbExtractor();
        void detectAndCompute();
        std::vector<cv::KeyPoint>& getKeyPoints(){return mKps;} 
        cv::Mat getDesc(){return mDescriptors;}
        std::vector<cv::Mat> getPyramid(){return mvImagePyramid;}
        float getScaleFactor(int level);
    private:
        void computePyramid(cv::Mat image);
    private:
        cv::Mat mImg;
        int mFeature;
        float mScale;
        int mLevels;
        std::vector<cv::Mat> mvImagePyramid;       
        cv::Ptr<cv::ORB> mOrb;
        std::vector<cv::KeyPoint> mKps;
        cv::Mat mDescriptors;
        
};
}
#endif
