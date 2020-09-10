#include "orbExtractor.h"
namespace MySlam
{
orbExtractor::orbExtractor(cv::Mat img,
                           const int num,
                           const float scale,
                           const int levels):mvImagePyramid(levels,cv::Mat())
{
    mLevels = levels;
    mFeature = num;
    mImg = img;
    mScale = scale;

    cvtColor(mImg,mImg,cv::COLOR_BGR2GRAY);
    mOrb = cv::ORB::create(num,scale,levels,31,0,2,cv::ORB::FAST_SCORE);
   computePyramid(mImg);
}


orbExtractor::~orbExtractor()
{

}

void orbExtractor::computePyramid(cv::Mat image)
{
    for (int level = 0; level < mLevels; ++level)
    {
        float scale = 1.0f/getScaleFactor(level);
        cv::Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
        cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        cv::Mat temp(wholeSize, image.type()), masktemp;
        mvImagePyramid[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 )
        {
            resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);

            copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                    cv::BORDER_REFLECT_101+cv::BORDER_ISOLATED);
        }
        else
        {
            copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                    cv::BORDER_REFLECT_101);
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

float orbExtractor::getScaleFactor(int level)
{
    float scale = 1.0f;
    for(int i = 1; i < level; i++)
    {
        scale*=mScale;
    }
    return scale;

}

void orbExtractor::detectAndCompute()
{
    if(mOrb)
        mOrb->detectAndCompute(mImg,cv::Mat(),mKps,mDescriptors);
}

}



