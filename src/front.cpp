#include "front.h"
#include "feature.h"
#include "mappoint.h"
#include "optimizerG2o.h"
#include <iostream>
#include <vector>
#include <opencv2/core/eigen.hpp>
#include "optimizerG2o.h"

using namespace std;
namespace MySlam
{
    frontEnd::frontEnd(DataSets& a_sets, SLAMMap::ptr a_newmap)
    {
        mp_detector = cv::GFTTDetector::create();
        m_sets = a_sets;
        m_map = a_newmap;
    }

    void frontEnd::addFrame(frame::ptr newframe) 
    {
        m_currentFrame = newframe;
        if(m_status == E_INITING)
        {
            detectedFeature();
            findFeatureInRight();
            calcMapPoint();
            m_lastFrame = m_currentFrame;	
        }

        if(m_status == E_TRACKING)
        {
            if(m_lastFrame)
            {
                m_currentFrame->setPose(m_relative_motion * m_lastFrame->getPose());
            }
            trackingLastFrame(); //寻找和上一帧match的features
            m_tracking_inlier = EstimateCurrentPose(); //根据match的features优化
            //cout<<"current frame inlier: "<<m_tracking_inlier<<endl;
            if(m_tracking_inlier > m_num_feature_tracking)
            {
                cout<<"inlier num: "<<m_tracking_inlier<<endl;
                m_status = E_TRACKING;
            }
            else
            {
                //当前观测到的三维目标点太少了，不够建图，这说明位姿可能有严重偏差，需要将当前帧设为keyframe用于后端优化
                insertKeyFrame();					
            }
            m_relative_motion = m_currentFrame->getPose() * m_lastFrame->getPose().inverse();
        }

    }

    void frontEnd::addObservationToMapPoint()
    {
        for(auto& k:m_currentFrame->m_leftKPs)
        {
            auto mp = k->m_mapPt.lock();
            if(mp)
            {
                mp->addObservation(k);		
            }
        }		
    }

    void frontEnd::insertKeyFrame()
    {
        if( m_tracking_inlier > m_num_feature_need_for_keyframe)
            return ;
        m_currentFrame->setKeyFrame();
        //将当前观测到的mappoint和feature联系起来，因为上一帧不是重新detetcet特征的，而是根据前一帧算出来的，所以相关的mappoint和feature没有联系起来
        addObservationToMapPoint();  //NOTE: 关联的原因是mappoint可能会被很多的frame看到，图有化的结构想一想就知道了	
        m_map->insertKeyFrame(m_currentFrame);	
        detectedFeature();
        findFeatureInRight();
        calcMapPoint();
        m_becken->wakeUpBeckenLoop();						
    }



    int frontEnd::EstimateCurrentPose()
    {
        using BlockSolverType = g2o::BlockSolver_6_3;
        using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        //define g2o vertex
        vertexPose *vertex_pose = new vertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(m_currentFrame->getPose());
        optimizer.addVertex(vertex_pose);

        //Get K from camera
        Eigen::Matrix<double, 3, 3> K = m_sets.mv_cameras[0].getK();

        //define edges
        std::vector<feature::ptr> features;
        std::vector<edgeProjectionPoseOnly*> edges;
        int index = 1;
        for(int i =0; i < m_currentFrame->m_leftKPs.size(); i++)
        {
            auto mp = m_currentFrame->m_leftKPs[i]->m_mapPt;
            if(mp.lock())
            {
                auto l_mp = mp.lock();
                edgeProjectionPoseOnly *edge = new edgeProjectionPoseOnly(l_mp->getEigenPose(), K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(m_currentFrame->m_leftKPs[i]->getPoint2f()));
                features.push_back(m_currentFrame->m_leftKPs[i]);
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                optimizer.addEdge(edge);
                edges.push_back(edge);
                index++;
            }
        }

        int tracking_inliers = 0;
        const double chi2_th = 5.991;
        for(int iteration = 0; iteration < 4; iteration++)
        {

            tracking_inliers = 0;
            vertex_pose->setEstimate(m_currentFrame->getPose());
            optimizer.initializeOptimization();
            optimizer.optimize(20);
            for(int i = 0; i < edges.size(); i++)
            {
                auto e = edges[i];
                e->computeError();
                if(e->chi2() < chi2_th)
                {
                    //this is a good edge
                    e->setLevel(1);
                    features[i]->m_inlier = true;
                    tracking_inliers++;	
                }
                else
                {
                    //this is not a good edge , mappopint must out of this frame.
                    features[i]->m_inlier = false;
                    e->setLevel(0); // Need optimize this edge anymore.
                }
            }
        }
        m_currentFrame->setPose(vertex_pose->estimate());
        cout<<"Frame estimate pose: "<< m_currentFrame->getPose().matrix3x4()<<endl;
        for(auto& m:features)
        {
            if( !(m->m_inlier) )
            {				
                m->m_mapPt.reset();
                m->m_inlier = true;			
            }
        }
        return tracking_inliers;	
    }


    void frontEnd::trackingLastFrame()
    {
        std::vector<cv::Point2f> kps_last, kps_current;
        for(auto& kp:m_lastFrame->m_leftKPs)
        {
            if(kp->m_mapPt.lock())
            {	
                auto mp = kp->m_mapPt.lock();
                Eigen::Matrix<double,3,1> mpEigen;
                cv::cv2eigen(mp->getPose(),mpEigen);
                auto px = m_sets.mv_cameras[0].world2pixel(mpEigen, m_currentFrame->getPose());
                kps_last.push_back(kp->getPoint2f());
                kps_current.push_back(cv::Point2f(px[0],px[1]));
            }
            else
            {
                kps_last.push_back(kp->getPoint2f());
                kps_current.push_back(kp->getPoint2f());	
            }			
        }

        int num_good_pts = 0;
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(m_currentFrame->m_leftImg, m_currentFrame->m_rightImg, kps_last, kps_current, status, error);
        for(int i = 0; i < status.size(); i++)
        {
            if(status[i])
            {
                feature::ptr feat = make_shared<feature>(kps_current[i]);
                feat->m_isOnLeftImg = true;
                feat->m_frame = m_currentFrame;
                feat->m_mapPt = m_lastFrame->m_leftKPs[i]->m_mapPt;
                m_currentFrame->m_leftKPs.push_back(feat);
                num_good_pts++;
            }
        }
        //cout<<"find match points: "<<num_good_pts<<" in the last image"<<endl;
    }

    void PrintMat(cv::Mat A)
    {
        for(int j=0;j<A.cols;j++)
        {
            for(int i=0;i<A.rows;i++)
                cout<<A.at<float>(i,j)<<' ';
            cout<<endl;
        }
        cout<<endl;
    }


    void homogeneous2normalcoordinate(cv::Mat& A)
    {
        for(int i = 0; i < A.cols; i++)
        {
            for(int j = 0; j < 4; j++)
                A.at<float>(j,i) /= A.at<float>(3,i);
        }
    }

    void frontEnd::calcMapPoint()
    {
        Sophus::SE3d l_poseL, l_poseR;
        l_poseL = m_sets.m_leftcamera.m_pose;
        l_poseR = m_sets.m_rightcamera.m_pose;
        cv::Mat l_camLposcv = cv::Mat(3,4,CV_64FC4);
        cv::Mat l_camRposcv = cv::Mat(3,4,CV_64FC4);
        cv::eigen2cv(l_poseL.matrix3x4(),l_camLposcv);
        cv::eigen2cv(l_poseR.matrix3x4(),l_camRposcv);

        for(int i=0; i < m_currentFrame->m_leftKPs.size(); i++)
        {
            //首帧或位姿一旦不准，重新获取最新的三维目标点	
            if(m_currentFrame->m_leftKPs[i]->m_mapPt.expired() &&
                    m_currentFrame->m_rightKPs[i] != nullptr)
            {

                /* pixel point in camera coordinate */
                cv::Point2f rcpt;  
                cv::Point2f lcpt;
                /*=================================*/

                lcpt = m_sets.m_leftcamera.pixel2camera(m_currentFrame->m_leftKPs[i]->getPoint2f());  //左图camera坐标特征
                rcpt = m_sets.m_rightcamera.pixel2camera(m_currentFrame->m_rightKPs[i]->getPoint2f()); //右图camera坐标特征
                std::vector<cv::Point2f> l_lkps, l_rkps;
                l_lkps.push_back(lcpt);
                l_rkps.push_back(rcpt);	

                cv::Mat homogeneous4D; //point4D
                //进行三角测量, 接口需要矩阵或是vector形式,所以用vector类型
                cv::triangulatePoints(l_camLposcv,l_camRposcv,l_lkps,l_rkps,homogeneous4D);
                homogeneous2normalcoordinate(homogeneous4D);
                cv::Mat mp3D = homogeneous4D.rowRange(0,3);
                if(mp3D.at<float>(0,2) > 0)
                {
                    //获取新的三维目标点
                    mappoint::ptr newMapPoint = make_shared<mappoint>(mp3D); 
                    m_currentFrame->m_leftKPs[i]->m_mapPt = newMapPoint;
                    m_currentFrame->m_rightKPs[i]->m_mapPt = newMapPoint;
                    newMapPoint->addObservation(m_currentFrame->m_leftKPs[i]);
                    newMapPoint->addObservation(m_currentFrame->m_rightKPs[i]);	
                    m_map->insertPoints(newMapPoint);
                }
            }
        }
        m_currentFrame->setKeyFrame();
        m_map->insertKeyFrame(m_currentFrame);	
        m_status = E_STATUS::E_TRACKING;
    }

    void frontEnd::detectedFeature()
    {
        std::vector<cv::KeyPoint> keypoints;
        mp_detector->detect(m_currentFrame->m_leftImg, keypoints);
        std::vector<cv::Point2f> keypoints2f;
        cv::KeyPoint::convert(keypoints, keypoints2f);
        for(auto& k:keypoints2f)
        {
            feature::ptr feat = make_shared<feature>(k);
            feat->m_frame = m_currentFrame;
            feat->m_isOnLeftImg = true;
            m_currentFrame->m_leftKPs.push_back(feat);	
        }
        m_currentFrame->showFrameWithKeyPoint(keypoints);
        cv::waitKey(0);
    }

    unsigned int frontEnd::findFeatureInRight()
    {
        unsigned int num_good_pts = 0;
        std::vector<cv::Point2f> rkps2f, lkps2f;
        for(auto& k:m_currentFrame->m_leftKPs)
        {
            lkps2f.push_back(k->getPoint2f());	
            auto mp = k->m_mapPt.lock();
            if(mp)
            {
                //映射三维目标点到pixel坐标下
                Eigen::Matrix<double,3,1> mpEigen;
                cv::cv2eigen(mp->getPose(),mpEigen);
                Eigen::Matrix<double,2,1> pt = m_sets.mv_cameras[1].world2pixel(mpEigen,m_currentFrame->getPose());  //右侧相机空间
                cv::Point2f p2f;
                p2f.x = pt(0,0);
                p2f.y = pt(1,0);
                rkps2f.push_back(p2f);
            }
            else
            {
                rkps2f.push_back(k->getPoint2f());
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(m_currentFrame->m_leftImg, m_currentFrame->m_rightImg, lkps2f, rkps2f, status, error);
        for(int i = 0;  i < status.size(); i++)
        {
            if(status[i])
            {
                feature::ptr lp_rfeat = make_shared<feature>(rkps2f[i]);
                lp_rfeat->m_frame = m_currentFrame;
                lp_rfeat->m_isOnLeftImg = false;	
                m_currentFrame->m_rightKPs.push_back(lp_rfeat);
                num_good_pts++;
            }
            else
            {
                m_currentFrame->m_rightKPs.push_back(nullptr);
            }
        }
        cout<<"match points: "<<num_good_pts<<endl;
        return num_good_pts;		
        //cout<<"frame leftKP: "<<m_currentFrame->m_leftKPs.size()<<endl;
        //cout<<"frame rightKP: "<<m_currentFrame->m_rightKPs.size()<<endl;
    }
}

