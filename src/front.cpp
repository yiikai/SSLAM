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
	frontEnd::frontEnd(DataSets& a_sets)
	{
		mp_detector = cv::GFTTDetector::create();
		m_sets = a_sets;
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
		else if(m_status == E_TRACKING)
		{
			if(m_lastFrame)
			{
				m_currentFrame->setPose(m_relative_motion * m_lastFrame->getPose());
			}
			trackingLastFrame();
		}
		else
		{
			
		}
	}
	
	void frontEnd::EstimateCurrentPose()
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
				edge->setMeasurement(toVec2(m_currentFrame->m_leftKPs[i]->getPts()));
			}
		}

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
				kps_last.push_back(kp->getPts());
				kps_current.push_back(cv::Point2f(px[0],px[1]));
			}
			else
			{
				kps_last.push_back(kp->getPts());
				kps_current.push_back(kp->getPts());	
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
				feat->m_mapPt = m_lastFrame->m_leftKPs[i]->m_mapPt;
				m_currentFrame->m_leftKPs.push_back(feat);
				num_good_pts++;
			}
		}
		cout<<"find "<<num_good_pts<<" in the last image"<<endl;
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
		l_poseL = m_sets.mv_cameras[0].m_pose;
		l_poseR = m_sets.mv_cameras[1].m_pose;
		cv::Mat l_cv_camLpos;
		cv::Mat l_cv_camRpos;
		cv::eigen2cv(l_poseL.matrix3x4(),l_cv_camLpos);
		cv::eigen2cv(l_poseR.matrix3x4(),l_cv_camRpos);
		
		for(int i=0; i < m_currentFrame->m_leftKPs.size(); i++)
		{
			if(m_currentFrame->m_rightKPs[i] == nullptr)
				continue;
			auto l_rpts = m_currentFrame->m_rightKPs[i]->getPts();
			auto l_lpts = m_currentFrame->m_leftKPs[i]->getPts();
			std::vector<cv::Point2f> l_lkps, l_rkps;
			l_lkps.push_back(l_lpts);
			l_rkps.push_back(l_rpts);	
		
			cv::Mat mp; //point4D
			cv::triangulatePoints(l_cv_camLpos,l_cv_camRpos,l_lkps,l_rkps,mp);
			homogeneous2normalcoordinate(mp);
			cv::Mat mp3D = mp.rowRange(0,3);
			//PrintMat(mp);
			mappoint::ptr newMapPoint = make_shared<mappoint>(mp3D);
			m_currentFrame->m_leftKPs[i]->setMapPoint(newMapPoint);
			m_currentFrame->m_rightKPs[i]->setMapPoint(newMapPoint);
			m_map.insertPoints(newMapPoint);
		}
		m_status = E_STATUS::E_TRACKING;
	}

	void frontEnd::detectedFeature()
	{
		std::vector<cv::KeyPoint> keypoints;
		mp_detector->detect(m_currentFrame->m_leftImg
, keypoints);
		std::vector<cv::Point2f> l_lkps;
		cv::KeyPoint::convert(keypoints, l_lkps);
		for(auto& k:l_lkps)
		{
			feature::ptr feat = make_shared<feature>(k);
			m_currentFrame->m_leftKPs.push_back(feat);	
		}
	//	m_currentFrame->showFrameWithKeyPoint(keypoints);
	//	cv::waitKey(0);
	}

	void frontEnd::findFeatureInRight()
	{
		int num_good_pts = 0;
		if(m_status == E_STATUS::E_INITING)
		{
			std::vector<cv::Point2f> lv_rightKps,lv_leftKps;
			for(auto& k:m_currentFrame->m_leftKPs)
			{
				lv_leftKps.push_back(k->getPts());	
			}
			lv_rightKps = lv_leftKps;
			std::vector<uchar> status;
			cv::Mat error;
			cv::calcOpticalFlowPyrLK(m_currentFrame->m_leftImg, m_currentFrame->m_rightImg, lv_leftKps, lv_rightKps, status, error);
			for(int i = 0;  i < status.size(); i++)
			{
				if(status[i])
				{
					feature::ptr l_rf = make_shared<feature>(lv_rightKps[i]);		
					m_currentFrame->m_rightKPs.push_back(l_rf);
					num_good_pts++;
				}
				else
				{
					m_currentFrame->m_rightKPs.push_back(nullptr);
				}
			}		
		}
		cout<<"match right key point: "<<num_good_pts<<endl;
		cout<<"frame leftKP: "<<m_currentFrame->m_leftKPs.size()<<endl;
		cout<<"frame rightKP: "<<m_currentFrame->m_rightKPs.size()<<endl;
		
	}
}

