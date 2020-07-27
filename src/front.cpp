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
			m_tracking_inlier = EstimateCurrentPose();
			if(m_tracking_inlier > m_num_feature_tracking)
			{
				m_status = E_TRACKING;
			}
			else
			{
				m_status = E_RESET;
			}
			insertKeyFrome();
			m_relative_motion = m_currentFrame->getPose() * m_lastFrame->getPose().inverse();
		}
		else
		{
			
		}
	}
	
	void frontEnd::insertKeyFrome()
	{
		if( m_tracking_inlier > m_num_feature_need_for_keyframe)
			return ;
		m_currentFrame->setKeyFrame();
		detectedFeature();
		findFeatureInRight();		

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
				edge->setMeasurement(toVec2(m_currentFrame->m_leftKPs[i]->getPts()));
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
		//cout<<"Frame estimate pose: "<< m_currentFrame->getPose().matrix3x4()<<endl;
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
		cv::Mat l_cv_camLpos = cv::Mat(3,4,CV_64FC4);
		cv::Mat l_cv_camRpos = cv::Mat(3,4,CV_64FC4);
		cv::eigen2cv(l_poseL.matrix3x4(),l_cv_camLpos);
		cv::eigen2cv(l_poseR.matrix3x4(),l_cv_camRpos);
		
		for(int i=0; i < m_currentFrame->m_leftKPs.size(); i++)
		{
			if(m_currentFrame->m_rightKPs[i] == nullptr)
				continue;
			/* pixel point in camera coordinate */
			cv::Point2f rcpts;  
			cv::Point2f lcpts;
			/*=================================*/
			
			lcpts = m_sets.mv_cameras[0].pixel2camera(m_currentFrame->m_leftKPs[i]->getPts());
			rcpts = m_sets.mv_cameras[0].pixel2camera(m_currentFrame->m_rightKPs[i]->getPts());
			std::vector<cv::Point2f> l_lkps, l_rkps;
			l_lkps.push_back(lcpts);
			l_rkps.push_back(rcpts);	
		
			cv::Mat mp; //point4D
			cv::triangulatePoints(l_cv_camLpos,l_cv_camRpos,l_lkps,l_rkps,mp);
			//cout<<mp<<endl;
			homogeneous2normalcoordinate(mp);
			//cout<<mp<<endl;
			cv::Mat mp3D = mp.rowRange(0,3);
			if(mp3D.at<float>(0,2) > 0)
			{
				mappoint::ptr newMapPoint = make_shared<mappoint>(mp3D);
				m_currentFrame->m_leftKPs[i]->setMapPoint(newMapPoint);
				m_currentFrame->m_rightKPs[i]->setMapPoint(newMapPoint);
				m_map.insertPoints(newMapPoint);
			}
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
			cout<<"match right key point: "<<num_good_pts<<endl;
			cout<<"frame leftKP: "<<m_currentFrame->m_leftKPs.size()<<endl;
			cout<<"frame rightKP: "<<m_currentFrame->m_rightKPs.size()<<endl;

	}
}

