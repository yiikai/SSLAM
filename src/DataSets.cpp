#include "DataSets.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include "sophus/se3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace MySlam
{
	DataSets::DataSets(string& as_dp):ms_DataPath(as_dp)
	{
	
	}
	
	DataSets::~DataSets()
	{
			
	}

	bool DataSets::init()
	{
		// read camera instrinsics and extrinsics
		//kitti has 4 camera
		//Just Use 1 and 2 camera for gray img
		int li_cameracount = 2;
		char camera_name[4] = {0,0,0,0};
		double camera_projection[12];	
		ifstream l_ReadConfig(ms_DataPath + "/calib.txt");
		for(int i = 0 ; i < li_cameracount; i++)
		{
			Camera lc_camera;
			for(int j = 0; j < 3; j++)
			{
				l_ReadConfig >> camera_name[j];
				lc_camera.name = camera_name;
			}
			for(int j = 0 ; j < 12; j ++)
			{
				l_ReadConfig >> camera_projection[j];
			}
			Eigen::Matrix<double,3,3>	K;					
			Eigen::Matrix<double,3,1>   T;
			K << camera_projection[0], camera_projection[1], camera_projection[2],camera_projection[4],camera_projection[5],camera_projection[6],camera_projection[8],camera_projection[9],camera_projection[10];
			T << camera_projection[3] , camera_projection[7], camera_projection[11];
			double baseline = T.norm();
			Sophus::SE3d rotation(Sophus::SO3d(), T);	
			lc_camera.initCamera(K(0,0),K(1,1),K(0,2),K(1,2),baseline,rotation);
		mv_cameras.push_back(lc_camera);	    		
		}
		l_ReadConfig.close();
		return true;
	}


	frame::ptr DataSets::nextFrame()
	{
		std::string fmt = "/image_%d/%06d.png";
		cv::Mat ls_left, ls_right;  //left & right img
		char leftdir[100];
		sprintf(leftdir,fmt.c_str(),0,m_imgcount);
		char rightdir[100];
		sprintf(rightdir,fmt.c_str(),1,m_imgcount);
//		cout<<"left:"<< ms_DataPath + leftdir<<" right:"<<ms_DataPath + rightdir<<endl;
		m_imgcount++;
		ls_left = cv::imread(ms_DataPath + leftdir,cv::IMREAD_GRAYSCALE);
		ls_right = cv::imread(ms_DataPath + rightdir,cv::IMREAD_GRAYSCALE);
		frame::ptr lp_newframe = std::make_shared<frame>();
		lp_newframe->m_leftImg = ls_left;
		lp_newframe->m_rightImg = ls_right;
		return lp_newframe;	
	}
}
