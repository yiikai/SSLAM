#include "becken.h"
#include "optimizerG2o.h"
#include <map>
namespace MySlam
{

	becken::becken(DataSets a_sets)
	{
		m_sets = a_sets;
		m_loop = new thread(&becken::beckenLoop, this);
	}
	
	becken::~becken()
	{
		if(m_loop)
		{
			m_loop->detach();	
			delete m_loop;
		}
	}
	
	void becken::addMap(SLAMMap::ptr a_newmap)
	{
		m_map = a_newmap;
	}
	
	void becken::wakeUpBeckenLoop()
	{
				
		std::unique_lock<std::mutex> lck(m_beckenloopmutex);
		m_loopcv.notify_one();
			
	}

	void becken::beckenLoop()
	{
		while(1)
		{
			std::unique_lock<std::mutex> lck(m_beckenloopmutex);
			m_loopcv.wait(lck);
			//TODO: becken Optimizer
			
		}	
	}
	
	void becken::optimizer()
	{
		//do becken optimizer for key frame pose and mappoint
		const vector<mappoint::ptr>& activeMapPoints = 	m_map->getActiveMapPoints();
		const vector<frame::ptr>& activeFrames = m_map->getActiveFrames();
		typedef g2o::BlockSolver_6_3 BlockSolverType;
		typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
		auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		//add vertex to graph
		//添加位姿 	
		for(auto& frame:activeFrames)
		{
			vertexPose *v_pose = new vertexPose();
			v_pose->setId(frame->getKeyID());
			v_pose->setEstimate(frame->getPose());
			optimizer.addVertex(v_pose);	
		}
		//添加路标点
		Eigen::Matrix<double, 3, 3> K = m_sets.mv_cameras[0].getK();
		SE3d left_ext = m_sets.mv_cameras[0].m_pose;
		SE3d right_ext = m_sets.mv_cameras[1].m_pose;		
		
		int index = 1;
		double chi2_th = 5.991;
		//记录需要优化的定点,如果记录过了就不需要再添加这个定点的边了
		map<unsigned long, mappoint> verticesLandmarks; 	
		for(auto& point:activeMapPoints)
		{
			if(point->isOutLier())
				continue;
			//获取目标点的所有观察帧，遍历它们创建G2o的边和顶点
			auto observations = point->getObservation();
			for(auto& feat:observations)
			{
				auto frame = feat->m_frame.lock();
				if(feat->m_inlier || !frame )
					continue;
				if(feat->m_isOnLeftImg)
				{
					edgeProjectionPoseAndXYZ *edge = new edgeProjectionPoseAndXYZ(K,left_ext);			
				}
				else
				{
					edgeProjectionPoseAndXYZ *edge = new edgeProjectionPoseAndXYZ(K,right_ext);			
				}
				if(verticesLandmarks.find(point->getID()) == verticesLandmarks.end())
				{
					//TODO: 添加新的顶点
				}	
			}			
		}					
	}

}
