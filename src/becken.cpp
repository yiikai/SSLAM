#include "becken.h"
#include "optimizerG2o.h"
#include <map>
namespace MySlam
{

	becken::becken(DataSets a_sets)
	{
		cout<<"create thread becken"<<endl;
		m_sets = a_sets;
        m_running = true;
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
    
    void becken::stop()
    {
        m_running = false;
        //再次出发条件变量防止再停止线程之前，后端线程就已经block了 
		m_loopcv.notify_one();
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
		while(m_running)
		{
            cout<<"Optimizer all key frame!!!!!!!!!"<<endl;
			std::unique_lock<std::mutex> lck(m_beckenloopmutex);
			m_loopcv.wait(lck);
			//becken Optimizer
            if(m_running)
			    optimizer();	
		}	
	}
	
	void becken::optimizer()
	{
		//do becken optimizer for key frame pose and mappoint
		unsigned long max_kf_id = 0;
		const map<unsigned long,mappoint::ptr>& activeMapPoints = 	m_map->getActiveMapPoints();
		const map<unsigned long,frame::ptr>& activeFrames = m_map->getActiveFrames();
		typedef g2o::BlockSolver_6_3 BlockSolverType;
		typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
		auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		//add vertex to graph
		map<unsigned long, vertexPose*> vertices;
		//添加位姿 	
		for(auto& l_frame:activeFrames)
		{
			vertexPose *v_pose = new vertexPose();
			v_pose->setId(l_frame.first);
			v_pose->setEstimate(l_frame.second->getPose());
			optimizer.addVertex(v_pose);
			vertices.insert({l_frame.first, v_pose});
			if(l_frame.second->getKeyID() > max_kf_id)
				max_kf_id = l_frame.first;	
		}

		//添加路标点
		Eigen::Matrix<double, 3, 3> K = m_sets.mv_cameras[0].getK();
		SE3d left_ext = m_sets.mv_cameras[0].m_pose;
		SE3d right_ext = m_sets.mv_cameras[1].m_pose;		
		
		int index = 1;
		double chi2_th = 5.991;
		//记录需要优化的定点,如果记录过了就不需要再添加这个顶点的边了
		map<unsigned long, vertexXYZ*> verticesLandmarks; 
		map<edgeProjectionPoseAndXYZ*, feature::ptr> edges;
		for(auto& point:activeMapPoints)
		{
			//获取目标点的所有观察帧，遍历它们创建G2o的边和顶点
			auto observations = point.second->getObservation();
			for(auto& feat:observations)
			{
				auto l_frame = feat->m_frame.lock();
				if(!feat->m_inlier || !l_frame )
					continue;
				edgeProjectionPoseAndXYZ *edge = nullptr;
				if(feat->m_isOnLeftImg)
				{
					edge = new edgeProjectionPoseAndXYZ(K,left_ext);			
				}
				else
				{
					edge = new edgeProjectionPoseAndXYZ(K,right_ext);			
				}

                if(edge == nullptr)
				{
					cout<<"!!!!!!!!!!!!!!!ERROR!!!!!!!!!!!!!!!!"<<endl;
					return;
				}

				if(verticesLandmarks.find(point.second->getID()) == verticesLandmarks.end())
				{
					//TODO: 添加新的顶点
					vertexXYZ *v = new vertexXYZ();
					v->setEstimate(point.second->getEigenPose());
					v->setId(point.second->getID() + max_kf_id + 1);
					v->setMarginalized(true);
					verticesLandmarks.insert({point.second->getID(),v});
					optimizer.addVertex(v);		
				}
				
				edge->setId(index);
                try{
                    vertices.at(l_frame->getKeyID());
                }catch(exception& e)
                {
                    cout<<e.what()<<endl;
                    continue;
                }
				edge->setVertex(0, vertices.at(l_frame->getKeyID()));				   
				edge->setVertex(1, verticesLandmarks.at(point.second->getID()));
				edge->setMeasurement(feat->getEigenPts());
				edge->setInformation(Eigen::Matrix<double,2,2>::Identity());
				auto rk = new g2o::RobustKernelHuber();
				rk->setDelta(chi2_th);
				edge->setRobustKernel(rk);
				
				optimizer.addEdge(edge);
				edges.insert({edge,feat}); 
				index++; 
			}		
		}
		optimizer.initializeOptimization();
		optimizer.optimize(20);
		
		//优化后，如果发现目标点不再当前帧的可视范围内，计算有多少不再可视范围内，如果超过了一半，就要调整阀值，防止损失太多的目标点影响最终的map构建
		int iter = 5; 
		double cntInlier = 0;
		double cntOutlier = 0;	
		do
		{
			for(auto& edge:edges)
			{
				if(edge.first->chi2() > chi2_th)
				{
					cntInlier++;	
				}
				else
				{
					cntOutlier++;
				}
			}
			if(cntInlier / cntOutlier > 0.5)
				break;
			else
				chi2_th *= 2;			
		}while(iter-- != 0);
		
		//移除移除权重过大的特征点和目标点
		for(auto& edge:edges)
		{
			if(edge.first->chi2() > chi2_th)
			{
				auto feat = edge.second;
				feat->m_inlier = false;	
				feat->m_mapPt.lock()->removeObservation(feat);	
			}
			else
			{
				auto feat = edge.second;
				feat->m_inlier = true;
			}
		}
		//设置优化后的目标三维点和位姿
		for(auto& v:vertices)
		{
			activeFrames.at(v.first)->setPose(v.second->estimate());
			cout<<"frame ID:"<<v.first<<" pose is "<<v.second->estimate().matrix3x4()<<endl;	
		}

		for(auto& mp:verticesLandmarks)
		{
			activeMapPoints.at(mp.first)->setPose(mp.second->estimate());
		}
			
	}
	
}
