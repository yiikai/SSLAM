#ifndef OPTIMIZER_G2O_H
#define OPTIMIZER_G2O_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace Sophus;

namespace MySlam
{

//g2o 位姿顶点
class vertexPose : public g2o::BaseVertex<6,SE3d>
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		virtual void setToOriginImpl() override
		{
			_estimate = SE3d();	//vertex , transform matrix
		}
		
		virtual void oplusImpl(const double* update ) override
		{
			Eigen::Matrix<double, 6, 1> vec6f;
			vec6f << update[0], update[1], update[2], update[3], update[4], update[5];
			_estimate = SE3d::exp(vec6f) * _estimate;					
		}
		
		virtual bool read(std::istream &in) override {return true;}
		virtual bool write(std::ostream &out) const override {return true;}
};

//g2o 目标定点，三维mappoint
class vertexXYZ : public g2o::BaseVertex<3,Eigen::Matrix<double,3,1>>
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		virtual void setToOriginImpl() override
		{
			_estimate = Eigen::Matrix<double,3,1>::Zero();		
		}
		
		virtual void oplusImpl(const double* update) override
		{
			_estimate[0] += update[0];
			_estimate[1] += update[1];
			_estimate[2] += update[2];
		}
		virtual bool read(std::istream &in) override {return true;}
		virtual bool write(std::ostream &out) const override {return true;}
};

//g2o: BaseUnaryEdge<D,T,vertex>  D: observe size (pixel point 2D). T: observe type. vertex use para
class edgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Matrix<double, 2, 1>, vertexPose>	
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		// K: camer inner parma, 
		// pos: 3d point corrdinate
		edgeProjectionPoseOnly(const Eigen::Matrix<double, 3,1> &pos, const Eigen::Matrix<double, 3, 3>& K):m_pos(pos),m_k(K){}

		virtual void computeError() override
		{
			const vertexPose *v = static_cast<vertexPose*>(_vertices[0]);
			SE3d T = v->estimate();
			Eigen::Matrix<double, 3, 1> pos_pixel = m_k * (T * m_pos);
			pos_pixel /= pos_pixel[2];
			_error = _measurement - pos_pixel.head<2>();
		}
		
		virtual void linearizeOplus() override {
        	const vertexPose *v = static_cast<vertexPose *>(_vertices[0]);
        	SE3d T = v->estimate();
       	 	Eigen::Matrix<double,3,1> pos_cam = T * m_pos;
        	double fx = m_k(0, 0);
        	double fy = m_k(1, 1);
        	double X = pos_cam[0];
        	double Y = pos_cam[1];
        	double Z = pos_cam[2];
        	double Zinv = 1.0 / (Z + 1e-18);
        	double Zinv2 = Zinv * Zinv;
        	_jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
            	-fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
            	fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
            	-fy * X * Zinv;
    	}
	
		virtual bool read(std::istream &in) override {return true;}
		virtual bool write(std::ostream &out) const override {return true;}
	private:
		Eigen::Matrix<double, 3, 1> m_pos;
		Eigen::Matrix<double, 3, 3> m_k;
};

class edgeProjectionPoseAndXYZ : public g2o::BaseBinaryEdge<2,Eigen::Matrix<double,2,1>, vertexPose,vertexXYZ>
{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		edgeProjectionPoseAndXYZ(const Eigen::Matrix<double,3,3>& cam_K, const SE3d& cam_ex)
		{
			m_camEx = cam_ex;
			m_camK = cam_K;
		}

		virtual void computeError() override
		{
			const vertexPose *v_0 = static_cast<vertexPose*>(_vertices[0]);
			const vertexXYZ  *v_1 = static_cast<vertexXYZ*>(_vertices[1]);
			SE3d T = v_0->estimate();
			Eigen::Matrix<double,3,1> cam_pos = T * v_1->estimate();
			Eigen::Matrix<double,3,1> pixel = m_camK * (m_camEx * cam_pos); 
			pixel /= pixel[2];  //转成2维数据，因为第三维都是1
			_error = _measurement - pixel.head<2>(); 
		}
		
		virtual void linearizeOplus() override
		{
			const vertexPose *v0 = static_cast<vertexPose *>(_vertices[0]);
			const vertexXYZ *v1 = static_cast<vertexXYZ *>(_vertices[1]);
			SE3d T = v0->estimate();
			Eigen::Matrix<double,3,1> pw = v1->estimate();
			Eigen::Matrix<double,3,1> pos_cam = m_camEx * T * pw;
			double fx = m_camK(0, 0);
			double fy = m_camK(1, 1);
			double X = pos_cam[0];
			double Y = pos_cam[1];
			double Z = pos_cam[2];
			double Zinv = 1.0 / (Z + 1e-18);
			double Zinv2 = Zinv * Zinv;
			//两个定点对应的导数
			_jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
								 -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
								 fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
								 -fy * X * Zinv;

			_jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
						m_camEx.rotationMatrix() * T.rotationMatrix();
		}
	
		virtual bool read(std::istream &in) override {return true;}
		virtual bool write(std::ostream &out) const override {return true;}

	private:
		SE3d  m_camEx; //相机外参
		Eigen::Matrix<double, 3, 3> m_camK; //相机内参
};

}

#endif
