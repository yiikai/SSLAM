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
		
		virtual bool read(std::istream &in) override {return true;}
		virtual bool write(std::ostream &out) const override {return true;}
	private:
		Eigen::Matrix<double, 3, 1> m_pos;
		Eigen::Matrix<double, 3, 3> m_k;
};

}

#endif
