#include <iostream>
#include <opencv2/opencv.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel_impl.h>
#include "sophus/se3.hpp"
#include "inc/DataSets.h"
#include "front.h"
#include "slammap.h"
using namespace MySlam;

int main(int argc, char* * argv)
{
	string path = "/home/yiikai/Develop/MySlam/resource/21";
	DataSets sets(path);
	sets.init();
	SLAMMap::ptr lp_newmap = make_shared<SLAMMap>();
	becken::ptr l_becken = make_shared<becken>(sets);
	frontEnd l_front(sets, lp_newmap);
	l_front.addBecken(l_becken);
	while(1)
	{
        frame::ptr lf = sets.nextFrame();
        if(!lf)
        {
            cout<<"no frame any more"<<endl;
            l_becken->stop();
            break;
        }
		l_front.addFrame(lf);
	}
    cout<<"THE END"<<endl;	
	return 0;
}
