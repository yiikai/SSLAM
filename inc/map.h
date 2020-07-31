#ifndef MAP_H
#define MAP_H
#include <vector>
#include "mappoint.h"
#include "frame.h"
using namespace std;
namespace MySlam
{

class map
{
	public:
		map();
		~map();
		void insertPoints(mappoint::ptr point);
		void insertKeyFrame(frame::ptr frame);
	private:
		vector<mappoint::ptr> m_activeMapPoints;
		vector<frame::ptr> m_activeFrames;
		
};

}

#endif
