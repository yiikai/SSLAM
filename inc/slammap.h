#ifndef MAP_H
#define MAP_H
#include <map>
#include "mappoint.h"
#include "frame.h"
#include <memory>
using namespace std;
namespace MySlam
{

class SLAMMap
{
	public:
		using ptr = shared_ptr<SLAMMap>;
		SLAMMap();
		~SLAMMap();
		void insertPoints(mappoint::ptr point);
		void insertKeyFrame(frame::ptr frame);
		const map<unsigned long,mappoint::ptr> getActiveMapPoints();
		const map<unsigned long,frame::ptr> getActiveFrames();
	private:
		map<unsigned long,mappoint::ptr> m_activeMapPoints;
		map<unsigned long,frame::ptr> m_activeFrames;
};

}

#endif
