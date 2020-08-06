#ifndef MAP_H
#define MAP_H
#include <vector>
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
		const vector<mappoint::ptr>& getActiveMapPoints();
		const vector<frame::ptr>& getActiveFrames();
	private:
		vector<mappoint::ptr> m_activeMapPoints;
		vector<frame::ptr> m_activeFrames;
		
};

}

#endif
