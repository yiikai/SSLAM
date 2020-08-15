#ifndef BECKEN_H
#define BECKEN_H

#include <thread>
#include <condition_variable>
#include <mutex>
#include <memory>
#include "slammap.h"
#include "DataSets.h"
using namespace std;

namespace MySlam
{
class becken
{
public:
	using ptr = shared_ptr<becken>; 
	becken(DataSets a_sets);
	~becken();
	
	/* becken optimizer */
	void beckenLoop();
	void wakeUpBeckenLoop();
	void addMap(SLAMMap::ptr a_newmap);
    void stop();
private:
	void optimizer();	
private:
	thread* m_loop = {nullptr};
	mutex m_beckenloopmutex;
	condition_variable m_loopcv;
	SLAMMap::ptr m_map;
	DataSets m_sets;
    bool m_running = {false}; //becken thread if running	
};
}

#endif
