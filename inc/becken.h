#ifndef BECKEN_H
#define BECKEN_H

#include <thread>
#include <condition_variable>
#include <mutex>
#include <memory>
using namespace std;

namespace MySlam
{
class becken
{
public:
	using ptr = shared_ptr<becken>; 
	becken();
	~becken();
	
	/* becken optimizer */
	void beckenLoop();
	void wakeUpBeckenLoop();	
private:
	thread* m_loop = {nullptr};
	mutex m_beckenloopmutex;
	condition_variable m_loopcv;	
};
}

#endif
