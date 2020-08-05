#include "becken.h"
#include <function>
namespace MySlam
{

	becken::becken()
	{
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
		
	}

}
