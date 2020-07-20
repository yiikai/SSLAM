#ifndef MAP_H
#define MAP_H
#include <vector>
#include "mappoint.h"
using namespace std;
namespace MySlam
{

class map
{
	public:
		map();
		~map();
		void insertPoints(mappoint::ptr point);
		vector<mappoint::ptr>& getMapRef()
		{
			return m_3dpoints;
		}
	private:
		vector<mappoint::ptr> m_3dpoints;
};

}

#endif