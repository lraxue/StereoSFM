/***************************************
*
*
*
*
****************************************/

#include "MapPoint.h"
#include "ORBmatcher.h"

#include <mutex>

using namespace std;


long unsigned int MapPoint::nNextId = 0;
std::mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat& Pos, KeyFrame* pRefKF, Map* pMap)
{

}
MapPoint::MapPoint(const cv::Mat& pos, Map* pMap, 
	Frame* pFrame, const int &idxF)
{

}