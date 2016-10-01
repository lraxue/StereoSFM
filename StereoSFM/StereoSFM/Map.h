/***************************************
*
*
*
*
****************************************/

#ifndef MAP_H__
#define MAP_H__

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

class MapPoint;
class KeyFrame;

class Map
{
public:
	Map();
	~Map(){};

public:
	void AddKeyFrame(KeyFrame* pKF);
	void AddMapPoint(MapPoint* pMP);
	void EraseMapPoint(MapPoint* pMP);
	void EraseKeyFrame(KeyFrame* pKF);
	void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

	std::vector<KeyFrame*> GetAllKeyFrames();
	std::vector<MapPoint*> GetAllMapPoints();
	std::vector<MapPoint*> GetReferenceMapPoints();

	long unsigned int MapPointsInMap();
	long unsigned  KeyFramesInMap();

	long unsigned int GetMaxKFid();

	void clear();

	vector<KeyFrame*> mvpKeyFrameOrigins;

	std::mutex mMutexMapUpdate;

	// This avoid that two points are created simultaneously in separate threads (id conflict)
	std::mutex mMutexPointCreation;

protected:
	std::set<MapPoint*> mspMapPoints;
	std::set<KeyFrame*> mspKeyFrames;

	std::vector<MapPoint*> mvpReferenceMapPoints;

	long unsigned int mnMaxKFid;

	std::mutex mMutexMap;

};
#endif