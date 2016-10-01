/***************************************
*
*
*
*
****************************************/

#ifndef MAPPOINT_H__
#define MAPPOINT_H__

#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
	MapPoint(const cv::Mat& pos, KeyFrame* pRefKF, Map* pMap);
	MapPoint(const cv::Mat& pos, Map* pMap, Frame* pFrame, const int &idxF);

	~MapPoint(){};

public:
	void setWorldPos(const cv::Mat& Pos);
	cv::Mat GetWorldPos();

	cv::Mat GetNormal();
	KeyFrame* GetReferenceKeyFrame();

	std::map<KeyFrame*, size_t> GetObservations();
	int Observations();

	void AddObservation(KeyFrame* pKF, size_t idx);
	void EraseObservation(KeyFrame* pKF);

	int GetIndexInKeyFrame(KeyFrame* pKF);
	bool IsInKeyFrame(KeyFrame* pKF);

	void SetBadFlag();
	bool isBad();

	void Replace(MapPoint* pMP);
	MapPoint* GetReplaced();

	void IncreaseVisible(int n = 1);
	void IncreaseFound(int n = 1);
	float GetFoundRatio();
	inline int GetFound() {
		return mnFound;
	}

	void ComputeDiscrimitiveDescriptors();

	cv::Mat GetDescriptor();

	void UpdateNormalAndDepth();

	float GetMinDistanceInvariance();
	float GetMaxDistanceInvariance();
	int PredictScale(const float &currentDist, const float &logScaleFactor);

public:
	long unsigned int mnId;
	static long unsigned int nNextId;
	long int mnFirstKFid;
	int nObs;

	// Variable used by the tracking 
	float mTrackProjX;
	float mTrackProjY;
	float mTrackProjXR;
	bool  mbTrackInView;
	float mnTrackScaleLevel;
	float mTrackViewCos;
	long unsigned int mnTrackReferenceForFrame;
	long unsigned int mnLastFrameSeen;

	// Variable used by local mapping
	long unsigned int mnBALocalForKF;
	long unsigned int mnFuseCandiadateForKF;

	// Variable used by loop closing
	long unsigned int mnLoopPointForKF;
	long unsigned int mnCorrectedByKF;
	long unsigned int mnCorrectedReference;
	cv::Mat mPosGBA;
	long unsigned int mnBAGlobalForKF;

	static std::mutex mGlobalMutex;

protected:
	// Position in absolute coordinates
	cv::Mat mWorldPos;

	// KeyFrames observing the point and associated index in keyframe
	std::map<KeyFrame*, size_t> mObservations;

	// Mean viewing direction
	cv::Mat mNormalVector;

	// Best descriptor to fast matching
	cv::Mat mDescriptor;

	// Reference KeyFrame
	KeyFrame* mpRefKF;

	// Tracking counters
	int mnVisible;
	int mnFound;

	// Bad flag (we do not currently erase MapPoint from memory)
	bool mbBad;
	MapPoint* mpReplaced;

	// Scale invariance distance
	float mfMinDistance;
	float mfMaxDistance;

	Map* mpMap;

	std::mutex mMutexPos;
	std::mutex mMutexFeatures;

};

#endif