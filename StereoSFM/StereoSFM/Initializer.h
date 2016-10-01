/***************************************
*
*
*
*
****************************************/

#ifndef INITIALIZER_H__
#define INITIALIZER_H__

#include "Frame.h"
#include "opencv2/opencv.hpp"

class Initializer
{
	typedef pair<int, int> Match;

public:
	// Fix the reference frame
	Initializer(const Frame& ReferenceFrame, float sigma = 1.0, int iterations = 200);

	// Compute in parallel a fundamental matrix and a homography
	// Select a model and tries to recover the motion and the structure from motion
	bool Initialize(const Frame& CurrentFrame, const vector<int>& vMatches12,
		cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f>& vP3D, vector<bool>& vbTriangulated);

private:
	void FindHomography(vector<bool>& vbMatchesInliers, float &score, cv::Mat &H21);
	void FindFundamental(vector<bool>& vbInliers, float& score, cv::Mat& F21);

	cv::Mat ComputeH21(const vector<cv::Point2f>& vP1, const vector<cv::Point2f>& vP2);
	cv::Mat ComputeF21(const vector<cv::Point2f>& vP1, const vector<cv::Point2f>& vP2);

	float CheckHomography(const cv::Mat& H21, const cv::Mat &H12, vector<bool>& vMatchesInliers, float sigma);

	float CheckFundamental(const cv::Mat& F21, vector<bool>& vbMatchesInliers, float sigma);

	bool ReconstructF(vector<bool>& vbMatchesInliers, cv::Mat& F21, cv::Mat& K,
		cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f>& vP3D, vector<bool>& vbTriangulated, float minParallax, int minTriangulated);

	bool ReconstructH(vector<bool>& vbMatchesInliers, cv::Mat& H21, cv::Mat& K,
		cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f>& vP3D, vector<bool>& vbTriangulated, float minParallax, int minTriangulated);

	void Triangulate(const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const cv::Mat& P1, const cv::Mat& x3D);

	void Normalize(const vector<cv::KeyPoint>& vKeys, vector<cv::Point2f>& vNormalizedPoints, cv::Mat& T);

	int checkRT(const cv::Mat& R, const cv::Mat& t, const vector<cv::KeyPoint>& vKeys1, const vector<cv::KeyPoint>& vKey2,
		const vector<Match>& vMatches12, vector<bool>& vbInlies,
		const cv::Mat& K, vector<cv::Point3f>& vP3D, float th2, vector<bool>& vbGood, float &parallax);

	void DecomposeE(const cv::Mat& E, cv::Mat& R1, cv::Mat& R2, cv::Mat& t);

	// keypoints from Reference Frame (Frame 1)
	vector<cv::KeyPoint> mvKeys1;

	// keypoint s from Current Frame (Frame 2)
	vector<cv::KeyPoint> mvKeys2;

	// Current Matches from Reference to Current
	vector<Match> mvMatches12;
	vector<bool> mvbMatched1;

	// Calibration
	cv::Mat mK;

	// Standard Deviation and Variance
	float mSigma, mSigma2;

	// Ransac max iterations
	int mMaxIterations;

	// Ransac sets
	vector<vector<size_t> > mvSets;

};

#endif