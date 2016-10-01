/***************************************
*
*
*
*
****************************************/


#include "Initializer.h"

#include "DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"
#include <thread>

Initializer::Initializer(const Frame& ReferenceFrame, float sigma /* = 1.0 */, int iterations /* = 200 */)
{
	mK = ReferenceFrame.mK.clone();

	mvKeys1 = ReferenceFrame.mvKeysUn;

	mSigma = sigma;
	mSigma2 = sigma * sigma;
	mMaxIterations = iterations;
}


bool Initializer::Initialize(const Frame& CurrentFrame, const vector<int>& vMatches12,
	cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f>& vP3D, vector<bool>& vbTriangulated)
{
	// Fill structures with current keypoints and matches with reference frame
	// Reference Frame: 1, Current Frame: 2

	mvKeys2 = CurrentFrame.mvKeysUn;

	mvMatches12.clear();
	mvMatches12.reserve(mvKeys2.size());
	mvbMatched1.resize(mvKeys1.size());

	for (size_t i = 0, iend = vMatches12.size(); i < iend; ++i)
	{
		if (vMatches12[i] >= 0)
		{
			mvMatches12.push_back(make_pair(i, vMatches12[i]));
			mvbMatched1[i] = true;
		}
		else
			mvbMatched1[i] = false;
	}

	const int N = mvMatches12.size();

	// Indices for minimum set selection
	vector<size_t> vAllIndices;
	vAllIndices.reserve(N);
	vector<size_t> vAvaliableIndices;

	for (int i = 0; i < N; ++i)
	{
		vAllIndices.push_back(i);
	}

	// Generate sets of 8 points for each RANSAC iteration
	mvSets = vector<vector<size_t> >(mMaxIterations, vector<size_t>(8, 0));

	DUtils::Random::SeedRandOnce(0);

	for (int it = 0; it < mMaxIterations; ++it)
	{
		vAvaliableIndices = vAllIndices;

		// Select a minimum set
		for (size_t j = 0; j < 8; ++j)
		{
			int randi = DUtils::Random::RandomInt(0, vAvaliableIndices.size() - 1);
			int idx = vAvaliableIndices[randi];

			mvSets[it][j] = idx;

			vAvaliableIndices[randi] = vAvaliableIndices.back();
			vAvaliableIndices.pop_back();
		}
	}

	// Launch threads to compute in parallel a fundamental matrix and a homography
	vector<bool> vbMatchesInliersH, vbMatchesInliersF;
	float SH, SF;
	cv::Mat H, F;

	thread threadH(&Initializer::FindHomography, this, ref(vbMatchesInliersH), ref(SH), ref(H));
	thread threadF(&Initializer::FindFundamental, this, ref(vbMatchesInliersF), ref(SF), ref(F));

	// Wait until both threads have finished
	threadH.join();
	threadF.join();

	// Compute ratio of scores
	float RH = SH / (SH + SF);

	// Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
	if (RH > 0.40)
	{
		return ReconstructH(vbMatchesInliersH, H, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
	}
	else return ReconstructF(vbMatchesInliersF, F, mK, R21, t21, vP3D, vbTriangulated, 1.0, 5.0);

	return false;
}