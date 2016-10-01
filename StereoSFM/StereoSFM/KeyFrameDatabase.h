/***************************************
*
*
*
*
****************************************/

#ifndef KEYFRAMEDATABASE_H__
#define KEYFRAMEDATABASE_H__

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include <vector>
#include <list>
#include <set>
#include <mutex>

class KeyFrame;
class Frame;

class KeyFrameDatabase
{
public:
	KeyFrameDatabase(const ORBVocabulary& voc);
	~KeyFrameDatabase(){};

public:
	void add(KeyFrame* pKF);
	void erase(KeyFrame* pKF);
	void clear();

	// Loop Detection
	std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

	// Relocalization
	std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:
	// Associated vocabulary
	const ORBVocabulary* mpVoc;

	// Inverted file
	std::vector<list<KeyFrame*> >mvInvertedFile;

	// Mutex
	std::mutex mMutex;
};

#endif // end if