#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "ORBVocabulary.h"

#include <mutex>

class KeyFrame;
class Frame;

namespace ORB_SLAM2
{
class KeyFrameDatabase
{
public:
	KeyFrameDatabase(const ORBVocabulary &voc);
	
	void add(KeyFrame* pKF);
	
	void erase(KeyFrame* pKF);
	
	void clear();
	
	std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);
	
	std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame *F);
	

protected:
	const ORBVocabulary* mpVoc;
	std::vector<list<KeyFrame*> > mvInvertedFile;
	
	std::mutex mMutex;


};





} // namespace ORB_SLAM2


#endif