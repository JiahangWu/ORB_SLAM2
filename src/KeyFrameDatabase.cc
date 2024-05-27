#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include <mutex>


using namespace std;

namespace ORB_SLAM2
{

KeyFrameDatabase::KeyFrameDatabase(const ORBVocabulary &voc):
			mpVoc(&voc)
{
	mvInvertedFile.resize(voc.size());
}

vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
	list<KeyFrame*> lKFsSharingWords;
	
	{
		unique_lock<mutex> lock(mMutex);
		
		for(DBoW2::BowVector::const_iterator vit = F->mBowVec.begin(), vend = F->mBowVec.end(); vit != vend; vit++)
		{
			list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];
			
			for (list<KeyFrame*>::iterator lit=lKFs.begin(), lend = lKFs.end(); lit != lend; lit++)
			{
				KeyFrame *pKFi = *lit;
				if(pKFi->mnRelocQuery != F->mnId)
				{
					pKFi->mnRelocWords = 0;
					pKFi->mnRelocQuery = F->mnId;
					lKFsSharingWords.push_back(pKFi);
				}
				pKFi->mnRelocWords++;
			}
		}
	}
	
	if(lKFsSharingWords.empty())
		return vector<KeyFrame*>();
	
	int maxCommonWords = 0;
	for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
	{
		if((*lit)->mnRelocWords > maxCommonWords)
			maxCommonWords = (*lit)->mnRelocWords;
	}
	
	int minCommonWords = maxCommonWords * 0.8f;
	
	list<pair<float, KeyFrame*> > lScoreAndMatch;
	
	int nscores = 0;
	
	for (list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
	{
		KeyFrame *pKFi = *lit;
		
		if(pKFi->mnRelocWords > minCommonWords)
		{
			nscores++;
			float si = mpVoc->score(F->mBowVec, pKFi->mBowVec);
			pKFi->mRelocScore = si;
			lScoreAndMatch.push_back(make_pair(si, pKFi));
		}
	}
	
	if(lScoreAndMatch.empty())
		return vector<KeyFrame*>();
		
	list<pair<float, KeyFrame*> > lAccScoreAndMatch;
	float bestAccScore = 0;
	
	for(list<pair<float, KeyFrame*> >::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
	{
		KeyFrame *pKFi = it->second;
		vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
		
		float bestScore = it->first;
		float accScore = bestScore;
		KeyFrame *pBestKF = pKFi;
		
		for(vector<KeyFrame*>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
		{
			KeyFrame *pKF2 = *vit;
			if(pKF2->mnRelocQuery != F->mnId)
				continue;
			accScore += pKF2->mRelocScore;
			
			
			if(pKF2->mRelocScore > bestScore)
			{
				pBestKF = pKF2;
				bestScore = pKF2->mRelocScore;
			}
		}
		
		lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
		
		if(accScore > bestAccScore)
			bestAccScore = accScore;
	}
	
	
	float minScoreToRetain = 0.75f * bestAccScore;
	
	set<KeyFrame*> spAlreadyAddedKF;
	vector<KeyFrame*> vpRelocCandidates;
	vpRelocCandidates.reserve(lAccScoreAndMatch.size());
	
	for (list<pair<float, KeyFrame*> >::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
	{
		const float &si = it->first;
		
		if(si > minScoreToRetain)
		{
			KeyFrame *pKFi = it->second;
			if(!spAlreadyAddedKF.count(pKFi));
			{
				vpRelocCandidates.push_back(pKFi);
				spAlreadyAddedKF.insert(pKFi);
			}
		}
	}
	
	return vpRelocCandidates;

}

} // namespace ORB_SLAM2
