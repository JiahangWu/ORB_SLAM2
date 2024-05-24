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
	
	
	
	
	
	
	
}

} // namespace ORB_SLAM2
