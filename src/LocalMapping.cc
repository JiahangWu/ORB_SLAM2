#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"

#include<mutex>

namespace ORB_SLAM2
{

LocalMapping::LocalMapping(Map *pMap, const float bMonocular):
	mbMonocular(bMonocular), mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
	mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
	
}


void LocalMapping::Run()
{
	mbFinished = false;
	
	while(1)
	{
		SetAcceptKeyFrames(false);
	}
	if(CheckNewKeyFrames())
	{
		ProcessNewKeyFrame();
		
		MapPointCulling();
		
		CreateNewMapPoints();
	}
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
	unique_lock<mutex> lock(mMutexAccept);
	mbAcceptKeyFrames=flag;
}

bool LocalMapping::CheckNewKeyFrames()
{
	unique_lock<mutex> lock(mMutexNewKFs);
	return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
	{
		unique_lock<mutex> lock(mMutexNewKFs);
		mpCurrentKeyFrame = mlNewKeyFrames.front();
		mlNewKeyFrames.pop_front();
	}
	
	mpCurrentKeyFrame->ComputeBoW();
	
	const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
	
	for (size_t i = 0; i < vpMapPointMatches.size(); i++)
	{
		MapPoint* pMP = vpMapPointMatches[i];
		if(pMP)
		{
			if(!pMP->isBad())
			{
				if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
				{
					pMP->AddObservation(mpCurrentKeyFrame, i);
					
					pMP->UpdateNormalAndDepth();
					
					pMP->ComputeDistinctiveDescriptors();
				}
				else
				{
					mlpRecentAddedMapPoints.push_back(pMP);
				}
			}
		}
	}
	
	mpCurrentKeyFrame->UpdateConnections();
	
	mpMap->AddKeyFrame(mpCurrentKeyFrame);
	
}

void LocalMapping::MapPointCulling()
{
	list<MapPoint*>::iterator list = mlpRecentAddedMapPoints.begin();
	const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;
	
	int nThObs;
	if(mbMonocular)
		nThObs = 2;
	else
		nThObs = 3;
	const int cnThObs = nThObs;
		
	while (lit != mlpRecentAddedMapPoints.end())
	{
		MapPoint *pMP = *lit;
		if(pMP->isBad())
		{
			lit = mlpRecentAddedMapPoints.erase(lit);
		}
		else if(pMP->GetFoundRatio()<0.25f)
		{
			pMP->SetBadFlag();
			lit = mlpRecentAddedMapPoints.erase(lit);
		}
		else if(((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs)
		{
			pMP->SetBadFlag();
			lit = mlpRecentAddedMapPoints.erase(lit);
		}
		else if(((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
			lit = mlpRecentAddedMapPoints.erase(lit);
		else
			lit++;
	}
	
}


void LocalMapping::CreateNewMapPoints()
{
	int nn = 10;
	if(mbMonocular)
		nn=20;
		
		const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
		
		ORBmatcher matcher(0.6, false);
}


} // namespace ORB_SLAM2
