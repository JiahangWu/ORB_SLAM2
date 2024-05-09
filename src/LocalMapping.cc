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

// 设置回环检测线程句柄
void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

// 设置追踪线程句柄
void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
	mbFinished = false;
	
	while(1)
	{
		SetAcceptKeyFrames(false);
		
		if(CheckNewKeyFrames())
		{
			ProcessNewKeyFrame();
			
			MapPointCulling();
			
			CreateNewMapPoints();
		}
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
	list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
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
		
		cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
		cv::Mat Rwc1 = Rcw1.t();
		cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
		cv::Mat Tcw1(3, 4, CV_32F);
		Rcw1.copyTo(Tcw1.colRange(0, 3));
		tcw1.copyTo(Tcw1.col(3));
		
		cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();
		const float &fx1 = mpCurrentKeyFrame->fx;
		const float &fy1 = mpCurrentKeyFrame->fy;
		const float &cx1 = mpCurrentKeyFrame->cx;
		const float &cy1 = mpCurrentKeyFrame->cy;
		const float &invfx1 = mpCurrentKeyFrame->invfx;
		const float &invfy1 = mpCurrentKeyFrame->invfy;
		
		const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;
		
		int nnew = 0;
		for (size_t i = 0; i < vpNeighKFs.size(); i++)
		{
			if(i > 0 && CheckNewKeyFrames())
				return;
			
			KeyFrame *pKF2 = vpNeighKFs[i];
			
			cv::Mat Ow2 = pKF2->GetCameraCenter();
			cv::Mat vBaseline = Ow2 - Ow1;
			const float baseline = cv::norm(vBaseline);
			
			if(!mbMonocular)
			{
				if(baseline < pKF2->mb)
					continue;
			}
			else
			{
				const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
				
				const float ratioBaselineDepth = baseline/medianDepthKF2;
				if(ratioBaselineDepth < 0.01)
					continue;
			}
		}
}

void LocalMapping::RequestStop()
{	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
	unique_lock<mutex> lock2(mMutexNewKFs);
	mbAbortBA = true;
}


void LocalMapping::RequestReset()
{
	{
		unique_lock<mutex> lock(mMutexReset);
		mbResetRequested = true;
	}
	while(1)
	{
		{
			unique_lock<mutex> lock2(mMutexReset);
			if(!mbResetRequested)
				break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}
}


bool LocalMapping::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void LocalMapping::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	unique_lock<mutex> lock2(mMutexFinish);
	if(mbFinished)
		return;
	mbStoped = false;
	mbStopRequested = false;
	for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
		delete *lit;
	mlNewKeyFrames.clear();
	
	cout << "Local Mapping RELEASE" << endl;
}


} // namespace ORB_SLAM2
