#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{


cv::Mat KeyFrame::GetCameraCenter()
{
	unique_lock<mutex> lock(mMutexPose);
	return Ow.clone();
}

KeyFrame* KeyFrame::GetParent()
{
	unique_lock<mutex> lockCon(mMutexConnections);
	return mpParent;
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
	unique_lock<mutex> lockCon(mMutexConnections);
	mspChildrens.insert(pKF);
}

cv::Mat KeyFrame::GetPoseInverse()
{
	unique_lock<mutex> lock(mMutexPose);
	return Twc.clone();
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
	unique_lock<mutex> lockCon(mMutexConnections);
	return mspLoopEdge;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
	unique_lock<mutex> lock(mMutexConnections);
	
	if((int)mvpOrderedConnectedKeyFrames.size() < N)
		return mvpOrderedConnectedKeyFrames;
	else
		return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+N);
}


vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
	unique_lock<mutex> lock(mMutexConnections);
	
	if(mvpOrderedConnectedKeyFrames.empty())
		return vector<KeyFrame*>();
		
	vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),
										   mvOrderedWeights.end(),
										   w,
										   KeyFrame::weightComp);
	
	if(it == mvOrderedWeights.end() && *mvOrderedWeights.rbegin()<w)
		return vector<KeyFrame*>();
	else
	{
		int n = it-mvOrderedWeights.begin();
		return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
	}
}

void KeyFrame::UpdateBestCovisibles()
{
	unique_lock<mutex> lock(mMutexConnections);
	
	vector<pair<int, KeyFrame*> > vPairs;
	vPairs.reserve(mConnectedKeyFrameWeights.size());
	for(map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit != mend; mit++)
		vPairs.push_back(make_pair(mit->second, mit->first));
	
	sort(vPairs.begin(), vPairs.end());
	
	list<KeyFrame*> lKFs;
	list<int>lWs;
	
	for (size_t i = 0, iend = vPairs.size(); i < iend; i++)
	{
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}
	
	mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
	mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}



void KeyFrame::ComputeBoW()
{
	if(mBowVec.empty() || mFeatVec.empty())
	{
		vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
		mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
	}
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints;
}

void KeyFrame::UpdateConnections()
{
	map<KeyFrame*, int>KFcounter;
	vector<MapPoint*> vpMP;
	{
		unique_lock<mutex> lockMPs(mMutexFeatures);
		vpMP = mvpMapPoints;
	}
	
	for(vector<MapPoint*>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit !=vend; vit++)
	{
		MapPoint *pMP = *vit;
		
		if(!pMP)
			continue;
			
		if(pMP->isBad())
			continue;
			
		map<KeyFrame*, size_t> observations = pMP->GetObservations();
		
		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), observations.end(); mit != mend; mit++)
		{
			if(mit->first->mnId == mnId)
				continue;
			KFcounter[mit->first]++;
		}
	}
	
	if(KFcounter.empty())
		return;
	
	int nmax=0;
	KeyFrame *pKFmax=NULL;
	
	int th = 15;
	
	vector<pair<int, KeyFrame*> > vPairs;
	vPairs.reserve(KFcounter.size());
	
	for(map<KeyFrame*, int>::iterator mit = KFcounter.begin(), mend = KFcounter.end(); mit != mend; mit++)
	{
		if(mit->second > nmax)
		{
			nmax = mit->second;
			pKFmax = mit->first;
		}
		
		if(mit->second >= th)
		{
			vPairs.push_back(make_pair(mit->second, mit->first));
			(mit->first)->AddConnection(this, mit->second);
		}
	}
	
	if(vPairs.empty())
	{
		vPairs.push_back(make_pair(nmax, pKFmax));
		pKFmax->AddConnection(this, nmax);
	}
	
	sort(vPairs.begin(), vPairs.end());
	
	list<KeyFrame*> lKFs;
	list<int> lWs;
	for (size_t i = 0; i < vPairs.size(); i++)
	{
		lKFs.push_front(vPairs[i].second);
		lWs.push_front(vPairs[i].first);
	}
	
	{
		unique_lock<mutex> lockCon(mMutexConnections);
		mConnectedKeyFrameWeights = KFcounter;
		mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
		mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
		
		if(mbFirstConnection && mnId != 0)
		{
			mpParent = mvpOrderedConnectedKeyFrames.front();
			mpParent->AddChild(this);
			mbFirstConnection = false;
		}
	}
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
	{
		unique_lock<mutex> lock(mMutexConnections);
		if(!mConnectedKeyFrameWeights.count(pKF))
			mConnectedKeyFrameWeights[pKF] = weight;
		else if(mConnectedKeyFrameWeights[pKF] != weight)
			mConnectedKeyFrameWeights[pKF] = weight;
		else
			return;
	}
	UpdateBestCovisibles();
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
	int idx = pMP->GetIndexInKeyFrame(this);
	if(idx>=0)
		mvpMapPoints[idx] = static_cast<MapPoint*>(NULL);
}


} // namespace ORB_SLAM2
