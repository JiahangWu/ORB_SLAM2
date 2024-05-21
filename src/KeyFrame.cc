#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId = 0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
	mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
	mfGridElementHeightInv(F.mfGridElementHeightInv), mfGridElementWidthInv(F.mfGridElementWidthInv), 
	mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
	mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
	fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
	mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
	mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
	mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
	mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
	mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
	mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
	mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
	mbToBeErased(false), mbBad(false), 
	mHalfBaseline(F.mb/2),      // 计算双目相机长度的一半
	mpMap(pMap)
{
	mnId = nNextId++;
	mGrid.resize(mnGridCols);
	for (int i = 0; i < mnGridCols; i++)
	{
		mGrid[i].resize(mnGridRows);
		for (int j = 0; j < mnGridRows; j++)
		{
			mGrid[i][j] = F.mGrid[i][j];
		}
	}
	
	SetPose(F.mTcw);
	
}



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

cv::Mat KeyFrame::GetRotation()
{
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0, 3).colRange(0, 3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.rowRange(0, 3).col(3).clone();
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

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
	unique_lock<mutex> lock(mMutexPose);
	Tcw_.copyTo(Tcw);
	cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
	cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
	cv::Mat Rwc = Rcw.t();
	Ow = -Rwc * tcw;
	
	Twc = cv::Mat::eye(4, 4, Tcw.type());
	Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
	Ow.copyTo(Twc.rowRange(0, 3).col(3));
	
	cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
	
	Cw = Twc * center;
}

cv::Mat KeyFrame::GetPose()
{
	unique_lock<mutex> lock(mMutexPose);
	return Tcw.clone();
}


vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mvpMapPoints;
}


int KeyFrame::TrackedMapPoints(const int &minObs)
{
	unique_lock<mutex> lock(mMutexFeatures);
	
	int nPoints = 0;
	const bool bCheckObs = minObs > 0;
	
	for (int i = 0; i < N; i++)
	{
		MapPoint *pMP = mvpMapPoints[i];
		if(pMP)
		{
			if(!pMP->isBad())
			{
				if(bCheckObs)
				{
					if(mvpMapPoints[i]->Observations() >= minObs)
						nPoints++;
				}
				else
					nPoints++;
			}
			
		}
	}
	
	return nPoints;
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
		
		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
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

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
	unique_lock<mutex> lock(mMutexFeatures);
	mvpMapPoints[idx] = pMP;
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

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
	vector<MapPoint*> vpMapPoints;
	cv::Mat Tcw_;
	{
		unique_lock<mutex> lock(mMutexFeatures);
		unique_lock<mutex> lock2(mMutexPose);
		vpMapPoints = mvpMapPoints;
		Tcw_ = Tcw.clone();
	}
	vector<float> vDepths;
	vDepths.reserve(N);
	cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
	Rcw2 = Rcw2.t();
	float zcw = Tcw_.at<float>(2, 3);
	
	for (int i = 0; i < N; i++)
	{
		if(mvpMapPoints[i])
		{
			MapPoint* pMP = mvpMapPoints[i];
			cv::Mat x3Dw = pMP->GetWorldPos();
			float z = Rcw2.dot(x3Dw) + zcw;
			vDepths.push_back(z);
		}
	}
	
	sort(vDepths.begin(), vDepths.end());
	
	return vDepths[(vDepths.size()-1)/q];
}


bool KeyFrame::isBad()
{
	unique_lock<mutex> lock(mMutexConnections);
	return mbBad;
}

} // namespace ORB_SLAM2
