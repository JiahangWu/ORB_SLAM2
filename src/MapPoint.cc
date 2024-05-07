#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{


long unsigned int MapPoint::nNextId = 0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos,
				   KeyFrame *pRefKF,
				   Map *pMap):
	mnFirstKFid(pRefKF->mnId),
	mnFirstFrame(pRefKF->mnFrameId),
	nObs(0),
	mnTrackReferenceForFrame(0),
	mnLastFrameSeen(0),
	mnBALoaclForKF(0),
	mnFuesCandiateForKF(0),
	mnLoopPointForKF(0),
	mnCorrectedByKF(0),
	mnCorrectedReference(0),
	mnBAGlobalForKF(0),
	mpRefKF(pRefKF),
	mnVisible(1),
	mnFound(1),
	mbBad(false),
	mpReplaced(static_cast<MapPoint*>(NULL)),
	mfMinDistance(0),
	mfMaxDistance(0),
	mpMap(pMap)
{
	Pos.copyTo(mWorldPos);
	mNormalVector = cv::Mat::zeros(3, 1, CV_32F);
	
	
	unique_lock<mutex> lock(mpMap->mMutexPointCreation);
	mnId = nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF):
			mnFirstKFid(-1),
			mnFirstFrame(pFrame->mnId),
			nObs(0),
			mnTrackReferenceForFrame(0),
			mnLastFrameSeen(0),
			mnBALoaclForKF(0),
			mnFuesCandiateForKF(0),
			mnLoopPointForKF(0),
			mnCorrectedByKF(0),
			mnCorrectedReference(0),
			mnBAGlobalForKF(0),
			mpRefKF(static_cast<KeyFrame*>(NULL)),
			mnVisible(1),
			mnFound(1),
			mbBad(false),
			mpReplaced(NULL),
			mpMap(pMap)
{
	Pos.copyTo(mWorldPos);
	cv::Mat Ow = pFrame->GetCameraCenter();
	mNormalVector = mWorldPos - Ow;
	mNormalVector = mNormalVector/cv::norm(mNormalVector);
	
	cv::Mat PC = Pos - Ow;
	
	const float dist = cv::norm(PC);
	const int level = pFrame->mvKeysUn[idxF].octave;
	const float levelScaleFactor = pFrame->mvScaleFactors[level];
	const int nLevels = pFrame->mnScaleLevels;
	
	mfMaxDistance = dist * levelScaleFactor;
	mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];
	
	pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);
	unique_lock<mutex> lock(mpMap->mMutexPointCreation);
	
	mnId = nNextId++;
}
	
cv::Mat MapPoint::GetWorldPos()
{
	unique_lock<mutex> lock(mMutexPos);
	return mWorldPos.clone();
}

bool MapPoint::isBad()
{
	unique_lock<mutex> lock(mMutexFeatures);
	unique_lock<mutex> lock2(mMutexPos);
	return mbBad;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mObservations.count(pKF);
}

void MapPoint::AddObservation(KeyFrame *pKF, size_t idx)
{
	unique_lock<mutex>lock(mMutexFeatures);
	if(mObservations.count(pKF))
		return;
	mObservations[pKF]=idx;
	if(pKF->mvuRight[idx]>=0)
		nObs+=2;
	else
		nObs++;
}

void MapPoint::UpdateNormalAndDepth()
{
	map<KeyFrame*, size_t> observations;
	KeyFrame *pRefKF;
	cv::Mat Pos;
	{
		unique_lock<mutex> lock1(mMutexFeatures);
		unique_lock<mutex> lock2(mMutexPos);
		if(mbBad)
			return;
		
		observations = mObservations;
		pRefKF = mpRefKF;
		Pos = mWorldPos.clone();
	}
	
	if(observations.empty())
		return;
		
	cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
	int n = 0;
	for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
	{
		KeyFrame *pKF = mit->first;
		cv::Mat Owi = pKF->GetCameraCenter();
		cv::Mat normali = mWorldPos - Owi;
		normal = normal + normal / cv::norm(normali);
		n++;
	}
	cv::Mat PC = Pos - pRefKF->GetCameraCenter();
	const float dist = cv::norm(PC);
	const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
	const float levelScaleFactor = pRefKF->mvScaleFactors[level];
	const int nLevels = pRefKF->mnScaleLevels;
	
	{
		unique_lock<mutex> lock3(mMutexPos);
		mfMaxDistance = dist * levelScaleFactor;
		mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
		mNormalVector = normal / n;
	}
}

void MapPoint::ComputeDistinctiveDescriptors()
{
	vector<cv::Mat> vDescriptors;
	
	map<KeyFrame*, size_t> observations;
	
	{
		unique_lock<mutex> lock1(mMutexFeatures);
		if(mbBad)
			return;
		observations= mObservations;
	}
	
	if(observations.empty())
		return;
	
	vDescriptors.reserve(observations.size());
	
	for (map<KeyFrame*, size_t>::iterator mit = observations.begin(), mend=observations.end(); mit != mend; mit++)
	{
		KeyFrame *pKF = mit->first;
		if(!pKF->isBad())
			vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
	}
	
	if(vDescriptors.empty())
		return;
		
	const size_t N = vDescriptors.size();
	
	std::vector<std::vector<float> > Distances;
	Distances.resize(N, std::vector<float>(N, 0));
	for (size_t i = 0; i < N; i++)
	{
		Distances.resize(N, vector<float>(N, 0));
		Distances[i][i] = 0;
		for (size_t j = i+1; j < N; j++)
		{
			int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[i]);
			Distances[i][j] = distij;
			Distances[j][i] = distij;
		}
	}
	
	int BestMedian = INT_MAX;
	int BestIdx = 0;
	for (size_t i = 0; i < N; i++)
	{
		vector<int> vDists(Distances[i].begin(), Distances[i].end());
		sort(vDists.begin(), vDists.end());
		
		int median = vDists[0.5*(N-1)];
		
		if(median < BestMedian)
		{
			BestMedian = median;
			BestIdx = i;
		}
	}
	
	{
		unique_lock<mutex> lock(mMutexFeatures);
		mDescriptor = vDescriptors[BestIdx].clone();
	}
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return mObservations;
}

void MapPoint::SetBadFlag()
{
	map<KeyFrame*, size_t> obs;
	{
		unique_lock<mutex> lock1(mMutexFeatures);
		unique_lock<mutex> lock2(mMutexPos);
		mbBad = true;
		obs = mObservations;
		mObservations.clear();
	}
	
	for(map<KeyFrame*, size_t>::iterator mit=obs.begin(), mend=obs.end(); mit != mend; mit++)
	{
		KeyFrame *pKF = mit->first;
		pKF->EraseMapPointMatch(mit->second);
	}
	
	mpMap->EraseMapPoint(this);
}


float MapPoint::GetFoundRatio()
{
	unique_lock<mutex> lock(mMutexFeatures);
	return static_cast<float>(mnFound)/mnVisible;
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
	unique_lock<mutex> lock(mMutexFeatures);
	if(mObservations.count(pKF))
		return mObservations[pKF];
	else
		return -1;
}



} // namespace ORB_SLAM2
