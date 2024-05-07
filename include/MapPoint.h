#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{
class KeyFrame;
class Map;
class Frame;

class MapPoint
{

public:
	
	MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
	
	MapPoint(const cv::Mat & Pos, Map* pMap, Frame* pFrame, const int &idxF);
	
	void SetWorldPos(const cv::Mat &Pos);
	
	cv::Mat GetWorldPos();
	
	cv::Mat GetNormal();
	
	KeyFrame* GetReferenceKeyFrame();
	
	std::map<KeyFrame*, size_t> GetObservations();
	
	int Observations();
	
	void AddObservation(KeyFrame *pKF, size_t idx);
	
	void EraseObservation(KeyFrame *pKF);
	
	int GetIndexInKeyFrame(KeyFrame *pKF);
	
	bool IsInKeyFrame(KeyFrame *pKF);
	
	void SetBadFlag();
	
	bool isBad();
	
	void RePlace(MapPoint *pMP);
	
	MapPoint* GetReplaced();
	
	void IncreaseVisible(int n=1);
	
	void IncreaseFound(int n=1);
	
	float GetFoundRatio();
	
	inline int GetFound()
	{
		return mnFound;
	}
	
	void ComputeDistinctiveDescriptors();
	
	cv::Mat GetDescriptor();
	
	void UpdateNormalAndDepth();
	
	float GetMinDistanceInvariance();
	
	float GetMaxDistanceInvariance();
	
	int PredictScale(const float &currentDis, KeyFrame *pKF);
	
	int PredictScale(const float &currentDist, Frame *pF);
	
public:
	long unsigned int mnId;
	static long unsigned int nNextId;
	const long int mnFirstKFid;
	const long int mnFirstFrame;
	
	int nObs;
	
	float mTrackProjX;
	float mTrackProjY;
	float mTrackProjXR;
	int mnTrackScaleLevel;
	float mTrackViewCos;
	
	bool mbTrackInView;
	
	long unsigned int mnTrackReferenceForFrame;
	
	long unsigned int mnLastFrameSeen;
	
	
	long unsigned int mnBALoaclForKF;
	long unsigned int mnFuesCandiateForKF;
	
	long unsigned int mnLoopPointForKF;
	
	long unsigned int mnCorrectedByKF;
	long unsigned int mnCorrectedReference;
	
	cv::Mat mPosGBA;
	long unsigned int mnBAGlobalForKF;
	
	static std::mutex mGlobalMutex;

protected:
	cv::Mat mWorldPos;
	std::map<KeyFrame*, size_t> mObservations;
	cv::Mat mNormalVector;
	cv::Mat mDescriptor;
	
	KeyFrame* mpRefKF;
	
	int mnVisible;
	int mnFound;
	bool mbBad;
	MapPoint* mpReplaced;
	
	float mfMinDistance;
	float mfMaxDistance;
	
	Map* mpMap;
	std::mutex mMutexPos;
	std::mutex mMutexFeatures;
	
	

};





} // namespace ORB_SLAM2



#endif