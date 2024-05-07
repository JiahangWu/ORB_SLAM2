
#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
// #include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2
{
class Tracking;
class Map;
class LoopClosing;

class LocalMapping
{
public:
	
	LocalMapping(Map* pMap, const float bMonocular);
	
	void SetLoopCloser(LoopClosing* pLoopCloser);
	
	void SetTracker(Tracking* pTracker);
	
	void Run();
	
	void InsertKeyFrame(KeyFrame* pKF);
	
	void RequestStop();
	
	void RequestReset();
	
	bool Stop();
	
	void Release();
	
	bool isStopped();
	
	bool stopRequested();
	
	bool AcceptKeyFrames(bool flag);
	
	bool SetNotStop(bool flag);
	
	void InterruptBA();
	
	void RequestFinish();
	
	bool isFinished();
	
	int KeyframesInQueue(){
		unique_lock<std::mutex> lock(mMutexNewKFs);
		return mlNewKeyFrames.size();
	}
	
protected:
	
	bool CheckNewKeyFrames();
	
	void ProcessNewKeyFrame();
	
	void CreateNewMapPoints();
	
	void MapPointCulling();
	
	void SearchInNeighbors();
	
	void KeyFrameCulling();
	
	cv::Mat ComputerF12(KeyFrame* &pKF1, KeyFrame* &pKF2);
	
	cv::Mat SkewSymetricMatrix(const cv::Mat &v);
	
	bool mbMonocular;
	
	void ResetIfRequested();
	
	bool mbResetRequested;
	
	std::mutex mMutexReset;
	
	bool CheckFinish();
	
	void SetFinish();
	
	bool mbFinishRequested;
	
	bool mbFinished;
	
	std::mutex mMutexFinish;
	
	Map* mpMap;
	
	LoopClosing* mpLoopCloser;
	
	Tracking* mpTracker;
	
	std::list<KeyFrame*> mlNewKeyFrames;
	
	KeyFrame* mpCurrentKeyFrame;
	
	std::list<MapPoint*> mlpRecentAddedMapPoints;
	
	std::mutex mMutexNewKFs;
	
	bool mbAbortBA;
	
	bool mbStopped;
	
	bool mbStopRequested;
	
	bool mbNotStop;
	
	std::mutex mMutexStop;
	
	bool mbAcceptKeyFrames;
	
	std::mutex mMutexAccept;
	
};


}

#endif