#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Viewer
{

public:
	Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);
	
	void Run();
	
	void RequestFinish();
	
	void RequestStop();
	
	bool isFinished();
	
	bool isStopped();
	
	void Release();
	
private:

	bool Stop();
	
	System* mpSystem;
	
	FrameDrawer* mpFrameDrawer;
	
	MapDrawer* mpMapDrawer;
	
	Tracking* mpTracker;
	
	double mT;
	
	float mImageWidth, mImageHeight;
	
	float mViewpointX, mViewPointY, mViewpointZ, mViewpointF;
	
	bool CheackFinish();
	
	void SetFinish();
	
	bool mbFinishRequested;
	
	bool mbFinished;
	
	std::mutex mMutexFinish;
	
	bool mbStopped;
	
	bool mbStopRequested;
	
	std::mutex mMutexStop;

};


} // namespace ORB_SLAM2



#endif
