
#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
	mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
	mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false)
{
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	
	float fps = fSettings["Camera.fps"];
	if(fps < 1)
		fps = 30;
	mT = 1e3 / fps;
	
	mImageWidth = fSettings["Camera.width"];
	mImageHeight = fSettings["Camera.height"];
	
	if(mImageWidth < 1 || mImageHeight < 1)
	{
		mImageHeight = 480;
		mImageWidth = 640;
	}
	
	mViewpointX = fSettings["Viewer.ViewpointX"];
	mViewPointY = fSettings["Viewer.ViewpointY"];
	mViewpointZ = fSettings["Viewer.ViewpointZ"];
	mViewpointF = fSettings["Viewer.ViewpointF"];
}


} // namespace ORB_SLAM2
