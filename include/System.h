#ifndef SYSTEM_H
#define SYSTEM_H

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ORB_SLAM2 module
#include "Tracking.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "FrameDrawer.h"
#include "Viewer.h"
#include "LocalMapping.h"
#include "LoopClosing.h"



using namespace std;

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;


class System
{
public:
	enum eSensor{
		MONOCULAR=0,
		STEREO=1,
		RGBD=2
	};
	
	
public:
	System(const string &strVocFile,
		   const string &strSettingFile,
		   const eSensor sensor,
		   const bool bUseViewer = true);
		   
	cv::Mat TrackMonocular(const cv::Mat &im,
						   const double &timestamp);
						   
	void ActivateLocalizationMode();
	
	void DeactivateLocalizationMode();
	
	bool MapChanged();
	
	void Reset();
	
	void Shutdown();
	
	void SaveTrajectoryTUM(const string &filename);
	
	void SaveKeyFrameTrajectoryTUM(const string &filename);
	
	int GetTrackingState();
	std::vector<MapPoint*> GetTrackedMapPoints();
	std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
	

private:
	
	//注意变量命名方式，类的变量有前缀m，如果这个变量是指针类型还要多加个前缀p，
	//如果是进程那么加个前缀t

	eSensor mSensor;
	
	ORBVocabulary* mpVocabulary;
	
	KeyFrameDatabase* mpKeyFrameDatabase;
	
	Map* mpMap;
	
	Tracking* mpTracker;
	
	LocalMapping* mpLocalMapper;
	
	LoopClosing* mpLocalCloser;
	
	Viewer* mpViewer;
	FrameDrawer* mpFrameDrawer;
	MapDrawer* mpMapDrawer;
	
	std::thread* mpLocalMapping;
	std::thread* mptLoopClosing;
	std::thread* mptViewer;
	
	std::mutex mMutexReset;
	bool mbReset;
	
	std::mutex mMutexMode;
	bool mbActivateLocalizationMode;
	bool mbDeactivateLocalizationMode;
	
	int mTrackingState;
	std::vector<MapPoint*> mTrackedMapPoints;
	std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
	std::mutex mMutexState;
	
	
	
};




}
#endif