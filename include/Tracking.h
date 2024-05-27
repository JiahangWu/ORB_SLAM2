#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Map.h"
#include "System.h"
#include "Frame.h"
#include "FrameDrawer.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "Viewer.h"
#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Initializer.h"



namespace ORB_SLAM2
{

class FrameDrawer;
class System;
class Map;
class Viewer;
class LocalMapping;
class LoopClosing;


class Tracking
{

public:
	Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
			 KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);
	
	cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
	
	void SetLocalMapper(LocalMapping* pLocalMapper);
	
	void SetLoopClosing(LoopClosing* pLoopClosing);
	
	void SetViewer(Viewer* pViewer);
	
	void ChangeCalibration(const string &strSettingPath);
	
	void InformOnlyTracking(const bool &flag);

public:
	enum eTrackingState{
		SYSTEM_NOT_READY=-1,
		NO_IMAGES_YET=0,
		NOT_INITIALIZED=1,
		OK=2,
		LOST=3
	};
	
	eTrackingState mState;
	eTrackingState mLastProcessedState;
	
	int mSensor;
	
	Frame mCurrentFrame;
	
	cv::Mat mImGray;
	std::vector<int> mvIniLastMatches;
	std::vector<int> mvIniMatches;
	
	std::vector<cv::Point2f> mvbPrevMatched;
	std::vector<cv::Point3f> mvIniP3D;
	Frame mInitialFrame;
	
	list<cv::Mat>mlRelativeFramePoses;
	list<KeyFrame*> mlpReferences;
	list<double> mlFrameTimes;
	list<bool> mlbLost;
	
	bool mbOnlyTracking;
	
	void Reset();
	
protected:
	
	void Track();
	
	void StereoInitialization();
	
	void MonocularInitialization();
	
	void CreateInitialMapMonocular();
	
	void CheckReplacedInLastFrame();
	
	bool TrackReferenceKeyFrame();
	
	void UpdateLastFrame();
	
	bool TrackWithMotionModel();
	
	bool Relocalization();
	
	void UpdateLocalMap();
	
	void UpdateLocalPoints();
	
	void UpdateLocalKeyFrames();
	
	bool TrackLocalMap();
	
	void SearchLocalPoints();
	
	bool NeedNewKeyFrame();
	
	void CreateNewKeyFrame();
	
	bool mbVO;
	
	LocalMapping* mpLocalMapper;
	
	LoopClosing* mpLoopClosing;
	
	ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
	
	ORBextractor* mpIniORBextractor;
	
	ORBVocabulary* mpORBVocabulary;
	
	KeyFrameDatabase* mpKeyFrameDB;
	
	Initializer* mpInitializer;
	
	KeyFrame* mpReferenceKF;
	
	std::vector<KeyFrame*> mvpLocalKeyFrames;
	std::vector<MapPoint*> mvpLocalMapPoints;
	
	System* mpSystem;
	
	Viewer* mpViewer;
	FrameDrawer* mpFrameDrawer;
	MapDrawer* mpMapDrawer;
	
	Map* mpMap;
	
	cv::Mat mK;
	cv::Mat mDistCoef;
	float mbf;
	
	int mMinFrames;
	int mMaxFrames;
	
	float mThDepth;
	
	float mDepthMapFactor;
	
	int mnMatchesInliers;
	
	KeyFrame* mpLastKeyFrame;
	Frame mLastFrame;
	
	unsigned int mnLastKeyFrameId;
	unsigned int mnLastRelocFrameId;
	
	cv::Mat mVelocity;
	
	bool mbRGB;
	
	list<MapPoint*>mlpTemporalPoints;

};

} // namespace ORB_SLAM2



#endif