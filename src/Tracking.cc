#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

// #include "Optimizer.h"
// #include "PnPsolver.h"

#include <iostream>
#include <cmath>
#include <mutex>

using namespace std;

namespace ORB_SLAM2
{
Tracking::Tracking(
	System *pSys,
	ORBVocabulary* pVoc,
	FrameDrawer *pFrameDrawer,
	MapDrawer *pMapDrawer,
	Map *pMap,
	KeyFrameDatabase *pKFDB,
	const string &strSettingPath,
	const int sensor):
		mState(NO_IMAGES_YET),
		mSensor(sensor),
		mbOnlyTracking(false),
		mbVO(false),
		mpORBVocabulary(pVoc),
		mpKeyFrameDB(pKFDB),
		mpInitializer(static_cast<Initializer*>(NULL)),
		mpSystem(pSys),
		mpViewer(NULL),
		mpFrameDrawer(pFrameDrawer),
		mpMapDrawer(pMapDrawer),
		mpMap(pMap),
		mnLastRelocFrameId(0)
{
	cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
	float fx = fSettings["Camera.fx"];
	float fy = fSettings["Camera.fy"];
	float cx = fSettings["Camera.cx"];
	float cy = fSettings["Camera.cy"];
	
	
	cv::Mat K = cv::Mat::eye(3,3,CV_32F);
	K.at<float>(0,0) = fx;
	K.at<float>(1,1) = fy;
	K.at<float>(0,2) = cx;
	K.at<float>(1,2) = cy;
	K.copyTo(mK);
	
	cv::Mat DistCoef(4, 1, CV_32F);
	DistCoef.at<float>(0) = fSettings["Camera.k1"];
	DistCoef.at<float>(1) = fSettings["Camera.k2"];
	DistCoef.at<float>(2) = fSettings["Camera.p1"];
	DistCoef.at<float>(3) = fSettings["Camera.p2"];
	const float k3 = fSettings["Camera.k3"];
	
	
	if(k3!=0)
	{
		DistCoef.resize(5);
		DistCoef.at<float>(4) = k3;
	}
	
	mbf = fSettings["Camera.bf"];
	
	float fps = fSettings["Camera.fps"];
	if(fps==0)
		fps = 30;
	
	mMinFrames = 0;
	mMaxFrames = fps;
	
	//输出
	cout << endl << "Camera Parameters: " << endl;
	cout << "- fx: " << fx << endl;
	cout << "- fy: " << fy << endl;
	cout << "- cx: " << cx << endl;
	cout << "- cy: " << cy << endl;
	cout << "- k1: " << DistCoef.at<float>(0) << endl;
	cout << "- k2: " << DistCoef.at<float>(1) << endl;
	if(DistCoef.rows==5)
		cout << "- k3: " << DistCoef.at<float>(4) << endl;
	cout << "- p1: " << DistCoef.at<float>(2) << endl;
	cout << "- p2: " << DistCoef.at<float>(3) << endl;
	cout << "- fps: " << fps << endl;
	
	
	int nRGB = fSettings["Camera.RGB"];
	mbRGB = nRGB;
	
	if(mbRGB)
		cout << "- color order: RGB (ignored if grayscale)" << endl;
	else
		cout << "- color order: BGR (ignored if grayscale)" << endl;
	
	int nFeatures = fSettings["ORBextractor.nFeatures"];
	float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
	int nLevels = fSettings["ORBextractor.nLevels"];
	int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
	int fMinThFAST = fSettings["ORBextractor.minThFAST"];
	
	mpORBextractorLeft = new ORBextractor(
		nFeatures,
		fScaleFactor,
		nLevels,
		fIniThFAST,
		fMinThFAST);
		
	if(sensor == System::MONOCULAR)
		mpIniORBextractor = new ORBextractor(2*nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
	
	cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;
	
	
	
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
	mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
	mpLoopClosing = pLoopClosing;
}




} // namespace ORB_SLAM2
