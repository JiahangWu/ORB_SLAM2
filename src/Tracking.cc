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

void Tracking::SetViewer(Viewer *pViewer)
{
	mpViewer = pViewer;
}

void Tracking::Reset()
{
	if(mpViewer)
	{
		mpViewer->RequestStop();
		while(!mpViewer->isStopped())
			std::this_thread::sleep_for(std::chrono::milliseconds(3));
	}
	
	cout << "System Reseting" << endl;
	
	cout << "Reseting Local Mapper...";
	mpLocalMapper->RequestReset();
	cout << " done" << endl;
	
	cout << "Reseting Loop Closing...";
	mpLoopClosing->RequestReset();
	cout << " done" << endl;
	
	mpMap->clear();
	
	KeyFrame::nNextId = 0;
	Frame::nNextId = 0;
	mState = NO_IMAGES_YET;
	
	if(mpInitializer)
	{
		delete mpInitializer;
		mpInitializer = static_cast<Initializer*>(NULL);
	}
	
	mlRelativeFramePoses.clear();
	mlpReferences.clear();
	mlFrameTimes.clear();
	mlbLost.clear();
	
	if(mpViewer)
		mpViewer->Release();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
	mIMGray = im;
	
	if(mImGray.channels() == 3)
	{
		if(mbRGB)
			cv::cvtColor(mImGray, mImGray, CV_RGB2GRAY);
		else
			cv::cvtColor(mImGray, mImGray, CV_BGR2GRAY);
	}
	else if(mImGray.channels() == 4)
	{
		if(mbRGB)
			cv::cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
		else
			cv::cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
	}
	
	if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
		mCurrentFrame = Frame(
				mImGray,
				timestamp,
				mpIniORBextractor,
				mpORBVocabulary,
				mK,
				mDistCoef,
				mbf,
				mThDepth);
	else
		mCurrentFrame = Frame(
				mImGray,
				timestamp,
				mpORBextractorLeft,
				mpORBVocabulary,
				mK,
				mDistCoef,
				mbf,
				mThDepth);
	
	Track();
	
	return mCurrentFrame.mTcw.clone();
}


void Tracking::Track()
{
	if(mState == NO_IMAGES_YET)
	{
		mState = NOT_INITIALIZED;
	}
	
	mLastProcessedState = mState;
	
	unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
	
	if(mState == NOT_INITIALIZED)
	{
		if(mSensor == System::STEREO || mSensor == System::RGBD)
			StereoInitialization();
		else
			MonocularInitialization();
		
	}
	
	
}

void Tracking::StereoInitialization()
{
	
}

void Tracking::MonocularInitialization()
{
	if(!mpInitializer)
	{
		if(mCurrentFrame.mvKeys.size() > 100)
		{
			mInitialFrame = Frame(mCurrentFrame);
			mLastFrame = Frame(mCurrentFrame);
			
			mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
			for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
				mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;
			
			if(mpInitializer)
				delete mpInitializer;
			
			mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);
			fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
			
			return;
		}
	}
	else
	{
		if((int)mCurrentFrame.mvKeys.size() <= 100)
		{
			delete mpInitializer;
			mpInitializer = static_cast<Initializer*>(NULL);
			fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
			return;
		}
		
		ORBmatcher matcher(0.9, true);
		
		int nmatches = matcher.SearchForInitialization(
			mInitialFrame, mCurrentFrame,
			mvbPrevMatched,
			mvIniMatches,
			100);
		
		if(nmatches < 100)
		{
			delete mpInitializer;
			mpInitializer = static_cast<Initializer*>(NULL);
			return;
		}
		
		cv::Mat Rcw;
		cv::Mat tcw;
		vector<bool> vbTriangulated;
		
		if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, vbTriangulated))
		{
			for (size_t i = 0, iend = mvIniLastMatches.size(); i < iend; i++)
			{
				if(mvIniMatches[i] >= 0 && !vbTriangulated[i])
				{
					mvIniMatches[i] = -1;
					nmatches--;
				}
			}
			
			mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
			
			cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
			Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0,3));
			tcw.copyTo(Tcw.rowRange(0, 3).col(3));
			mCurrentFrame.SetPose(Tcw);
			
			CreateInitialMapMonocular();
		}
	}
}


void Tracking::CreateInitialMapMonocular()
{
	KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
	KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);
	
	pKFini->ComputeBoW();
	pKFcur->ComputeBoW();
	
	mpMap->AddKeyFrame(pKFini);
	mpMap->AddKeyFrame(pKFcur);
	
	for (size_t i = 0; i < mvIniMatches.size(); i++)
	{
		if(mvIniMatches[i] < 0)
			continue;
		
		cv::Mat worldPos(mvIniP3D[i]);
		
		MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);
		
		pKFini->AddMapPoint(pMP, i);
		pKFcur->AddMapPoint(pKFcur, mvIniMatches[i]);
		
		pMP->AddObservation(pKFini, i);
		pMP->AddObservation(pKFcur, mvIniMatches[i]);
		
		pMP->ComputeDistinctiveDescriptors();
		pMP->UpdateNormalAndDepth();
		
		mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
		mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;
		
		mpMap->AddMapPoint(pMP);
	}
	
	pKFini->UpdateConnections();
	pKFcur->UpdateConnections();
	
	cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;
	
	Optimizer::GlobalBundleAdjustment(mpMap, 20);
	
	float medianDepth = pKFini->ComputeSceneMedianDepth(2);
	float invMedianDepth = 1.0f / medianDepth;
	
	if(medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
	{
		cout << "Wrong initialization, reseting..." << endl;
		Reset();
		return;
	}
	
	
	cv::Mat Tc2w = pKFcur->GetPose();
	Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
	pKFcur->SetPose(Tc2w);
	
	vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
	for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
	{
		if(vpAllMapPoints[iMP])
		{
			MapPoint* pMP = vpAllMapPoints[iMP];
			pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
		}
	}
	
	mpLocalMapper->InsertKeyFrame(pKFini);
	mpLocalMapper->InsertKeyFrame(pKFcur);
	
	mCurrentFrame.SetPose(pKFcur->GetPose());
	mnLastKeyFrameId = mCurrentFrame.mnId;
	mpLastKeyFrame = pKFcur;
	
	mvpLocalKeyFrames.push_back(pKFcur);
	mvpLocalKeyFrames.push_back(pKFini);
	mvpLocalMapPoints = mpMap->GetAllMapPoints();
	mpReferenceKF = pKFcur;
	
	mCurrentFrame.mpReferenceKF = pKFcur;
	mLastFrame = Frame(mCurrentFrame);
	
	mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
	mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose);
	mpMap->mvpKeyFrameOrigins.push_back(pKFini);
	mState = OK;
	
	
	
	
}





void Tracking::InformOnlyTracking(const bool &flag)
{
	mbOnlyTracking = flag;
}


} // namespace ORB_SLAM2
