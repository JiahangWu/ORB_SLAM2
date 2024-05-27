#include "Tracking.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

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
		
		mpFrameDrawer->Update(this);
		
		if(mState != OK)
			return;
	}
	else
	{
		bool bOK;
		
		if(!mbOnlyTracking)
		{
			if(mState == OK)
			{
				CheckReplacedInLastFrame();
				
				if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId + 2)
				{
					bOK = TrackReferenceKeyFrame();
				}
				else
				{
					bOK = TrackWithMotionModel();
					if(!bOK)
						bOK = TrackReferenceKeyFrame();
				}
			}
			else
			{
				bOK = Relocalization();
			}
		}
		
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

void Tracking::CheckReplacedInLastFrame()
{
	for (int i = 0; i < mLastFrame.N; i++)
	{
		MapPoint *pMP = mLastFrame.mvpMapPoints[i];
		if(pMP)
		{
			MapPoint *pRep = pMP->GetReplaced();
			if(pRep)
			{
				mLastFrame.mvpMapPoints[i] = pRep;
			}
		}
	}
}


bool Tracking::TrackReferenceKeyFrame()
{
	mCurrentFrame.ComputeBoW();
	
	ORBmatcher matcher(0.7, true);
	
	vector<MapPoint*> vpMapPointMatches;
	
	int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);
	if(nmatches < 15)
		return false;
		
	mCurrentFrame.mvpMapPoints = vpMapPointMatches;
	mCurrentFrame.SetPose(mLastFrame.mTcw);
	
	Optimizer::PoseOptimization(&mCurrentFrame);
	
	int nmatchesMap = 0;
	for (int i = 0; i < mCurrentFrame.N; i++)
	{
		if(mCurrentFrame.mvpMapPoints[i])
		{
			if(mCurrentFrame.mvbOutlier[i])
			{
				MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
				
				mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
				mCurrentFrame.mvbOutlier[i] = false;
				pMP->mbTrackInView = false;
				pMP->mnLastFrameSeen = mCurrentFrame.mnId;
				nmatches--;
			}
			else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
				nmatchesMap++;
		}
	}
	return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame()
{
	KeyFrame *pRef = mLastFrame.mpReferenceKF;
	cv::Mat Tlr = mlRelativeFramePoses.back();
	
	// l:last, r:reference, w:world
	// Tlw = Tlr*Trw
	mLastFrame.SetPose(Tlr * pRef->GetPose());
	
	if(mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR)
		return;
		
	// Step 2：对于双目或rgbd相机，为上一帧生成新的临时地图点
    // 注意这些地图点只是用来跟踪，不加入到地图中，跟踪完后会删除

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    // Step 2.1：得到上一帧中具有有效深度值的特征点（不一定是地图点）
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);

    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            // vDepthIdx第一个元素是某个点的深度,第二个元素是对应的特征点id
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    // 如果上一帧中没有有效深度的点,那么就直接退出
    if(vDepthIdx.empty())
        return;

    // 按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    // Step 2.2：从中找出不是地图点的部分  
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)      
        {
            // 地图点被创建后就没有被观测，认为不靠谱，也需要重新创建
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            // Step 2.3：需要创建的点，包装为地图点。只是为了提高双目和RGBD的跟踪成功率，并没有添加复杂属性，因为后面会扔掉
            // 反投影到世界坐标系中
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(
                x3D,            // 世界坐标系坐标
                mpMap,          // 跟踪的全局地图
                &mLastFrame,    // 存在这个特征点的帧(上一帧)
                i);             // 特征点id

            // 加入上一帧的地图点中
            mLastFrame.mvpMapPoints[i]=pNewMP; 

            // 标记为临时添加的MapPoint，之后在CreateNewKeyFrame之前会全部删除，并未添加新的观测信息
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            // 因为从近到远排序，记录其中不需要创建地图点的个数
            nPoints++;
        }

        // Step 2.4：如果地图点质量不好，停止创建地图点
        // 停止新增临时地图点必须同时满足以下条件：
        // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
        // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::Relocalization()
{
	mCurrentFrame.ComputeBoW();
	
	vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
	
	if(vpCandidateKFs.empty())
		return false;
	
	const int nKFs = vpCandidateKFs.size();
	
	ORBmatcher matcher(0.75, true);
	
	vector<PnPsolver*> vpPnPsolvers;
	
	
}


bool Tracking::TrackWithMotionModel()
{
	ORBmatcher matcher(0.9, true);
	
	UpdateLastFrame();
	
	mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);
	
	fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
	
	int th;
	if(mSensor != System::STEREO)
		th = 15;
	else
		th = 7;
	
	int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

	if(nmatches < 20)
	{
		fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
		nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*th, mSensor == System::MONOCULAR);
	}
	
	if(nmatches < 20)
		return false;
	
	Optimizer::PoseOptimization(&mCurrentFrame);
	
	int nmatchesMap = 0;
	for (int i = 0; i < mCurrentFrame.N; i++)
	{
		if(mCurrentFrame.mvpMapPoints[i])
		{
			if(mCurrentFrame.mvbOutlier[i])
			{
				MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
				
				mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
				mCurrentFrame.mvbOutlier[i] = false;
				pMP->mbTrackInView = false;
				pMP->mnLastFrameSeen = mCurrentFrame.mnId;
				nmatches--;
			}
			else if(mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
				nmatchesMap++;
		}
	}
	
	if(mbOnlyTracking)
	{
		mbVO = nmatchesMap < 10;
		return nmatches > 20;
	}
	
	return nmatchesMap >= 10;
	
	
	
	
}

void Tracking::InformOnlyTracking(const bool &flag)
{
	mbOnlyTracking = flag;
}


} // namespace ORB_SLAM2
