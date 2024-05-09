#include "System.h"

//包含共有库
#include <thread>					//多线程
#include <pangolin/pangolin.h>		//可视化界面
#include <iomanip>					//主要是对cin,cout之类的一些操纵运算子
#include <unistd.h>
namespace ORB_SLAM2
{
System::System(const string &strVocFile,
			   const string &strSettingsFile,
			   const eSensor sensor,
			   const bool bUseViewer):
					 mSensor(sensor),
					 mpViewer(static_cast<Viewer*>(NULL)),
					 mbReset(false),
					 mbActivateLocalizationMode(false),
					 mbDeactivateLocalizationMode(false)
{
	cout << endl <<
	"ORB-SLAM2 Jiahang Wu" << endl;
	
	cout << "Input sensor was set to ";
	if(mSensor == MONOCULAR)
		cout << "MONOCULAR" << endl;
	else if(mSensor == STEREO)
		cout << "STEREO" << endl;
	else if(mSensor == RGBD)
		cout << "RGBD" << endl;
		
	
	cv::FileStorage fsSettings(strSettingsFile.c_str(),
							   cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
		cerr << "Failed to open settings file at: " << strSettingsFile << endl;
		exit(-1);
	}
	
	cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
	
	mpVocabulary = new ORBVocabulary();
	
	bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	
	if(!bVocLoad)
	{
		cerr << "Wrong path to vocabulary. " << endl;
		cerr << "Falied to open at: " << strVocFile << endl;
		exit(-1);
	}
	
	cout << "Vocabulary loaded!" << endl << endl;
	
	mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
	
	mpMap = new Map();
	
	mpFrameDrawer = new FrameDrawer(mpMap);
	mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
	
	mpTracker = new Tracking(this,
							 mpVocabulary,
							 mpFrameDrawer,
							 mpMapDrawer,
							 mpMap,
							 mpKeyFrameDatabase,
							 strSettingsFile,
							 mSensor);
	
	mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
	
	mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
	
	mpLoopCloser = new LoopClosing(mpMap,
									mpKeyFrameDatabase,
									mpVocabulary,
									mSensor!=MONOCULAR);
		
	mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run,
								mpLoopCloser);
	
	if(bUseViewer)
	{
		mpViewer = new Viewer(this,
							  mpFrameDrawer,
							  mpMapDrawer,
							  mpTracker,
							  strSettingsFile);
		
		mptViewer = new thread(&Viewer::Run, mpViewer);
		
		mpTracker->SetViewer(mpViewer);
	}
	
	mpTracker->SetLocalMapper(mpLocalMapper);
	mpTracker->SetLoopClosing(mpLoopCloser);
	
	mpLocalMapper->SetTracker(mpTracker);
	mpLocalMapper->SetLoopCloser(mpLoopCloser);
	
	mpLoopCloser->SetTracker(mpTracker);
	mpLoopCloser->SetLocalMapper(mpLocalMapper);
	
	
}

void System::ActivateLocalizationMode()
{
	unique_lock<mutex> lock(mMutexMode);
	mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
	unique_lock<mutex> lock(mMutexMode);
	mbDeactivateLocalizationMode = true;
}


cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
	if(mSensor != MONOCULAR)
	{
		cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
	}
	{
		unique_lock<mutex> lock(mMutexMode);
		if(mbActivateLocalizationMode)
		{
			mpLocalMapper->RequestStop();
			while(!mpLocalMapper->isStopped())
			{
				usleep(1000);
			}
			mpTracker->InformOnlyTracking(true);
			mbActivateLocalizationMode = false;
		}
		
		if(mbDeactivateLocalizationMode)
		{
			mpTracker->InformOnlyTracking(false);
			mpLocalMapper->Release();
			mbDeactivateLocalizationMode = false;
		}
	}
	
	{
		unique_lock<mutex> lock(mMutexReset);
		if(mbReset)
		{
			mpTracker->Reset();
			mbReset = false;
		}
	}
	
	cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);
	
	
}

void System::Reset()
{
	unique_lock<mutex> lock(mMutexReset);
	mbReset = true;
}


} // namespace ORB_SLAM2
