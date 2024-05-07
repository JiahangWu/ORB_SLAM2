#include "System.h"

//包含共有库
#include <thread>					//多线程
#include <pangolin/pangolin.h>		//可视化界面
#include <iomanip>					//主要是对cin,cout之类的一些操纵运算子
#include <unistd.h>
namespace ORB_SLAM2
{
System::System(const string &strVocFile,
			   const string &strSettingFile,
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
		
	
	cv::FileStorage fsSettings(strSettingFile.c_str(),
							   cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
		cerr << "Failed to open settings file at: " << strSettingFile << endl;
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
	mpMapDrawer = new MapDrawer(mpMap, strSettingFile);
	
	mpTracker = new Tracking(this,
							 mpVocabulary,
							 mpFrameDrawer,
							 mpMapDrawer,
							 mpMap,
							 mpKeyFrameDatabase,
							 strSettingFile,
							 mSensor);
	
	mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
	
	mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
	
}





} // namespace ORB_SLAM2
