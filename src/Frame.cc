#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

Frame::Frame(){}

Frame::Frame(const Frame &frame)
	:mpORBvocabulary(frame.mpORBvocabulary),
	 mpORBextractorLeft(frame.mpORBextractorLeft),
	 mpORBextractorRight(frame.mpORBextractorRight),
	 mTimeStamp(frame.mTimeStamp),
	 mK(frame.mK.clone()),
	 mDistCoef(frame.mDistCoef.clone()),
	 mbf(frame.mbf),
	 mb(frame.mb),
	 mThDepth(frame.mThDepth),
	 N(frame.N),
	 mvKeys(frame.mvKeys),
	 mvKeysRight(frame.mvKeysRight),
	 mvKeysUn(frame.mvKeysUn),
	 mvuRight(frame.mvuRight),
	 mvDepth(frame.mvDepth),
	 mBowVec(frame.mBowVec),
	 mFeatVec(frame.mFeatVec),
	 mDescriptors(frame.mDescriptors.clone()),
	 mDescriptorsRight(frame.mDescriptorsRight.clone()),
	 mvpMapPoints(frame.mvpMapPoints),
	 mvbOutlier(frame.mvbOutlier),
	 mnId(frame.mnId),
	 mpReferenceKF(frame.mpReferenceKF),
	 mnScaleLevels(frame.mnScaleLevels),
	 mnScaleLevels(frame.mnScaleLevels),
	 mfScaleFactor(frame.mfScaleFactor),
	 mfLogScaleFactor(frame.mfLogScaleFactor),
	 mvInvScaleFactors(frame.mvInvScaleFactors),
	 mvLevelSigma2(frame.mvLevelSigma2),
	 mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
	for(int i = 0; i < FRAME_GRID_COLS; i++)
		for (int j = 0; j < FRAME_GRID_ROWS; j++)
			mGrid[i][j] = frame.mGrid[i][j];
	
	if(!frame.mTcw.empty())
		SetPose(frame.mTcw);
		
		
}

Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth):
	mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
	mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
	mnId = nNextId++;
	
	mnScaleLevels = mpORBextractorLeft->GetLevels();
	mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
	mfLogScaleFactor = log(mfScaleFactor);
	mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
	mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
	mvLevelSigma2 = mpORBextractorLeft->GetInverseScaleFactors();
	mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
	
	ExtractORB(0, imGray);
	
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
	if(flag == 0)
		(*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
	else
		(mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
}


void Frame::SetPose(cv::Mat Tcw)
{
	mTcw = Tcw.clone();
	UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    // mOw：    当前相机光心在世界坐标系下坐标
    // mTcw：   世界坐标系到相机坐标系的变换矩阵
    // mRcw：   世界坐标系到相机坐标系的旋转矩阵
    // mtcw：   世界坐标系到相机坐标系的平移向量
    // mRwc：   相机坐标系到世界坐标系的旋转矩阵

	//从变换矩阵中提取出旋转矩阵
    //注意，rowRange这个只取到范围的左边界，而不取右边界
    mRcw = mTcw.rowRange(0,3).colRange(0,3);

    // mRcw求逆即可
    mRwc = mRcw.t();

    // 从变换矩阵中提取出旋转矩阵
    mtcw = mTcw.rowRange(0,3).col(3);

    // mTcw 求逆后是当前相机坐标系变换到世界坐标系下，对应的光心变换到世界坐标系下就是 mTcw的逆 中对应的平移向量
    mOw = -mRcw.t()*mtcw;
}


} // namespace ORB_SLAM2
