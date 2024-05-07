#ifndef FRAME_H
#define FRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{

public:
	Frame();
	
	Frame(const Frame &frame);
	
	// Monocular
	Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &disCoef, const float &bf, const float &thDepth);
	
	
	void ExtractORB(int flag, const cv::Mat &im);
	
	void ComputeBoW();
	
	void SetPose(cv::Mat Tcw);
	
	void UpdatePoseMatrices();
	
	inline cv::Mat GetCameraCenter()
	{
		return mOw.clone();
	}
	
	inline cv::Mat GetRotationInverse()
	{
		return mRwc.clone();
	}
	
	bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
	bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
	vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevev=-1, const int maxLevel=-1);

public:

	ORBVocabulary *mpORBvocabulary;
	ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
	
	double mTimeStamp;
	
	cv::Mat mK;
	
	static float fx;
	static float fy;
	static float cx;
	static float cy;
	static float invfx;
	static float invfy;
	
	cv::Mat mDistCoef;
	
	float mbf;
	float mb;
	
	float mThDepth;
	int N;
	
	
	std::vector<cv::KeyPoint> mvKeys;
	std::vector<cv::KeyPoint> mvKeysRight;
	std::vector<cv::KeyPoint> mvKeysUn;
	
	std::vector<float> mvuRight;
	std::vector<float> mvDepth;
	DBoW2::BowVector mBowVec;
	DBoW2::FeatureVector mFeatVec;
	
	cv::Mat mDescriptors, mDescriptorsRight;
	std::vector<MapPoint*> mvpMapPoints;
	std::vector<bool> mvbOutlier;
	
	static float mfGridElementWidthInv;
	static float mfGridElementHeightInv;
	std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
	
	cv::Mat mTcw;
	
	static long unsigned int nNextId;
	long unsigned int mnId;
	
	KeyFrame* mpReferenceKF;
	
	int mnScaleLevels;
	float mfScaleFactor;
	float mfLogScaleFactor;
	
	vector<float> mvScaleFactors;
	vector<float> mvInvScaleFactors;
	vector<float> mvLevelSigma2;
	vector<float> mvInvLevelSigma2;
	
	
	static float mnMinX;
	static float mnMaxX;
	static float mnMinY;
	static float mnMaxY;
	
private:
	void UndistortKeyPoints();
	
	void ComputeImageBounds(const cv::Mat &imLeft);
	
	void AssignFeaturesToGrid();
	
	cv::Mat mRcw; ///< Rotation from world to camera
	cv::Mat mtcw; ///< Translation from world to camera
	cv::Mat mRwc; ///< Rotation from camera to world
	cv::Mat mOw;  ///< mtwc,Translation from camera to world
	
};

} // namespace ORB_SLAM2






#endif