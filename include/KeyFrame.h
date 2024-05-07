#ifndef KEYFRAME_H
#define KEYFRAME_H


#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include <mutex>

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
	KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);
	
	void SetPose(const cv::Mat &Tcw);
	
	cv::Mat GetPose();
	cv::Mat GetPoseInverse();
	cv::Mat GetCameraCenter();
	cv::Mat GetStereoCenter();
	cv::Mat GetRoration();
	cv::Mat GetTranslation();
	
	void ComputeBoW();
	
	void AddConnection(KeyFrame* pKF, const int &weight);
	
	void EraseConnection(KeyFrame* pKF);
	
	void UpdateConnections();
	
	void UpdateBestCovisibles();
	
	std::set<KeyFrame*> GetConnectedKeyFrames();
	
	std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
	
	std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
	
	std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
	
	int GetWeight(KeyFrame *pKF);
	
	void AddChild(KeyFrame *pKF);
	
	void EraseChild(KeyFrame *pKF);
	
	void ChangeParent(KeyFrame *pKF);
	
	std::set<KeyFrame*> GetChilds();
	
	KeyFrame* GetParent();
	
	bool hasChild(KeyFrame *pKF);
	
	void AddLoopEdge(KeyFrame *pKF);
	
	std::set<KeyFrame*> GetLoopEdges();
	
	void AddMapPoint(MapPoint* pMP, const size_t &idx);
	
	void EraseMapPointMatch(const size_t &idx);
	
	void EraseMapPointMatch(MapPoint* pMP);
	
	void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
	
	std::set<MapPoint*> GetMapPoints();
	
	std::vector<MapPoint*> GetMapPointMatches();
	
	int TrackedMapPoints(const int &minObs);
	
	MapPoint* GetMapPoint(const size_t &idx);
	
	std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
	
	cv::Mat UnprojectStereo(int i);
	
	bool IsInImage(const float &x, const float &y) const;
	
	void SetNotErase();
	
	void SetErase();
	
	void SetBadFlag();
	
	bool isBad();
	
	float ComputeSceneMedianDepth(const int q);
	
	static bool weightComp(int a, int b)
	{
		return a > b;
	}
	
	static bool lId(KeyFrame *pKF1, KeyFrame *pKF2)
	{
		return pKF1->mnId < pKF2->mnId;
	}
	
public:
	
	static long unsigned int nNextId;
	long unsigned int mnId;
	const long unsigned int mnFrameId;
	
	const double mTimeStamp;
	
	const int mnGridCols;
	const int mnGridRows;
	const float mfGridElementWidthInv;
	const float mfGridElementHeightInv;
	
	long unsigned int mnTrackReferenceForFrame;
	long unsigned int mnFuseTargetForKF;
	
	long unsigned int mnBALocalForKF;
	long unsigned int mnBAFixedForKF;
	
	long unsigned int mnLoopQuery;
	int mnLoopWords;
	float mLoopScore;
	long unsigned int mnRelocQuery;
	
	int mnRelocWords;
	float mRelocScore;
	
	
	cv::Mat mTcwGBA;
	cv::Mat mTcwBefGBA;
	long unsigned int mnBAGlobalForKF;
	
	const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
	
	
	const int N;
	
	const std::vector<cv::KeyPoint> mvKeys;
	const std::vector<cv::KeyPoint> mvKeysUn;
	const std::vector<float> mvuRight;
	const std::vector<float> mvDepth;
	const cv::Mat mDescriptors;
	
	DBoW2::BowVector mBowVec;
	
	DBoW2::FeatureVector mFeatVec;
	
	cv::Mat mTcp;
	
	const int mnScaleLevels;
	const float mfScaleFactor;
	const float mfLogScaleFactor;
	const std::vector<float> mvScaleFactors;
	const std::vector<float> mvLevelSigma2;
	const std::vector<float> mvInvLevelSigma2;
	
	const int mnMinX;
	const int mnMinY;
	const int mnMaxX;
	const int mnMaxY;
	const cv::Mat mK;
	
	
protected:

	cv::Mat Tcw;
	cv::Mat Twc;
	cv::Mat Ow;
	
	cv::Mat Cw;
	
	std::vector<MapPoint*> mvpMapPoints;
	KeyFrameDatabase *mpKeyFrameDB;
	ORBVocabulary *mpORBvocabulary;
	std::vector< std::vector <std::vector<size_t> > > mGrid;
	
	
	// Covisibility Graph
	std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
	std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
	std::vector<int> mvOrderedWeights;
	
	// ============ Spanning Tree and Loop Edge =============
	bool mbFirstConnection;
	KeyFrame* mpParent;
	std::set<KeyFrame*> mspChildrens;
	std::set<KeyFrame*> mspLoopEdge;
	
	bool mbNotErase;
	bool mbToBeErased;
	bool mbBad;
	
	float mHalfBaseline;
	
	Map* mpMap;
	
	std::mutex mMutexPose;
	std::mutex mMutexConnections;
	std::mutex mMutexFeatures;
};

} // namespace ORB_SLAM2




#endif