#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>

namespace ORB_SLAM2
{
class Tracking;


class FrameDrawer
{
private:
	/* data */
public:
	FrameDrawer(Map* pMap);
	
	void Update(Tracking* pTracker);
	
	cv::Mat DrawFrame();

protected:
	
	void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);
	
	cv::Mat mIm;
	
	int N;
	
	vector<cv::KeyPoint> mvCurrentKeys;
	vector<bool> mvbMap, mvbVO;
	bool mbOnlyTracking;
	int mnTracked, mnTrackedVO;
	vector<cv::KeyPoint> mvIniKeys;
	vector<int> mvIniMatches;
	
	int mState;
	
	Map* mpMap;
	std::mutex mMutex;

};



} // namespace ORB_SLAM2





#endif
