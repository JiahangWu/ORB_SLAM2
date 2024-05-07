#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
private:
	/* data */
public:
	Map();
	
	void AddKeyFrame(KeyFrame *pKF);
	
	void AddMapPoint(MapPoint *pMP);
	
	void EraseMapPoint(MapPoint *pMP);
	
	void EraseKeyFrame(KeyFrame* pKF);
	
	void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
	
	void InformNewBigChange();
	
	int GetLastBigChangeIdx();
	
	std::vector<KeyFrame*> GetAllKeyFrames();
	
	std::vector<MapPoint*> GetAllMapPoints();
	
	std::vector<MapPoint*> GetReferenceMapPoints();
	
	long unsigned int MapPointsInMap();
	
	long unsigned int KeyFramesInMap();
	
	long unsigned int GetMaxKFid();
	
	void clear();
	
	vector<KeyFrame*> mvpKeyFrameOrigins;
	
	std::mutex mMutexMapUpdate;
	
	std::mutex mMutexPointCreation;



protected:
	std::set<MapPoint*> mspMapPoints;
	
	std::set<KeyFrame*> mspKeyFrames;
	
	std::vector<MapPoint*> mvpReferenceMapPoints;
	
	long unsigned int mnMaxKFid;
	
	int mnBigChangeIdx;
	
	std::mutex mMutexMap;

};

} // namespace ORB_SLAM2






#endif //MAP_H