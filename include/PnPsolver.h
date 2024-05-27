
#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"


namespace ORB_SLAM2
{

class PnPsolver
{
public:
	PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);
	
	~PnPsolver();
	
	void SetRansacParameters(double probability = 0.99, int minInliers = 8, int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
							 float th2 = s.991);
	
	cv::Mat find(vector<bool> &vbInliers, int &nInliers);
	
	cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);
	
private:

	
	
};


	
} // namespace ORB_SLAM2


#endif //PNPSOLVER_H