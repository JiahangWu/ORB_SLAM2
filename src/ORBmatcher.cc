#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include<stdint.h>

using namespace std;

namespace ORB_SLAM2
{
const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;
	

ORBmatcher::ORBmatcher(float nnratio, bool checkOri):mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
	
}


	
} // namespace ORB_SLAM2
