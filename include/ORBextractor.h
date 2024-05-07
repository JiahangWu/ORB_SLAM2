#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H


#include <vector>
#include <list>
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
class ORBextractor
{
public:
	enum{HARRIS_SCORE=0, FAST_SCORE=1};
	
	ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST, int minThFast);
	~ORBextractor(){}
	
	void operator()(cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint>& keypoints, cv::OutputArray descriptors);
	
	int inline GetLevels() 
	{
		return nlevels;
	}
	
	float inline GetScaleFactor() {
		return scaleFactor;
	}
	
	std::vector<float> inline GetInverseScaleFactors() {
		return mvInvScaleFactor;
	}
	
	std::vector<float> inline GetScaleSigmaSquares(){
		return mvLevelSigma2;
	}

	/**
	 * @brief 获取上面sigma平方的倒数
	 * @return std::vector<float> 
	 */
	std::vector<float> inline GetInverseScaleSigmaSquares(){
		return mvInvLevelSigma2;
	}

	///这个是用来存储图像金字塔的变量，一个元素存储一层图像
	std::vector<cv::Mat> mvImagePyramid;
	
protected:

	void ComputePyramid(cv::Mat image);
	
	std::vector<cv::Point> pattern;
	
	int nfeatures;
	double scaleFactor;
	int nlevels;
	int iniThFAST;
	int minThFAST;
	
	std::vector<int> mnFeaturesPerLevel;
	std::vector<int> umax;
	
	std::vector<float> mvScaleFactor;
	std::vector<float> mvInvScaleFactor;
	std::vector<float> mvLevelSigma2;
	std::vector<float> mvInvLevelSigma2;

	
};


} // namespace ORB_SLAM2




#endif