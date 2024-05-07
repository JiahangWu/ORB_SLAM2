
#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"          //不太明白为什么这个头文件的包含在vscode中会报错
#include "ORBmatcher.h"

//这里使用到了多线程的加速技术
#include<thread>

namespace ORB_SLAM2
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
	mK = ReferenceFrame.mK.clone();
	
	mvKeys1 = ReferenceFrame.mvKeysUn;
	
	mSigma = sigma;
	mSigma2 = sigma*sigma;
	
	mMaxIterations = iterations;
}


	
	
} // namespace ORB_SLAM2
