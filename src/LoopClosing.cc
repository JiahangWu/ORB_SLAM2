#include "LoopClosing.h"

// #include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>

namespace ORB_SLAM2
{

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0)
{
	// 连续性阈值
	mnCovisibilityConsistencyTh = 3;
}

// 设置追踪线程句柄
void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}
// 设置局部建图线程的句柄
void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void LoopClosing::Run()
{
	
}
void LoopClosing::RequestReset()
{
    // 标志置位
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    // 堵塞,直到回环检测线程复位完成
    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
		//usleep(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

    }
}



} // namespace ORB_SLAM2
