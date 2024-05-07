#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
	mState = Tracking::SYSTEM_NOT_READY;
	mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
	
}

cv::Mat FrameDrawer::DrawFrame()
{
	cv::Mat im;
	vector<cv::KeyPoint> vIniKeys;
	vector<int> vMatches;
	vector<cv::KeyPoint> vCurrentKeys;
	vector<bool> vbVO, vbMap;
	int state;
	
	{
		unique_lock<mutex> lock(mMutex);
		state = mState;
		if(mState == Tracking::SYSTEM_NOT_READY)
			mState = Tracking::NO_IMAGES_YET;
			
		mIm.copyTo(im);
		
		
		if(mState == Tracking::NOT_INITIALIZED)
		{
			vCurrentKeys = mvCurrentKeys;
			vIniKeys = mvIniKeys;
			vMatches = mvIniMatches;
		}
		else if(mState == Tracking::OK)
		{
			vCurrentKeys = mvCurrentKeys;
			vbVO = mvbVO;
			vbMap = mvbMap;
		}
		else if(mState == Tracking::LOST)
		{
			vCurrentKeys = mvCurrentKeys;
		}
	}
	
	if(im.channels() < 3)
		cv::cvtColor(im, im, cv::COLOR_GRAY2BGR);
		
	
	if(state==Tracking::NOT_INITIALIZED)
	{
		for (unsigned int i = 0; i < vMatches.size(); i++)
		{
			if(vMatches[i] >= 0)
			{
				cv::line(im, vIniKeys[i].pt, vCurrentKeys[vMatches[i]].pt,
						 cv::Scalar(0, 255, 0));
			}
		}
	}
	else if(state == Tracking::OK)
	{
		mnTracked = 0;
		mnTrackedVO = 0;
		
		const float r = 5;
		const int n = vCurrentKeys.size();
		for(int i = 0; i < n; i++)
		{
			if(vbVO[i] || vbMap[i])
			{
				cv::Point2f pt1, pt2;
				pt1.x = vCurrentKeys[i].pt.x-r;
				pt1.y = vCurrentKeys[i].pt.y-r;
				pt2.x = vCurrentKeys[i].pt.x+r;
				pt2.y = vCurrentKeys[i].pt.y+r;
				
				if(vbMap[i])
				{
					cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
					cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0, 255, 0), -1);
					mnTracked++;
				}
				else
				{
					cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
					cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(255, 0, 0), -1);
					mnTrackedVO++;
				}
			}
		}
	}
	
	cv::Mat imWithInfo;
	DrawTextInfo(im, state, imWithInfo);
	
	return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
	stringstream s;
	if(nState == Tracking::NO_IMAGES_YET)
		s << " WAITING FOR IMAGE";
	else if(nState == Tracking::NOT_INITIALIZED)
		s << " TRYING TO INITIALIZE ";
	else if(nState==Tracking::OK)
	{
		if(!mbOnlyTracking)
			s << "SLAM MODE |  ";
		else
			s << "LOCLIZATION | ";
		int nKFs = mpMap->KeyFramesInMap();
		int nMPs = mpMap->MapPointsInMap();
		 s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
		//在视觉里程计中匹配到的
		if(mnTrackedVO>0)
			s << ", + VO matches: " << mnTrackedVO;
	}
	else if(nState==Tracking::LOST)
	{
		s << " TRACK LOST. TRYING TO RELOCALIZE ";
	}
	else if(nState==Tracking::SYSTEM_NOT_READY)
	{
		s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
	}
	
	int baseline=0;
	//计算字符串文字所占用的图像区域的大小
	cv::Size textSize = cv::getTextSize(
		s.str(),                    //字符串
		cv::FONT_HERSHEY_PLAIN,     //字体
		1,                          //字体缩放
		1,                          //粗细
		&baseline);                 //基线,相对于最低端的文本点的,y坐标  //? 不是太明白
	//扩展图像
	imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
	im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
	//扩充区域填充黑色背景
	imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
	//并且绘制文字
	cv::putText(
		imText,                         //目标图像
		s.str(),                        //要输出的文字
		cv::Point(5,imText.rows-5),     //输出文字的起始位置
		cv::FONT_HERSHEY_PLAIN,         //字体
		1,                              //缩放
		cv::Scalar(255,255,255),        //颜色,白色
		1,                              //线宽
		8);                             //线型
}


// void FrameDrawer::Update(Tracking *pTracker)
// {
// 	unique_lock<mutex> lock(mMutex);
// 	pTracker->mImGray.copyTo(mIm);
// }

} // namespace ORB_SLAM2
