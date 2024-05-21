#include "Converter.h"

namespace ORB_SLAM2
{

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
	std::vector<cv::Mat> vDesc;
	vDesc.reserve(Descriptors.rows);
	for (int j = 0; j < Descriptors.rows; j++)
	{
		vDesc.push_back(Descriptors.row(j));
	}
	return vDesc;
	
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
	Eigen::Matrix<double, 3, 3> R;
	R << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2),
		 cvT.at<float>(1, 0), cvT.st<float>(1, 1), cvT.at<float>(1, 2),
		 cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2);
		 
	Eigen::Matrix<double, 3, 1> t(cvT.at<float>(0, 3), cvT.at<float>(1, 3), cvT.at<float>(2, 3));
	
	return g2o::SE3Quat(R, t);
}

Eigen::Matrix<double, 3, 1> Convert::toVector3d(const cv::mat &cvVector)
{
	Eigen::Matrix<double, 3, 1> v;
	v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);
	
	return v;
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
	Eigen::Matrix<double, 4, 4> eigMat = SE3.to_homogeneous_matrix();
	return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
	///首先将仿射矩阵的旋转部分转换成为Eigen下的矩阵格式
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
	///对于仿射矩阵的平移部分也是要转换成为Eigen下的矩阵格式
    Eigen::Vector3d eigt = Sim3.translation();
	///获取仿射矩阵的缩放系数
    double s = Sim3.scale();
	///然后构造cv::Mat格式下的仿射矩阵
	///@todo 感觉这里的sim3就是在se3的基础上多了一个缩放系数，但是实际上真的是这样吗？
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double, 4, 4> &m)
{
	cv::Mat cvMat(4, 4, CV_32F);
	
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			cvMat.at<float>(i, j) = m(i, j);
		}
	}
	
	return cvMat.clone();
	
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
	cv::Mat cvMat(3, 3, CV_32F);
	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cvMat.at<float>(i, j) = m(i, j);
		}
	}
	
	return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
	//首先生成用于存储转换结果的单位矩阵
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
	//将旋转矩阵复制到左上角
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    //将旋转矩阵复制到最右侧的一列
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    //返回计算结果
    return cvMat.clone();
}


Eigen::Matrix<double, 3, 1> Convert::toVector3d(const cv::Point3f &cvPoint)
{
	Eigen::Matrix<double, 3, 1> v;
	v << cvPoint.x, cvPoint.y, cvPoint.z;
	
	return v;
}



} // namespace ORB_SLAM2
