
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


bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
							 vector<cv::Point3f> &vP3D, vecrtor<bool> &vbTriangulated)
{
	mvKeys2 = CurrentFrame.mvKeysUn;
	mvMatches12.clear();
	mvMatches12.reserve(mvKeys2.size());
	
	mvbMatched1.resize(mvKeys1.size());
	
	for(size_t i = 0; iend = vMatches12.size(); i < iend; i++)
	{
		if(vMatches12[i] >= 0)
		{
			mvMatches12.push_back(make_pair(i, vMatches12[i]));
			mvbMatched1[i] = true;
		}
		else
			mvbMatched1[i] = false;
	}
	
	const int N = mvMatches12.size();
	vector<size_t> vAllIndices;
	vAllIndices.reserve(N);
	
	vector<size_t> vAvailableIndices;
	for(int i = 0; i < N; i++)
	{
		vAllIndices.push_back(i);
	}
	
	mvSets = vector< vector<size_t> >(mMaxIterations, vector<size_t>(8, 0));
	
	DUtils::Random::SeedRandOnce(0);
	for (int it = 0; it < mMaxIterations; it++)
	{
		vAvailableIndices = vallIndices;
		for (size_t j = 0; j < 8; j++)
		{
			int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size() - 1);
			int idx = vAvailableIndices[randi];
			
			mvSets[it][j] = idx;
			vAvailableIndices[randi] = vAvailableIndices.back();
			vAvailableIndices.pop_back();
		}
	}
	
	vector<bool> vbMatchesInliersH, vbMatchesInliersF;
	float SH, SF;
	cv::Mat H, F;
	
	thread threadH(&Initializer::FindHomography, this, ref(vbMatchesInliersH), ref(SH), ref(H));
	thread threadF(&initializer::FindFundamental, this, ref(vbMatchesInliersF), ref(SF), ref(F));
	threadH.join();
	threadF.join();
	
	float(Rh > 0.4)
		return ReconstructH(vbMatchesInliersH, H, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
	else
		return ReconstructF(vbMatchesInliersF, F, mK, R21, t21, vP3D, vbTriangulated, 1.0, 50);
	
	
}

void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
	const int N = mvMatches12.size();
	
	vector<cv::Point2f> vPn1, vPn2;
	cv::Mat T1, T2;
	Normalize(mvKeys1, vPn1, T1);
	Normalize(mvKeys2, vPn2, T2);
	
	cv::Mat T2inv = T2.inv();
	
	score = 0.0;
	vbMatchesInliers = vector<bool>(N, false);
	
	vector<cv::Point2f> vPn1i(8);
	vector<cv::Point2f> vPn2i(8);
	
	cv::Mat H21i, H12i;
	vector<bool> vbCurrentInliers(N, false);
	float currentScore;
	
	for(int it = 0; it <mMaxIterations; it++)
	{
		for (size_t j = 0; j < 8; j++)
		{
			int idx = mvSets[it][j];
			vPn1i[j] = vPn1[mvMatches12[idx].first];
			vPn2i[j] = vPn2[mvMatches12[idx].second];
		}
		
		cv::Mat Hn = ComputeH21(vPn1i, vPn2i);
		
		H21i = T2inv * Hn * T1;
		H12i = H21i.inv();
		
		currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);
		
		if(currentScore > score)
		{
			H21 = H21i.clone();
			vbMatchesInliers = vbCurrentInliers;
			score = currentScore;
		}
	}
	
	
}

cv::Mat Initializer::ComputeH21(
	const vector<cv::Point2f> &vP1,
	const vector<cv::Point2f> &vP2)
{
	const int N = vP1.size();
	
	cv::Mat A(2*N, 9, CV_32F);
	
	for (int i = 0; i < N; i++)
	{
		const float u1 = vP1[i].x;
		const float v1 = vP1[i].y;
		const float u2 = vP2[i].x;
		const float v2 = vP2[i].y;
		
		A.at<float>(2*i, 0) = 0.0;
		A.at<float>(2*i, 1) = 0.0;
		A.at<float>(2*i, 2) = 0.0;
		A.at<float>(2*i, 3) = -u1;
		A.at<float>(2*i, 4) = -v1;
		A.at<float>(2*i, 5) = -1;
		A.at<float>(2*i, 6) = v2 * u1;
		A.at<float>(2*i, 7) = v2 * v1;
		A.at<float>(2*i, 8) = v2;
		
		A.at<float>(2*i + 1, 0) = u1;
		A.at<float>(2*i + 1, 1) = v1;
		A.at<float>(2*i + 1, 2) = 1;
		A.at<float>(2*i + 1, 3) = 0.0;
		A.at<float>(2*i + 1, 4) = 0.0;
		A.at<float>(2*i + 1, 5) = 0.0;
		A.at<float>(2*i + 1, 6) = -u2 * u1;
		A.at<float>(2*i + 1, 7) = -u2 * v1;
		A.at<float>(2*i + 1, 8) = -u2;

	}
	
	cv::Mat u, w, vt;
	cv::SVDcomp(A, w, u, vt,
				cv::SVD::MODIFY_A |
				cv::SVD::FULL_UV);
	
	return vt.row(8).reshape(0, 3);
	
}

float Initializer::CheckHomography(
	const cv::Mat &H21,
	const cv::Mat &H12,
	vector<bool> &vbMatchesInliers,
	float sigma)
{
	const int N = mvMatches12.size();
	
	const float h11 = H21.at<float>(0, 0);
	const float h12 = H21.at<float>(0, 1);
	const float h13 = H21.at<float>(0, 2);
	const float h21 = H21.at<float>(1, 0);
	const float h22 = H21.at<float>(1, 1);
	const float h23 = H21.at<float>(1, 2);
	const float h31 = H21.at<float>(2, 0);
	const float h32 = H21.at<float>(2, 1);
	const float h33 = H21.at<float>(2, 2);
	
	const float h11inv = H12.at<float>(0, 0);
	const float h12inv = H12.at<float>(0, 1);
	const float h13inv = H12.at<float>(0, 2);
	const float h21inv = H12.at<float>(1, 0);
	const float h22inv = H12.at<float>(1, 1);
	const float h23inv = H12.at<float>(1, 2);
	const float h31inv = H12.at<float>(2, 0);
	const float h32inv = H12.at<float>(2, 1);
	const float h33inv = H12.at<float>(2, 2);
	
	vbMatchesInliers.resize(N);
	
	float score = 0;
	
	const float th = 5.991;
	
	const float invSigmaSquare = 1.0 / (sigma * sigma);
	
	for (int i = 0; i < N; i++)
	{
		bool bIn = true;
		
		const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
		const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
		const float u1 = kp1.pt.x;
		const float v1 = kp1.pt.y;
		const float u2 = kp2.pt.x;
		const float v2 = kp2.pt.y;
		
		const float w2in1inv = 1.0 / (h31inv * u2 + h32inv * v2 + h33inv);
		const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
		const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;
		
		const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);
		const float chiSquare1 = squareDist1 * invSigmaSquare;
		
		if(chiSquare1 > th)
			bIn = false;
		else
			score += th - chiSquare1;
			
		
		const float w1in2inv = 1.0 / (h31 * u1 + h32 * v1 + h33);
		const float u1in2 = (h11 * u1 + h12 * v1 + h13) * w1in2inv;
		const float v1in2 = (h21 * u1 + h22 * v1 + h23) * w1in2inv;
		
		const float squareDist2 = (u2 - u1lin2) * (u2 - u1in2) + (v2 - v1in2);
		const float chiSquare2 = squareDist2 * invSigmaSquare;
		
		if(bIn)
			vbMatchesInliers[i] = true;
		else
			vbMatchesInliers[i] = false;
	}
	
	return score;
}


void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float & score, cv::Mat, &F21)
{
	const int N = mvMatches12.size();
	
	vector<cv::Point2f> vPn1, vPn2;
	cv::Mat T1, T2;
	Normalize(mvKeys1, vPn1, T1);
	Normalize(mvKeys2, vPn2, T2);
	cv::Mat T2t = T2.t();
	
	score = 0.0;
	vbMatchesInliers = vector<bool>(N, false);
	
	vector<cv::Point2f> vPn1i(8);
	vector<cv::Point2f> vPn2i(8);
	
	cv::Mat F21i;
	
	vector<bool> vbCurrentInliers(N, false);
	float currentScore;
	
	for (int it = 0; it < mMaxIterations; it++)
	{
		for (int j = 0; j < 8; j++)
		{
			int idx = mvSets[it][j];
			vPn1i[j] = vPn1[mvMatches12[idx].first];
			vPn2i[j] = vPn2[mvMatches12[idx].second];
		}
		
		cv::Mat Fn = ComputeF21(vPn1i, vPn2i);
		
		F21i = T2t * Fn * T1;
		
		currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);
		
		if(currentScore > score)
		{
			F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
		}
	}
	
	
	
	
}


cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
	const int N = vP1.size();

	//初始化A矩阵
	cv::Mat A(N,9,CV_32F); // N*9维

	// 构造矩阵A，将每个特征点添加到矩阵A中的元素
	for(int i=0; i<N; i++)
	{
		const float u1 = vP1[i].x;
		const float v1 = vP1[i].y;
		const float u2 = vP2[i].x;
		const float v2 = vP2[i].y;

		A.at<float>(i,0) = u2*u1;
		A.at<float>(i,1) = u2*v1;
		A.at<float>(i,2) = u2;
		A.at<float>(i,3) = v2*u1;
		A.at<float>(i,4) = v2*v1;
		A.at<float>(i,5) = v2;
		A.at<float>(i,6) = u1;
		A.at<float>(i,7) = v1;
		A.at<float>(i,8) = 1;
	}

	//存储奇异值分解结果的变量
	cv::Mat u,w,vt;


	// 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
	cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	// 转换成基础矩阵的形式
	cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

	//基础矩阵的秩为2,而我们不敢保证计算得到的这个结果的秩为2,所以需要通过第二次奇异值分解,来强制使其秩为2
	// 对初步得来的基础矩阵进行第2次奇异值分解
	cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	// 秩2约束，强制将第3个奇异值设置为0
	w.at<float>(2)=0; 

	// 重新组合好满足秩约束的基础矩阵，作为最终计算结果返回 
	return  u*cv::Mat::diag(w)*vt;
}


float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
	const int N = mvMatches12.size();

	// Step 1 提取基础矩阵中的元素数据
	const float f11 = F21.at<float>(0,0);
	const float f12 = F21.at<float>(0,1);
	const float f13 = F21.at<float>(0,2);
	const float f21 = F21.at<float>(1,0);
	const float f22 = F21.at<float>(1,1);
	const float f23 = F21.at<float>(1,2);
	const float f31 = F21.at<float>(2,0);
	const float f32 = F21.at<float>(2,1);
	const float f33 = F21.at<float>(2,2);

	// 预分配空间
	vbMatchesInliers.resize(N);

	// 设置评分初始值（因为后面需要进行这个数值的累计）
	float score = 0;

	// 基于卡方检验计算出的阈值
	// 自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
	// ?是因为点到直线距离是一个自由度吗？
	const float th = 3.841;

	// 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
	const float thScore = 5.991;

	// 信息矩阵，或 协方差矩阵的逆矩阵
	const float invSigmaSquare = 1.0/(sigma*sigma);
	
	for(int i=0; i<N; i++)
	{
		//默认为这对特征点是Inliers
		bool bIn = true;

		// Step 2.1 提取参考帧和当前帧之间的特征匹配点对
		const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
		const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

		// 提取出特征点的坐标
		const float u1 = kp1.pt.x;
		const float v1 = kp1.pt.y;
		const float u2 = kp2.pt.x;
		const float v2 = kp2.pt.y;

		// Reprojection error in second image
		// Step 2.2 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
		const float a2 = f11*u1+f12*v1+f13;
		const float b2 = f21*u1+f22*v1+f23;
		const float c2 = f31*u1+f32*v1+f33;
	
		// Step 2.3 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
		const float num2 = a2*u2+b2*v2+c2;
		const float squareDist1 = num2*num2/(a2*a2+b2*b2);
		// 带权重误差
		const float chiSquare1 = squareDist1*invSigmaSquare;
		
		// Step 2.4 误差大于阈值就说明这个点是Outlier 
		// ? 为什么判断阈值用的 th（1自由度），计算得分用的thScore（2自由度）
		// ? 可能是为了和CheckHomography 得分统一？
		if(chiSquare1>th)
			bIn = false;
		else
			// 误差越大，得分越低
			score += thScore - chiSquare1;

		// 计算img2上的点在 img1 上投影得到的极线 l1= p2 * F21 = (a1,b1,c1)
		const float a1 = f11*u2+f21*v2+f31;
		const float b1 = f12*u2+f22*v2+f32;
		const float c1 = f13*u2+f23*v2+f33;

		// 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
		const float num1 = a1*u1+b1*v1+c1;
		const float squareDist2 = num1*num1/(a1*a1+b1*b1);

		// 带权重误差
		const float chiSquare2 = squareDist2*invSigmaSquare;

		// 误差大于阈值就说明这个点是Outlier 
		if(chiSquare2>th)
			bIn = false;
		else
			score += thScore - chiSquare2;
		
		// Step 2.5 保存结果
		if(bIn)
			vbMatchesInliers[i]=true;
		else
			vbMatchesInliers[i]=false;
	}
	//  返回评分
	return score;
}

bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{

    // 目的 ：通过单应矩阵H恢复两帧图像之间的旋转矩阵R和平移向量T
    // 参考 ：Motion and structure from motion in a piecewise plannar environment.
    //        International Journal of Pattern Recognition and Artificial Intelligence, 1988
    // https://www.researchgate.net/publication/243764888_Motion_and_Structure_from_Motion_in_a_Piecewise_Planar_Environment
    
    // 流程:
    //      1. 根据H矩阵的奇异值d'= d2 或者 d' = -d2 分别计算 H 矩阵分解的 8 组解
    //        1.1 讨论 d' > 0 时的 4 组解
    //        1.2 讨论 d' < 0 时的 4 组解
    //      2. 对 8 组解进行验证，并选择产生相机前方最多3D点的解为最优解

    // 统计匹配的特征点对中属于内点(Inlier)或有效点个数
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    // 参考SLAM十四讲第二版p170-p171
    // H = K * (R - t * n / d) * K_inv
    // 其中: K表示内参数矩阵
    //       K_inv 表示内参数矩阵的逆
    //       R 和 t 表示旋转和平移向量
    //       n 表示平面法向量
    // 令 H = K * A * K_inv
    // 则 A = k_inv * H * k

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    // 对矩阵A进行SVD分解
    // A 等待被进行奇异值分解的矩阵
    // w 奇异值矩阵
    // U 奇异值分解左矩阵
    // Vt 奇异值分解右矩阵，注意函数返回的是转置
    // cv::SVD::FULL_UV 全部分解
    // A = U * w * Vt
    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A, w, U, Vt, cv::SVD::FULL_UV);

    // 根据文献eq(8)，计算关联变量
    V=Vt.t();

    // 计算变量s = det(U) * det(V)
    // 因为det(V)==det(Vt), 所以 s = det(U) * det(Vt)
    float s = cv::determinant(U)*cv::determinant(Vt);
    
    // 取得矩阵的各个奇异值
    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    // SVD分解正常情况下特征值di应该是正的，且满足d1>=d2>=d3
    if(d1/d2<1.00001 || d2/d3<1.00001) {
        return false;
    }


    // 在ORBSLAM中没有对奇异值 d1 d2 d3按照论文中描述的关系进行分类讨论, 而是直接进行了计算
    // 定义8中情况下的旋转矩阵、平移向量和空间向量
    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    // Step 1.1 讨论 d' > 0 时的 4 组解
    // 根据论文eq.(12)有
    // x1 = e1 * sqrt((d1 * d1 - d2 * d2) / (d1 * d1 - d3 * d3))
    // x2 = 0
    // x3 = e3 * sqrt((d2 * d2 - d2 * d2) / (d1 * d1 - d3 * d3))
    // 令 aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3))
    //    aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3))
    // 则
    // x1 = e1 * aux1
    // x3 = e3 * aux2

    // 因为 e1,e2,e3 = 1 or -1
    // 所以有x1和x3有四种组合
    // x1 =  {aux1,aux1,-aux1,-aux1}
    // x3 =  {aux3,-aux3,aux3,-aux3}

    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};


    // 根据论文eq.(13)有
    // sin(theta) = e1 * e3 * sqrt(( d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) /(d1 + d3)/d2
    // cos(theta) = (d2* d2 + d1 * d3) / (d1 + d3) / d2 
    // 令  aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2)
    // 则  sin(theta) = e1 * e3 * aux_stheta
    //     cos(theta) = (d2*d2+d1*d3)/((d1+d3)*d2)
    // 因为 e1 e2 e3 = 1 or -1
    // 所以 sin(theta) = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta}
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);
    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};
	
	for (int i = 0; i < 4; i++)
	{
		cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
		Rp.at<float>(0, 0) = ctheta;
		Rp.at<float>(0, 2) = -stheta[i];
		Rp.at<float>(2, 0) = stheta[i];
		Rp.at<float>(2, 2) = ctheta;
		
		cv::Mat R = s * U * Rp *Vt;
		
		vR.push_back(R);
		
		cv::Mat tp(3, 1, CV_32F);
		tp.at<float>(0) = x1[i];
		tp.at<float>(1) = 0;
		tp.at<float>(2) = -x3[i];
		tp *= d1 - d3;
		
		cv::Mat t = U * tp;
		vt.push_back(t / cv::norm(t));
		
		cv::Mat np(3, 1, CV_32F);
		np.at<float>(0) = x1[i];
		np.at<float>(1) = 0;
		np.at<float>(2) = x3[i];
		
		cv::Mat n = V * np;
		if(n.at<float>(2) < 0)
			n = -n;
		vn.push_back(n);
	}
	
	float aux_sphi = sqrt((d1 * d1 - d2 * d2) * (d2 * d2 - d3 * d3)) / ((d1 - d3) * d2);
	float cphi = sqrt(d1 * d3 - d2 * d2) / ((d1 - d3) * d2);
	float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};
	
	for (int i = 0; i < 4; i++)
	{
		cv::Mat Rp = cv::Mat::eye(3, 3, CV_32F);
		Rp.at<float>(0, 0) = cphi;
		Rp.at<float>(0, 2) = sphi[i];
		Rp.at<float>(1, 1) = -1;
		Rp.at<float>(1, 1) = sphi[i];
		Rp.at<float>(2, 2) = -cphi;
		
		cv::Mat R = s * U * Rp *Vt;
		vR.push_back(R);
		
		cv::Mat tp(3, 1, CV_32F);
		tp.at<float>(0)=x1[i];
		tp.at<float>(1)=0;
		tp.at<float>(2)=x3[i];
		tp*=d1+d3;

		// 恢复出原来的t
		cv::Mat t = U*tp;
		// 归一化之后加入到vector中,要提供给上面的平移矩阵都是要进行过归一化的
		vt.push_back(t/cv::norm(t));

		// 构造法向量np
		cv::Mat np(3,1,CV_32F);
		np.at<float>(0)=x1[i];
		np.at<float>(1)=0;
		np.at<float>(2)=x3[i];

		// 恢复出原来的法向量
		cv::Mat n = V*np;
		// 保证法向量指向上方
		if(n.at<float>(2)<0)
			n=-n;
		// 添加到vector中
		vn.push_back(n);
		
	}
	
	// 最好的good点
	int bestGood = 0;
	// 其次最好的good点
	int secondBestGood = 0;    
	// 最好的解的索引，初始值为-1
	int bestSolutionIdx = -1;
	// 最大的视差角
	float bestParallax = -1;
	// 存储最好解对应的，对特征点对进行三角化测量的结果
	vector<cv::Point3f> bestP3D;
	// 最佳解所对应的，那些可以被三角化测量的点的标记
	vector<bool> bestTriangulated;
	
	for (size_t i = 0; i < 8; i++)
	{
		float parallaxi;
		vector<cv::Point3f> vP3D;
		vector<bool> vbTriangulatedi;
		
	}
	
	
	
}



bool Initializer::ReconstructF(vector<bool> & vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
							   cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated,
							   float minParallax, int minTriangulated)
{
	
}



int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
			const vector<Match> &vMatches12, vector<bool> &vbInliers,
			const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
	const float fx = K.at<float>(0, 0);
	const float fy = K.at<float>(1, 1);
	const float cx = K.at<float>(0, 2);
	const float cy = K.at<float>(1, 2);
	
	vbGood = vector<bool>(vKeys1.size(), false);
	vP3D.resize(vKeys1.size());
	vector<float> vCosParallax;
	vCosParallax.reserve(vKeys1.size());
	
	cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
	
	K.copyTo(P1.rowRange(0, 3).colRange(0, 3));
	cv::Mat O1 = cv::Mat::zeros(3, 1, CV_32F);
	
}


void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
	float meanX = 0;
	float meanY = 0;
	
	const int N = vKeys.size();
	vNormalizedPoints.resize(N);
	
	for(int i = 0; i < N; i++)
	{
		meanX += vKeys[i].pt.x;
		meanY += vKeys[i].pt.y;
	}
	
	meanX = meanX/N;
	meanY = meanY/N;
	
	float meanDevX = 0;
	float meanDevY = 0;
	
	for (int i = 0; i < N; i++)
	{
		vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
		vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;
		
		meanDevX += fabs(vNormalizedPoints[i].x);
		meanDevY += fabs(vNormalizedPoints[i].y);
	}
	
	meanDevX = meanDevX/N;
	meanDevY = meanDevY/N;
	
	float sX = 1.0 / meanDevX;
	float sY = 1.0 / meanDevY;
	
	for (int i = 0; i < N; i++)
	{
		vNormalizedPoints[i].x = vNormalizedPoints.x * sX;
		vNormalizedPoints[i].y = vNormalizedPoints.y * sY;
	}
	
	T = cv::Mat::eye(3, 3, CV_32F);
	T.at<float>(0, 0) = sX;
	T.at<float>(1, 1) = sY;
	T.at<float>(0, 2) = -meanX * sX;
	T.at<float>(1, 2) = -meanY * sY;
}

	
} // namespace ORB_SLAM2
