#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

#include "Converter.h"

#include<mutex>

namespace ORB_SLAM2
{
	
void Optimizer::GlobalBundleAdjustment(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
{
	vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
	vector<MapPoint*> vpMP = pMap->GetAllMapPoints();
	
	BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
	
}

void Optimizer::BundleAdjustment(const vector<KeyFrame*> &vpKFs, const vector<MapPoint*> &vpMP,
								 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
								 const bool bRobust)
{
	vector<bool> vbNotIncludedMP;
	vbNotIncludedMP.resize(vpMP.size());
	
	g2o::SparseOptimizer optimizer;
	g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
	linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
	
	g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
	optimizer.setAlgorithm(solver)
	
	if(pbStopFlag)
		optimizer.setForceStopFlag(pbStopFlag);
	
	long unsigned int maxKFid = 0;
	
	for (size_t i = 0; i < vpKFs.size; i++)
	{
		KeyFrame *pKF = vpKFs[i];
		if(pKF->isBad())
			continue;
		
		
		g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
		vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
		
		vSE3->SetId(pKF->mnId);
		vSE3->setFixed(pKF->mnId == 0);
		
		optimizer.addVertex(vSE3);
		if(pKF->mnId > maxKFid)
			maxKFid = pKF->mnId;
	}
	
	const float thHuber2D = sqrt(5.99);
	const float thHuber3D = sqrt(7.815);
	
	for (size_t i = 0; i < vpMP.size(); i++)
	{
		MapPoint *pMP = vpMP[i];
		
		if(pMP->isBad())
			continue;
		
		g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
		vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
		
		const int id = pMP->mnId + maxKFid + 1;
		vPoint->setId(id);
		
		vPoint->setMarginalized(true);
		optimizer.addVertex(vPoint);
		
		const map<KeyFrame*, sixe_t> observations = pMP->GetObservations();
		
		int nEdges = 0;
		
		for (map<KeyFrame*, size_t>::const_iterator mit = 0; mit != observations.end(); mit++)
		{
			KeyFrame *pKF = mit->first;
			
			if(pKF->isBad() || pKF->mnId > maxKFid)
				continue;
			
			nEdges++;
			
			const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];
			
			if(pKF->mvuRight[mit->second] < 0)
			{
				Eigen::Matrix<double, 2, 1> obs;
				obs << kpUn.pt.x, kpUn.pt.y;
				
				g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();
				e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
				e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
				e->setMeasurement(obs);
				
				const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
				e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);
				
				if(bRobust)
				{
					g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
					e->setRobustKernel(rk);
					rk->setDelta(thHuber2D);
				}
				
				e->fx = pKF->fx;
				e->fy = pKF->fy;
				e->cx = pKF->cx;
				e->cy = pKF->cy;
				
				optimizer.addEdge(e);
				
			}
			else
			{
				
			}
		}
		
		if(nEdges == 0)
		{
			optimizer.removeVertex(vPoint);
			vbNotIncludedMP[i] = true;
		}
		else
		{
			vbNotIncludedMP[i] = false;
		}
	}
	
	optimizer.initializeOptimization();
	optimizer.optimize(nIertations);
	
	for (size_t i = 0; i < vpKFs.size; i++)
	{
		KeyFrame *pKF = vpKFs[i];
		if(pKF->isBad())
			continue;
		
		g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
		g2o::SE3Quat SE3quat = vSE3->estimate();
		
		if(nLoopKF == 0)
		{
			pKF->SetPose(Converter::toCvMat(SE3quat));
		}
		else
		{
			pKF->mTcwGBA.create(4, 4, CV_32F);
			Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
			pKF->mnBAGlobalForKF = nLoopKF;
		}
	}
	
	for (size_t i = 0; i < vpMP.size(); i++)
	{
		if(vbNotIncludedMP[i])
			continue;
		MapPoint *pMP = vpMP[i];
		
		if(pMP->isBad())
			continue;
		
		g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMO->mnId + maxKFid + 1));
		
		if(nLoopKF == 0)
		{
			pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
			pMP->UpdateNormalAndDepth();
		}
		else
		{
			pMP->mPosGBA.create(3, 1, CV_32F);
			Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
			pMP->mnBAGlobalForKF = nLoopKF;
		}
	}
	
	
	
}



} // namespace ORB_SLAM2
