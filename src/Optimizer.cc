/**
 * This file is part of MOV-SLAM
 *
 * Copyright (C) 2022
 *
 * MOV-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MOV-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with MOV-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "Optimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/slam2d_linear/solver_slam2d_linear.h"
#include "g2o/core/creators.h"
#include "g2o/core/factory.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "G2oTypes.h"
#include "Converter.h"
#include "Tracking.h"

#include <sstream>
#include <mutex>

#include "OptimizableTypes.h"
#include <opencv2/core/eigen.hpp>

static float delta = 5.0;

namespace MOV_SLAM
{
    bool sortByVal(const pair<MapPoint *, int> &a, const pair<MapPoint *, int> &b)
    {
        return (a.second < b.second);
    }

    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }

    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        Map *pMap = vpKFs[0]->GetMap();

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3>(solver_ptr));
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        const int nExpectedSize = (vpKFs.size()) * vpMP.size();

        vector<MOV_SLAM::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<MOV_SLAM::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
        vpEdgesBody.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFBody;
        vpEdgeKFBody.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeBody;
        vpMapPointEdgeBody.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        // Set KeyFrame vertices

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKF->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKF->mnId);
            vSE3->setFixed(pKF->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        const float thHuber2D = sqrt(delta);
        const float thHuber3D = sqrt(delta);

        // Set MapPoint vertices
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            MapPoint *pMP = vpMP[i];
            if (pMP->isBad())
                continue;
            g2o::VertexPointXYZ *vPoint = new g2o::VertexPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            int nEdges = 0;
            // SET EDGES
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                KeyFrame *pKF = mit->first;
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;
                if (optimizer.vertex(id) == NULL || optimizer.vertex(pKF->mnId) == NULL)
                    continue;
                nEdges++;

                const int leftIndex = get<0>(mit->second);

                if (leftIndex != -1 && pKF->mvuRight[get<0>(mit->second)] < 0)
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    MOV_SLAM::EdgeSE3ProjectXYZ *e = new MOV_SLAM::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->pCamera = pKF->mpCamera;

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKF);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else if (leftIndex != -1 && pKF->mvuRight[leftIndex] >= 0) // Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKF->mvKeysUn[leftIndex];

                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKF);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                if (pKF->mpCamera2)
                {
                    int rightIndex = get<1>(mit->second);

                    if (rightIndex != -1 && rightIndex < pKF->mvKeysRight.size())
                    {
                        rightIndex -= pKF->NLeft;

                        Eigen::Matrix<double, 2, 1> obs;
                        cv::KeyPoint kp = pKF->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        MOV_SLAM::EdgeSE3ProjectXYZToBody *e = new MOV_SLAM::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKF->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);

                        Sophus::SE3f Trl = pKF->GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKF->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKF);
                        vpMapPointEdgeBody.push_back(pMP);
                    }
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }
        }

        // Optimize!
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);
        Verbose::PrintMess("BA: End of the optimization", Verbose::VERBOSITY_NORMAL);

        // Recover optimized data
        // Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));

            g2o::SE3Quat SE3quat = vSE3->estimate();
            if (nLoopKF == pMap->GetOriginKF()->mnId)
            {
                pKF->SetPose(Sophus::SE3f(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>()));
            }
            else
            {
                pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(), SE3quat.translation()).cast<float>();
                pKF->mnBAGlobalForKF = nLoopKF;

                Sophus::SE3f mTwc = pKF->GetPoseInverse();
                Sophus::SE3f mTcGBA_c = pKF->mTcwGBA * mTwc;
                Eigen::Vector3f vector_dist = mTcGBA_c.translation();
                double dist = vector_dist.norm();
                if (dist > 1)
                {
                    int numMonoBadPoints = 0, numMonoOptPoints = 0;
                    int numStereoBadPoints = 0, numStereoOptPoints = 0;
                    vector<MapPoint *> vpMonoMPsOpt, vpStereoMPsOpt;

                    for (size_t i2 = 0, iend = vpEdgesMono.size(); i2 < iend; i2++)
                    {
                        MOV_SLAM::EdgeSE3ProjectXYZ *e = vpEdgesMono[i2];
                        MapPoint *pMP = vpMapPointEdgeMono[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 5.991 || !e->isDepthPositive())
                        {
                            numMonoBadPoints++;
                        }
                        else
                        {
                            numMonoOptPoints++;
                            vpMonoMPsOpt.push_back(pMP);
                        }
                    }

                    for (size_t i2 = 0, iend = vpEdgesStereo.size(); i2 < iend; i2++)
                    {
                        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i2];
                        MapPoint *pMP = vpMapPointEdgeStereo[i2];
                        KeyFrame *pKFedge = vpEdgeKFMono[i2];

                        if (pKF != pKFedge)
                        {
                            continue;
                        }

                        if (pMP->isBad())
                            continue;

                        if (e->chi2() > 7.815 || !e->isDepthPositive())
                        {
                            numStereoBadPoints++;
                        }
                        else
                        {
                            numStereoOptPoints++;
                            vpStereoMPsOpt.push_back(pMP);
                        }
                    }
                }
            }
        }

        // Points
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            g2o::VertexPointXYZ *vPoint = static_cast<g2o::VertexPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == pMap->GetOriginKF()->mnId)
            {
                pMP->SetWorldPos(vPoint->estimate().cast<float>());
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA = vPoint->estimate().cast<float>();
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }

    int Optimizer::PoseOptimization(Frame *pFrame, const bool isLost, const int iterationCount, const double reprojectionError, const double reprojectErrorLost, const double confidence, const int algorithm)
    {
        std::vector<cv::Point2d> image_points;
        std::vector<cv::Point3d> model_points;

        std::vector<int> indx;

        for (int i = 0; i < pFrame->mvpMapPoints.size(); i++)
        {
            if (pFrame->mvpMapPoints[i])
            {
                Eigen::Vector3f wp = pFrame->mvpMapPoints[i]->GetWorldPos();
                image_points.push_back(pFrame->mvKeys[i].pt);
                model_points.push_back(cv::Point3d(wp.x(), wp.y(), wp.z()));
                indx.push_back(i);
            }
        }

        if (image_points.size() < 4)
        {
            return 0;
        }

        cv::Mat R, t, R_, inliers, K = pFrame->mpCamera->toK();
        cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

        float repError = reprojectionError;
        if (isLost)
        {
            repError = reprojectErrorLost;
        }

        Eigen::Matrix3f r_ = pFrame->GetRwc();
        Eigen::Vector3f t_ = pFrame->GetOw();

        eigen2cv(r_, R);
        eigen2cv(t_, t);

        cv::Rodrigues(R, R);

        cv::solvePnPRansac(model_points, image_points, K, dist_coeffs, R, t, false, iterationCount, repError, confidence, inliers, algorithm);

        Eigen::Matrix3f R1;
        Eigen::Vector3f t1;

        if (R.cols == 0 || R.rows == 0)
        {
            return 0;
        }

        cv::Rodrigues(R, R_);
        cv2eigen(R_, R1);
        cv2eigen(t, t1);

        pFrame->SetPose(Sophus::SE3<float>(R1, t1));
        pFrame->mvbOutlier = std::vector<bool>(pFrame->N, true);
        for (int i = 0; i < inliers.rows; i++)
        {
            pFrame->mvbOutlier[indx[inliers.at<int>(i)]] = false;
        }

        return inliers.rows;
    }

    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int &num_fixedKF, int &num_OptKF, int &num_MPs, int &num_edges)
    {
        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        lLocalKeyFrames.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;
        Map *pCurrentMap = pKF->GetMap();

        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        num_fixedKF = 0;
        list<MapPoint *> lLocalMapPoints;
        set<MapPoint *> sNumObsMP;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            if (pKFi->mnId == pMap->GetInitKFid())
            {
                num_fixedKF = 1;
            }
            vector<MapPoint *> vpMPs = pKFi->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                    {
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
                    }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        list<KeyFrame *> lFixedCameras;
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            map<KeyFrame *, tuple<int, int>> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, tuple<int, int>>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                        lFixedCameras.push_back(pKFi);
                }
            }
        }
        num_fixedKF = lFixedCameras.size() + num_fixedKF;

        if (num_fixedKF == 0)
        {
            Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
            return;
        }

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<g2o::BlockSolver_6_3>(solver_ptr));

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // DEBUG LBA
        pCurrentMap->msOptKFs.clear();
        pCurrentMap->msFixedKFs.clear();

        // Set Local KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(pKFi->mnId == pMap->GetInitKFid());
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
            // DEBUG LBA
            pCurrentMap->msOptKFs.insert(pKFi->mnId);
        }
        num_OptKF = lLocalKeyFrames.size();

        // Set Fixed KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            Sophus::SE3<float> Tcw = pKFi->GetPose();
            vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
            // DEBUG LBA
            pCurrentMap->msFixedKFs.insert(pKFi->mnId);
        }

        // Set MapPoint vertices
        const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

        vector<MOV_SLAM::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<MOV_SLAM::EdgeSE3ProjectXYZToBody *> vpEdgesBody;
        vpEdgesBody.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFBody;
        vpEdgeKFBody.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeBody;
        vpMapPointEdgeBody.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(delta);
        const float thHuberStereo = sqrt(delta);

        int nPoints = 0;

        int nEdges = 0;

        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexPointXYZ *vPoint = new g2o::VertexPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);
            nPoints++;

            const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObservations();

            // Set edges
            for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                {
                    const int leftIndex = get<0>(mit->second);

                    // Monocular observation
                    if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] < 0)
                    {
                        const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        MOV_SLAM::EdgeSE3ProjectXYZ *e = new MOV_SLAM::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->pCamera = pKFi->mpCamera;

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);

                        nEdges++;
                    }
                    else if (leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)] >= 0) // Stereo observation
                    {
                        const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);

                        nEdges++;
                    }

                    if (pKFi->mpCamera2)
                    {
                        int rightIndex = get<1>(mit->second);

                        if (rightIndex != -1)
                        {
                            rightIndex -= pKFi->NLeft;

                            Eigen::Matrix<double, 2, 1> obs;
                            cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            obs << kp.pt.x, kp.pt.y;

                            MOV_SLAM::EdgeSE3ProjectXYZToBody *e = new MOV_SLAM::EdgeSE3ProjectXYZToBody();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                            e->setMeasurement(obs);
                            const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                            e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            Sophus::SE3f Trl = pKFi->GetRelativePoseTrl();
                            e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                            e->pCamera = pKFi->mpCamera2;

                            optimizer.addEdge(e);
                            vpEdgesBody.push_back(e);
                            vpEdgeKFBody.push_back(pKFi);
                            vpMapPointEdgeBody.push_back(pMP);

                            nEdges++;
                        }
                    }
                }
            }
        }
        num_edges = nEdges;

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        
        optimizer.initializeOptimization();
        optimizer.optimize(10);
      
        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesBody.size() + vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            MOV_SLAM::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > delta || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesBody.size(); i < iend; i++)
        {
            MOV_SLAM::EdgeSE3ProjectXYZToBody *e = vpEdgesBody[i];
            MapPoint *pMP = vpMapPointEdgeBody[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > delta || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFBody[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > delta || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data
        // Keyframes
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKFi->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
            pKFi->SetPose(Tiw);
        }

        // Points
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexPointXYZ *vPoint = static_cast<g2o::VertexPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
            pMP->UpdateNormalAndDepth();
        }

        pMap->IncreaseChangeIndex();
    }

    void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
    {
        int its = 10;
        long unsigned int maxKFid = pMap->GetMaxKFid();
        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(std::unique_ptr<g2o::BlockSolverX::LinearSolverType>(linearSolver));

        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<g2o::BlockSolverX>(solver_ptr));
        optimizer.setAlgorithm(solver);

        // Set KeyFrame vertices (all variables are fixed)
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];
            if (pKFi->mnId > maxKFid)
                continue;
            VertexPose *VP = new VertexPose(pKFi);
            VP->setId(pKFi->mnId);
            VP->setFixed(true);
            optimizer.addVertex(VP);

            VertexVelocity *VV = new VertexVelocity(pKFi);
            VV->setId(maxKFid + 1 + (pKFi->mnId));
            VV->setFixed(true);
            optimizer.addVertex(VV);

            // Vertex of fixed biases
            VertexGyroBias *VG = new VertexGyroBias(vpKFs.front());
            VG->setId(2 * (maxKFid + 1) + (pKFi->mnId));
            VG->setFixed(true);
            optimizer.addVertex(VG);
            VertexAccBias *VA = new VertexAccBias(vpKFs.front());
            VA->setId(3 * (maxKFid + 1) + (pKFi->mnId));
            VA->setFixed(true);
            optimizer.addVertex(VA);
        }

        // Gravity and scale
        VertexGDir *VGDir = new VertexGDir(Rwg);
        VGDir->setId(4 * (maxKFid + 1));
        VGDir->setFixed(false);
        optimizer.addVertex(VGDir);
        VertexScale *VS = new VertexScale(scale);
        VS->setId(4 * (maxKFid + 1) + 1);
        VS->setFixed(false);
        optimizer.addVertex(VS);

        // Graph edges
        int count_edges = 0;
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            if (pKFi->mPrevKF && pKFi->mnId <= maxKFid)
            {
                if (pKFi->isBad() || pKFi->mPrevKF->mnId > maxKFid)
                    continue;

                g2o::HyperGraph::Vertex *VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VV1 = optimizer.vertex((maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VP2 = optimizer.vertex(pKFi->mnId);
                g2o::HyperGraph::Vertex *VV2 = optimizer.vertex((maxKFid + 1) + pKFi->mnId);
                g2o::HyperGraph::Vertex *VG = optimizer.vertex(2 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VA = optimizer.vertex(3 * (maxKFid + 1) + pKFi->mPrevKF->mnId);
                g2o::HyperGraph::Vertex *VGDir = optimizer.vertex(4 * (maxKFid + 1));
                g2o::HyperGraph::Vertex *VS = optimizer.vertex(4 * (maxKFid + 1) + 1);
                if (!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
                {
                    Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ", " + to_string(VG->id()) + ", " + to_string(VA->id()) + ", " + to_string(VP2->id()) + ", " + to_string(VV2->id()) + ", " + to_string(VGDir->id()) + ", " + to_string(VS->id()), Verbose::VERBOSITY_NORMAL);

                    continue;
                }
                count_edges++;
                EdgeInertialGS *ei = new EdgeInertialGS(pKFi->mpImuPreintegrated);
                ei->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP1));
                ei->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV1));
                ei->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VG));
                ei->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VA));
                ei->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VP2));
                ei->setVertex(5, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VV2));
                ei->setVertex(6, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VGDir));
                ei->setVertex(7, dynamic_cast<g2o::OptimizableGraph::Vertex *>(VS));
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                ei->setRobustKernel(rk);
                rk->setDelta(1.f);
                optimizer.addEdge(ei);
            }
        }

        // Compute error for different scales
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        optimizer.computeActiveErrors();
        optimizer.activeRobustChi2();
        optimizer.optimize(its);
        optimizer.computeActiveErrors();
        optimizer.activeRobustChi2();
        // Recover optimized data
        scale = VS->estimate();
        Rwg = VGDir->estimate().Rwg;
    }
} // namespace MOV_SLAM
