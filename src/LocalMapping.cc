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

#include "LocalMapping.h"
#include "MOVMatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "GeometricTools.h"
#include <opencv2/core/eigen.hpp>

#include <mutex>
#include <chrono>

static int delta = 5.0;
namespace MOV_SLAM
{

    LocalMapping::LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName) : mpSystem(pSys), mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false), mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), bInitializing(false),
                                                                                                                                 mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
                                                                                                                                 mIdxInit(0), mScale(1.0), mInitSect(0), mbNotBA1(true), mbNotBA2(true), mIdxIteration(0), infoInertial(Eigen::MatrixXd::Zero(9, 9))
    {
        mnMatchesInliers = 0;

        mbBadImu = false;

        mTinit = 0.f;

        mNumLM = 0;
    }

    void LocalMapping::SetTracker(Tracking *pTracker)
    {
        mpTracker = pTracker;
    }

    void LocalMapping::Run()
    {
        mbFinished = false;

        while (1)
        {
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // Check if there are keyframes in the queue
            if (CheckNewKeyFrames() && !mbBadImu)
            {
                // BoW conversion and insertion in Map
                ProcessNewKeyFrame();
                MapPointCulling();

                // Triangulate new MapPoints
                CreateNewMapPoints();

                mbAbortBA = false;

                if (!CheckNewKeyFrames())
                {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    SearchInNeighbors();
                }

                int num_FixedKF_BA = 0;
                int num_OptKF_BA = 0;
                int num_MPs_BA = 0;
                int num_edges_BA = 0;

                if (!CheckNewKeyFrames() && !stopRequested())
                {
                    if (mpAtlas->KeyFramesInMap() > 2)
                    {
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(), num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
                    }
                }

                // mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            }
            else if (Stop() && !mbBadImu)
            {
                // Safe area to stop
                while (isStopped() && !CheckFinish())
                {
                    usleep(1000);
                }
                if (CheckFinish())
                    break;
            }

            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if (CheckFinish())
                break;

            usleep(500);
        }

        SetFinish();
    }

    void LocalMapping::MapPointCulling()
    {
        // Check Recent Added MapPoints
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if (mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;

        int borrar = mlpRecentAddedMapPoints.size();

        while (lit != mlpRecentAddedMapPoints.end())
        {
            MapPoint *pMP = *lit;

            if (pMP->isBad())
                lit = mlpRecentAddedMapPoints.erase(lit);
            else if (pMP->GetFoundRatio() < 0.25f)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 2 && pMP->Observations() <= cnThObs)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            else if (((int)nCurrentKFid - (int)pMP->mnFirstKFid) >= 3)
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
            {
                lit++;
                borrar--;
            }
        }
    }

    void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA = true;
    }

    bool LocalMapping::CheckNewKeyFrames()
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }

    void LocalMapping::ProcessNewKeyFrame()
    {
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for (size_t i = 0; i < vpMapPointMatches.size(); i++)
        {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();

                        // if (mbFarPoints && (pMP->GetMaxDistanceInvariance() >= mThFarPoints || pMP->GetMaxDistanceInvariance() >= mThFarPoints))
                        //{
                        //     pMP->SetBadFlag();
                        // }
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();

        // Insert Keyframe in Map
        mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::EmptyQueue()
    {
        while (CheckNewKeyFrames())
            ProcessNewKeyFrame();
    }

    void LocalMapping::CreateNewMapPoints()
    {
        vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(30);

        if (mbInertial)
        {
            KeyFrame *pKF = mpCurrentKeyFrame;
            int count = 0;
            while ((vpNeighKFs.size() <= 30) && (pKF->mPrevKF) && (count++ < 30))
            {
                vector<KeyFrame *>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
                if (it == vpNeighKFs.end())
                    vpNeighKFs.push_back(pKF->mPrevKF);
                pKF = pKF->mPrevKF;
            }
        }

        Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
        Eigen::Vector3f tcw1 = sophTcw1.translation();
        Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        int countStereo = 0;
        int countStereoGoodProj = 0;
        int countStereoAttempt = 0;
        int totalStereoPts = 0;

        cv::Mat proj1(3, 4, CV_64FC1);

        eigen2cv(eigTcw1, proj1);

        // Search matches with epipolar restriction and triangulate
        int cnt = 0;

        for (size_t i = 0; i < vpNeighKFs.size(); i++)
        {
            KeyFrame *pKF2 = vpNeighKFs[i];

            GeometricCamera *pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

            // Check first that baseline is not too short
            Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
            Eigen::Vector3f vBaseline = Ow2 - Ow1;
            const float baseline = vBaseline.norm();

            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline / medianDepthKF2;

            if (!mbMonocular)
            {
                if (baseline < pKF2->mb)
                    continue;
            }
            else
            {
                if (ratioBaselineDepth < 0.01)
                    continue;
            }

            // Search matches that fullfil epipolar constraint
            vector<pair<size_t, size_t>> vMatchedIndices;
            bool bCoarse = mbInertial && mpTracker->mState == Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();

            MOVMatcher::SearchForTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices, false, bCoarse);

            Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
            Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
            Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
            Eigen::Vector3f tcw2 = sophTcw2.translation();
            Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
            cv::Mat proj2(3, 4, CV_64FC1);
            eigen2cv(eigTcw2, proj2);

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // Triangulate each match
            const int nmatches = vMatchedIndices.size();
            for (int ikp = 0; ikp < nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;

                const int trackId = pKF2->mvVF[idx2].trackId;

                const cv::KeyPoint &kp1 = (mpCurrentKeyFrame->NLeft == -1)    ? mpCurrentKeyFrame->mvKeysUn[idx1]
                                          : (idx1 < mpCurrentKeyFrame->NLeft) ? mpCurrentKeyFrame->mvKeys[idx1]
                                                                              : mpCurrentKeyFrame->mvKeysRight[idx1 - mpCurrentKeyFrame->NLeft];
                const float kp1_ur = mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur >= 0);
                const bool bRight1 = (mpCurrentKeyFrame->NLeft == -1 || idx1 < mpCurrentKeyFrame->NLeft) ? false
                                                                                                         : true;

                const cv::KeyPoint &kp2 = (pKF2->NLeft == -1)    ? pKF2->mvKeysUn[idx2]
                                          : (idx2 < pKF2->NLeft) ? pKF2->mvKeys[idx2]
                                                                 : pKF2->mvKeysRight[idx2 - pKF2->NLeft];

                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur >= 0);
                const bool bRight2 = (pKF2->NLeft == -1 || idx2 < pKF2->NLeft) ? false
                                                                               : true;
                Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
                Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

                Eigen::Vector3f ray1 = Rwc1 * xn1;
                Eigen::Vector3f ray2 = Rwc2 * xn2;
                const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if (bStereo1)
                    cosParallaxStereo1 = cos(2 * atan2(mpCurrentKeyFrame->mb / 2, mpCurrentKeyFrame->mvDepth[idx1]));
                else if (bStereo2)
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mb / 2, pKF2->mvDepth[idx2]));

                if (bStereo1 || bStereo2)
                    totalStereoPts++;

                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                std::vector<cv::Point2f> p1{cv::Point2f(xn1(0), xn1(1))};
                std::vector<cv::Point2f> p2{cv::Point2f(xn2(0), xn2(1))};

                bool bPointStereo = false;
                bool goodProj = false;

                Eigen::Vector3f x3D;

                if (!bStereo1 && !bStereo2) // cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 && (bStereo1 || bStereo2 || (cosParallaxRays < 0.9996 && mbInertial) || (cosParallaxRays < 0.9998 && !mbInertial)))
                {
                    cv::Mat p3d(4, 1, CV_64FC1);
                    // Eigen::Vector3f x3D;
                    cv::triangulatePoints(proj1, proj2, p1, p2, p3d);

                    if (p3d.at<float>(3, 0) == 0)
                    {
                        continue;
                    }

                    x3D = Eigen::Vector3f(p3d.at<float>(0, 0), p3d.at<float>(1, 0), p3d.at<float>(2, 0));
                    x3D = x3D / p3d.at<float>(3, 0);
                    goodProj = true;
                }
                else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
                {
                    countStereoAttempt++;
                    bPointStereo = true;
                    goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
                }
                else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
                {
                    countStereoAttempt++;
                    bPointStereo = true;
                    goodProj = pKF2->UnprojectStereo(idx2, x3D);
                }
                else
                {
                    continue; // No stereo and very low parallax
                }

                if (!goodProj)
                    continue;

                // Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if (z1 <= 0)
                {
                    continue;
                }

                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if (z2 <= 0)
                {
                    continue;
                }

                // Check reprojection error in first keyframe
                const float invz1 = 1.0 / z1;
                const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);

                if (!bStereo1)
                {
                    cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1, y1, z1));
                    float errX1 = uv1.x - kp1.pt.x;
                    float errY1 = uv1.y - kp1.pt.y;

                    if ((errX1 * errX1 + errY1 * errY1) > delta)
                    {
                        continue;
                    }
                }
                else
                {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > delta)
                        continue;
                }

                // Check reprojection error in second keyframe
                const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
                const float invz2 = 1.0 / z2;
                if (!bStereo2)
                {
                    cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2, y2, z2));
                    float errX2 = uv2.x - kp2.pt.x;
                    float errY2 = uv2.y - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > delta) //5.991)
                    {
                        continue;
                    }
                }
                else
                {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > delta) //7.8)
                        continue;
                }

                // Check scale consistency
                Eigen::Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();

                Eigen::Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();

                if (dist1 == 0 || dist2 == 0)
                {
                    continue;
                }

                if (mbFarPoints && (dist1 >= mThFarPoints || dist2 >= mThFarPoints))
                {
                    continue;
                }

                // Triangulation is succesfull
                MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap(), trackId);

                pMP->AddObservation(mpCurrentKeyFrame, idx1);
                pMP->AddObservation(pKF2, idx2);

                pMP->UpdateNormalAndDepth();

                cnt++;
                mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);
                mpAtlas->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }

        //std::cout << "===========================================" << std::endl;
        //std::cout << "Added " << cnt << " MAP POINTS" << std::endl;
        //std::cout << "===========================================" << std::endl;
    }

    void LocalMapping::SearchInNeighbors()
    {
        // Retrieve neighbor keyframes
        int nn = 30;
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        vector<KeyFrame *> vpTargetKFs;
        for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;
            if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        }

        // Add some covisible of covisible
        // Extend to some second neighbors if abort is not requested
        for (int i = 0, imax = vpTargetKFs.size(); i < imax; i++)
        {
            const vector<KeyFrame *> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
            for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end(); vit2 != vend2; vit2++)
            {
                KeyFrame *pKFi2 = *vit2;
                if (pKFi2->isBad() || pKFi2->mnFuseTargetForKF == mpCurrentKeyFrame->mnId || pKFi2->mnId == mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
                pKFi2->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
            }
            if (mbAbortBA)
                break;
        }

        // Extend to temporal neighbors
        if (mbInertial)
        {
            KeyFrame *pKFi = mpCurrentKeyFrame->mPrevKF;
            while (vpTargetKFs.size() < 20 && pKFi)
            {
                if (pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                {
                    pKFi = pKFi->mPrevKF;
                    continue;
                }
                vpTargetKFs.push_back(pKFi);
                pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
                pKFi = pKFi->mPrevKF;
            }
        }

        // Search matches by projection from current KF in target KFs
        vector<MapPoint *> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (vector<KeyFrame *>::iterator vit = vpTargetKFs.begin(), vend = vpTargetKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;

            MOVMatcher::Fuse(pKFi, vpMapPointMatches);
        }

        if (mbAbortBA)
            return;

        // Search matches by projection from target KFs in current KF
        vector<MapPoint *> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size() * vpMapPointMatches.size());

        for (vector<KeyFrame *>::iterator vitKF = vpTargetKFs.begin(), vendKF = vpTargetKFs.end(); vitKF != vendKF; vitKF++)
        {
            KeyFrame *pKFi = *vitKF;

            vector<MapPoint *> vpMapPointsKFi = pKFi->GetMapPointMatches();

            for (vector<MapPoint *>::iterator vitMP = vpMapPointsKFi.begin(), vendMP = vpMapPointsKFi.end(); vitMP != vendMP; vitMP++)
            {
                MapPoint *pMP = *vitMP;
                if (!pMP)
                    continue;
                if (pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

        MOVMatcher::Fuse(mpCurrentKeyFrame, vpFuseCandidates);

        // Update points
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMapPointMatches[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    pMP->UpdateNormalAndDepth();
                    // if (mbFarPoints && (pMP->GetMaxDistanceInvariance() >= mThFarPoints || pMP->GetMaxDistanceInvariance() >= mThFarPoints))
                    //{
                    //     pMP->SetBadFlag();
                    // }
                }
            }
        }

        // Update connections in covisibility graph
        mpCurrentKeyFrame->UpdateConnections();
    }

    void LocalMapping::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

    bool LocalMapping::Stop()
    {
        unique_lock<mutex> lock(mMutexStop);
        if (mbStopRequested && !mbNotStop)
        {
            mbStopped = true;
            cout << "Local Mapping STOP" << endl;
            return true;
        }

        return false;
    }

    bool LocalMapping::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool LocalMapping::stopRequested()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopRequested;
    }

    void LocalMapping::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);
        if (mbFinished)
            return;
        mbStopped = false;
        mbStopRequested = false;
        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
            delete *lit;
        mlNewKeyFrames.clear();

        cout << "Local Mapping RELEASE" << endl;
    }

    bool LocalMapping::AcceptKeyFrames()
    {
        unique_lock<mutex> lock(mMutexAccept);
        return mbAcceptKeyFrames;
    }

    void LocalMapping::SetAcceptKeyFrames(bool flag)
    {
        unique_lock<mutex> lock(mMutexAccept);
        mbAcceptKeyFrames = flag;
    }

    bool LocalMapping::SetNotStop(bool flag)
    {
        unique_lock<mutex> lock(mMutexStop);

        if (flag && mbStopped)
            return false;

        mbNotStop = flag;

        return true;
    }

    void LocalMapping::InterruptBA()
    {
        mbAbortBA = true;
    }

    void LocalMapping::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LM: Map reset recieved" << endl;
            mbResetRequested = true;
        }
        cout << "LM: Map reset, waiting..." << endl;

        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(1000);
        }
        cout << "LM: Map reset, Done!!!" << endl;
    }

    void LocalMapping::RequestResetActiveMap(Map *pMap)
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LM: Active map reset recieved" << endl;
            mbResetRequestedActiveMap = true;
            mpMapToReset = pMap;
        }
        cout << "LM: Active map reset, waiting..." << endl;

        while (1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequestedActiveMap)
                    break;
            }
            usleep(1000);
        }
        cout << "LM: Active map reset, Done!!!" << endl;
    }

    void LocalMapping::ResetIfRequested()
    {
        bool executed_reset = false;
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbResetRequested)
            {
                executed_reset = true;

                cout << "LM: Reseting Atlas in Local Mapping..." << endl;
                mlNewKeyFrames.clear();
                mlpRecentAddedMapPoints.clear();
                mbResetRequested = false;
                mbResetRequestedActiveMap = false;

                // Inertial parameters
                mTinit = 0.f;
                mbNotBA2 = true;
                mbNotBA1 = true;
                mbBadImu = false;

                mIdxInit = 0;

                cout << "LM: End reseting Local Mapping..." << endl;
            }

            if (mbResetRequestedActiveMap)
            {
                executed_reset = true;
                cout << "LM: Reseting current map in Local Mapping..." << endl;
                mlNewKeyFrames.clear();
                mlpRecentAddedMapPoints.clear();

                // Inertial parameters
                mTinit = 0.f;
                mbNotBA2 = true;
                mbNotBA1 = true;
                mbBadImu = false;

                mbResetRequested = false;
                mbResetRequestedActiveMap = false;
                cout << "LM: End reseting Local Mapping..." << endl;
            }
        }
        if (executed_reset)
            cout << "LM: Reset free the mutex" << endl;
    }

    void LocalMapping::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool LocalMapping::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LocalMapping::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    bool LocalMapping::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void LocalMapping::ScaleRefinement()
    {
        // Minimum number of keyframes to compute a solution
        // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
        // unique_lock<mutex> lock0(mMutexImuInit);
        if (mbResetRequested)
            return;

        // Retrieve all keyframes in temporal order
        list<KeyFrame *> lpKF;
        KeyFrame *pKF = mpCurrentKeyFrame;
        while (pKF->mPrevKF)
        {
            lpKF.push_front(pKF);
            pKF = pKF->mPrevKF;
        }
        lpKF.push_front(pKF);
        vector<KeyFrame *> vpKF(lpKF.begin(), lpKF.end());

        while (CheckNewKeyFrames())
        {
            ProcessNewKeyFrame();
            vpKF.push_back(mpCurrentKeyFrame);
            lpKF.push_back(mpCurrentKeyFrame);
        }

        mRwg = Eigen::Matrix3d::Identity();
        mScale = 1.0;

        Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);

        if (mScale < 1e-1) // 1e-1
        {
            cout << "scale too small" << endl;
            bInitializing = false;
            return;
        }

        Sophus::SO3d so3wg(mRwg);
        // Before this line we are not changing the map
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
        {
            (*lit)->SetBadFlag();
            delete *lit;
        }
        mlNewKeyFrames.clear();

        // To perform pose-inertial opt w.r.t. last keyframe
        mpCurrentKeyFrame->GetMap()->IncreaseChangeIndex();

        return;
    }

    bool LocalMapping::IsInitializing()
    {
        return bInitializing;
    }

    double LocalMapping::GetCurrKFTime()
    {

        if (mpCurrentKeyFrame)
        {
            return mpCurrentKeyFrame->mTimeStamp;
        }
        else
            return 0.0;
    }

    KeyFrame *LocalMapping::GetCurrKF()
    {
        return mpCurrentKeyFrame;
    }

} // namespace MOV_SLAM
