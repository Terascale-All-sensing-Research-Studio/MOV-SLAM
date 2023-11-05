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

#ifndef MOVMATCHER_H
#define MOVMATCHER_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "sophus/sim3.hpp"

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"

namespace MOV_SLAM
{
    class MOVMatcher
    {
    public:
        static int SearchByVideoFeature(Frame &F, const vector<MapPoint *> &vpMapPoints, const bool bFarPoints, const float thFarPoints)
        {
            int nmatches = 0;

            for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++)
            {
                MapPoint *pMP = vpMapPoints[iMP];

                if (bFarPoints && pMP->mTrackDepth > thFarPoints)
                    continue;

                if (pMP->isBad())
                    continue;

                if (pMP->mbTrackInView)
                {
                    if(F.mvVFMap.count(pMP->mTrackId) > 0)
                    {
                        F.mvpMapPoints[F.mvVFMap[pMP->mTrackId]] = pMP;
                        nmatches++;
                    }
                    //for (size_t j = 0; j < F.mvKeysUn.size(); j++)
                    //{
                    //    if (F.mvVF[j].trackId == pMP->mTrackId && F.mvVF[j].age >= 0)
                    //    {
                    //        F.mvpMapPoints[j] = pMP;
                    //        nmatches++;
                    //        break;
                    //    }
                    //}
                }
            }
            return nmatches;
        }

        static int SearchByVideoFeature(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMapPointMatches)
        {
            const vector<MapPoint *> vpMapPointsKF = pKF->GetMapPointMatches();
            vpMapPointMatches = vector<MapPoint *>(F.N, static_cast<MapPoint *>(NULL));

            int nmatches = 0;

            for (size_t i = 0; i < vpMapPointsKF.size(); i++)
            {
                MapPoint *pMP = vpMapPointsKF[i];

                if (pMP)
                {
                    if (pMP->isBad())
                        continue;

                    if(F.mvVFMap.count(pMP->mTrackId) > 0)
                    {
                        vpMapPointMatches[F.mvVFMap[pMP->mTrackId]] = pMP;
                        nmatches++;
                    }
                    //for (size_t j = 0; j < F.mvVF.size(); j++)
                    //{
                    //    if (F.mvVF[j].trackId == pMP->mTrackId && F.mvVF[j].age >= 0)
                    //    {
                    //        vpMapPointMatches[j] = pMP;
                    //        nmatches++;
                    //        break;
                    //    }
                    //}
                }
            }
            return nmatches;
        }

        static int SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
        {
            int nmatches = 0;
            vnMatches12 = vector<int>(F1.mvKeysUn.size(), -1);
            vector<int> vnMatches21(F2.mvKeysUn.size(), -1);

            for (size_t i1 = 0, iend1 = F2.mvKeysUn.size(); i1 < iend1; i1++)
            {
                VideoFeature vf = F2.mvVF[i1];

                if(F1.mvVFMap.count(vf.trackId) > 0)
                {
                    vnMatches12[F1.mvVFMap[vf.trackId]] = i1;
                    nmatches++;
                }
                //for (size_t j = 0; j < F1.mvKeysUn.size(); j++)
                //{
                //    if (F1.mvVF[j].trackId == vf.trackId)
                //    {
                //        vnMatches12[j] = i1;
                //        nmatches++;
                //        break;
                //    }
                //}
            }

            // Update prev matched
            for (size_t i1 = 0, iend1 = vnMatches12.size(); i1 < iend1; i1++)
                if (vnMatches12[i1] >= 0)
                    vbPrevMatched[i1] = F2.mvKeysUn[vnMatches12[i1]].pt;

            return nmatches;
        }

        static int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                          vector<pair<size_t, size_t>> &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
        {
            int nmatches;
            const vector<MapPoint *> mp1 = pKF1->GetMapPointMatches();
            const vector<MapPoint *> mp2 = pKF2->GetMapPointMatches();

            for (size_t i = 0; i < mp1.size(); i++)
            {
                VideoFeature vf1 = pKF1->mvVF[i];

                if (mp1[i])
                    continue;

                for (size_t j = 0; j < mp2.size(); j++)
                {
                    if (mp2[j])
                        continue;
                    VideoFeature vf2 = pKF2->mvVF[j];
                    if (vf1.trackId == vf2.trackId)
                    {
                        nmatches++;
                        vMatchedPairs.push_back(make_pair(i, j));
                        break;
                    }
                }
            }

            return nmatches;
        }

        static int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints)
        {
            GeometricCamera *pCamera;
            Sophus::SE3f Tcw;
            Eigen::Vector3f Ow;

            Tcw = pKF->GetPose();
            Ow = pKF->GetCameraCenter();
            pCamera = pKF->mpCamera;

            int nFused = 0;

            const int nMPs = vpMapPoints.size();

            // For debbuging
            int count_notMP = 0, count_bad = 0, count_isinKF = 0, count_negdepth = 0, count_notinim = 0, count_dist = 0, count_normal = 0;
            for (int i = 0; i < nMPs; i++)
            {
                MapPoint *pMP = vpMapPoints[i];

                if (!pMP)
                {
                    count_notMP++;
                    continue;
                }

                if (pMP->isBad())
                {
                    count_bad++;
                    continue;
                }
                else if (pMP->IsInKeyFrame(pKF))
                {
                    count_isinKF++;
                    continue;
                }

                Eigen::Vector3f p3Dw = pMP->GetWorldPos();
                Eigen::Vector3f p3Dc = Tcw * p3Dw;

                // Depth must be positive
                if (p3Dc(2) < 0.0f)
                {
                    count_negdepth++;
                    continue;
                }

                const Eigen::Vector2f uv = pCamera->project(p3Dc);

                // Point must be inside the image
                if (!pKF->IsInImage(uv(0), uv(1)))
                {
                    count_notinim++;
                    continue;
                }

                const float maxDistance = pMP->GetMaxDistanceInvariance();
                const float minDistance = pMP->GetMinDistanceInvariance();
                Eigen::Vector3f PO = p3Dw - Ow;
                const float dist3D = PO.norm();

                // Depth must be inside the scale pyramid of the image
                if (dist3D < minDistance || dist3D > maxDistance)
                {
                    count_dist++;
                    continue;
                }

                // Viewing angle must be less than 60 deg
                Eigen::Vector3f Pn = pMP->GetNormal();

                if (PO.dot(Pn) < 0.5 * dist3D)
                {
                    count_normal++;
                    continue;
                }

                const std::vector<cv::KeyPoint> kps = pKF->mvKeysUn;

                for (size_t j = 0; j < kps.size(); j++)
                {
                    VideoFeature vf2 = pKF->mvVF[j];
                    if (pMP->mTrackId == vf2.trackId)
                    {
                        MapPoint *pMPinKF = pKF->GetMapPoint(j);
                        if (pMPinKF)
                        {
                            if (!pMPinKF->isBad())
                            {
                                if (pMPinKF->Observations() > pMP->Observations())
                                    pMP->Replace(pMPinKF);
                                else
                                    pMPinKF->Replace(pMP);
                            }
                        }
                        else
                        {
                            pMP->AddObservation(pKF, j);
                            pKF->AddMapPoint(pMP, j);
                        }
                        nFused++;
                        break;
                    }
                }
            }

            return nFused;
        }
    };

} // namespace MOV_SLAM

#endif // MOVMATCHER_H