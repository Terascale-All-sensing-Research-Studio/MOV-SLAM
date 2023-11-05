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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <iostream>

#include "MOVExtractor.h"
#include "KeyFrame.h"
#include "EXPRESS.h"

using namespace cv;
using namespace std;

namespace MOV_SLAM
{
    MOVExtractor::MOVExtractor(int threshold, double coverageThreshold, double relocalizationDistance) : mThreshold(threshold), mCoverageThreshold(coverageThreshold), mRelocalizationDistance(relocalizationDistance)
    {
    }

    static void extract_moves(const cv::Mat &img, std::vector<cv::KeyPoint> &kps, std::vector<std::bitset<256>> &descriptors, const int &threshold)
    {
        for (int y = 8; y < img.rows - 8; y += 16)
        {
            for (int x = 8; x < img.cols - 8; x += 16)
            {
                cv::Rect mb(x - 8, y - 8, 16, 16);
                if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < img.cols && (mb.y + mb.height) < img.rows)
                {
                    cv::Mat mb_img = img(mb);
                    bitset<256> desc;
                    if (compute_express(mb_img, threshold))
                    {
                        compute_descriptor(mb_img, threshold, desc);
                        cv::Point2f pt(x, y);
                        cv::KeyPoint kp = cv::KeyPoint(pt, mb.width);
                        kps.push_back(kp);
                        descriptors.push_back(desc);
                    }
                }
            }
        }
    }

    int MOVExtractor::operator()(const shared_ptr<MotionVectorImage> &_smv,
                                 vector<cv::KeyPoint> &_keypoints, vector<VideoFeature> &_vf, std::map<int, int> &_vfmap, std::vector<bitset<256>> &descriptors, Frame *_prev_frame)
    {
        _keypoints.clear();

        vector<bool> lbFound = std::vector<bool>(_smv->kps.size(), false);
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 20, 0.01);

        if (_smv->imGray.empty())
            return -1;

        int mov_cnt = 0;
        cv::Mat imGrey = _smv->imGray;
        assert(imGrey.type() == CV_8UC1);
        vector<int> vLapping = {0, 1000};

        if (_smv->ft == FrameType::I_FRAME)
        {
            if (_prev_frame && _prev_frame->mvVF.size() > 0)
            {
                std::vector<uchar> status;
                std::vector<float> err;
                vector<cv::Point2f> pts, pts_out;

                for (auto &pvf : _prev_frame->mvVF)
                {
                    pts.push_back(pvf.pt);
                }
                cv::calcOpticalFlowPyrLK(_prev_frame->imgLeft, imGrey, pts, pts_out, status, err,
                                         cv::Size(31, 31), 3, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

                for (size_t i = 0; i < pts_out.size(); i++)
                {
                    const cv::Point2f &ptR = pts_out[i];

                    if (status[i] == 0 || ptR.x < 0 || ptR.y < 0 || ptR.x >= imGrey.cols || ptR.y >= imGrey.rows)
                    {
                        continue;
                    }

                    VideoFeature &pvf = _prev_frame->mvVF[i];
                    cv::KeyPoint kp(ptR, pvf.mb.width);
                    _keypoints.push_back(kp);

                    VideoFeature vf;
                    vf.trackId = pvf.trackId;
                    vf.qIndx = i;
                    vf.dIndx = _keypoints.size() - 1;
                    vf.pt = kp.pt;
                    vf.mb = pvf.mb;
                    vf.age = pvf.age + 1;
                    vf.desc = pvf.desc;

                    _vf.push_back(vf);
                    _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                    descriptors.push_back(vf.desc);
                }
            }
            else
            {
                for (int y = 8; y < imGrey.rows - 8; y += 16)
                {
                    for (int x = 8; x < imGrey.cols - 8; x += 16)
                    {
                        cv::Rect mb(x - 8, y - 8, 16, 16);
                        if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < imGrey.cols && (mb.y + mb.height) < imGrey.rows)
                        {
                            cv::Mat mb_img = imGrey(mb);
                            bitset<256> desc;
                            if (compute_express(mb_img, mThreshold))
                            {
                                compute_descriptor(mb_img, mThreshold, desc);
                                cv::Point2f pt(x, y);

                                cv::KeyPoint kp = cv::KeyPoint(pt, mb.width);
                                _keypoints.push_back(kp);

                                mCurrentId++;

                                VideoFeature vf;
                                vf.trackId = mCurrentId;
                                vf.dIndx = _keypoints.size() - 1;
                                vf.qIndx = -1;
                                vf.pt = kp.pt;
                                vf.mb = mb;
                                vf.age = 0;
                                vf.desc = desc;
                                _vf.push_back(vf);
                                _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                                descriptors.push_back(vf.desc);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            if (_prev_frame->mLost)
            {
                KeyFrame *lpLastKeyFrame = _prev_frame->mpReferenceKF;
                std::vector<uchar> status;
                std::vector<float> err;
                vector<cv::Point2f> pts, pts_out;
                vector<int> trackIds;

                const vector<MapPoint *> vpMapPointsKF = lpLastKeyFrame->GetMapPointMatches();

                for (size_t iMP = 0; iMP < vpMapPointsKF.size(); iMP++)
                {
                    MapPoint *pMP = vpMapPointsKF[iMP];

                    if (!pMP)
                    {
                        continue;
                    }

                    if (!pMP->mbTrackInView)
                    {
                        continue;
                    }

                    if (pMP->isBad())
                    {
                        continue;
                    }

                    pts.push_back(cv::Point2f(pMP->mTrackProjX, pMP->mTrackProjY));
                    trackIds.push_back(pMP->mTrackId);
                }

                if (pts.size() > 0)
                {
                    cv::calcOpticalFlowPyrLK(lpLastKeyFrame->mImage, imGrey, pts, pts_out, status, err,
                                             cv::Size(31, 31), 3, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

                    int bestIndx = -1;
                    cv::Size imageSize = imGrey.size();
                    double thresholdDist = mRelocalizationDistance * sqrt(double(imageSize.height * imageSize.height + imageSize.width * imageSize.width));
                    for (size_t i = 0; i < pts_out.size(); i++)
                    {
                        const cv::Point2f &ptL = pts[i];
                        const cv::Point2f &ptR = pts_out[i];

                        if (status[i] == 0 || ptR.x < 0 || ptR.y < 0 || ptR.x >= imGrey.cols || ptR.y >= imGrey.rows)
                        {
                            continue;
                        }

                        // calculate local distance for each possible match
                        double dist = cv::norm(ptR - ptL);

                        if (dist < thresholdDist)
                        {
                            // Computer descriptor
                            cv::Rect mb(ptR.x - 8, ptR.y - 8, 16, 16);
                            if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < imGrey.cols && (mb.y + mb.height) < imGrey.rows)
                            {
                                cv::Mat mb_img = imGrey(mb);
                                bitset<256> desc;
                                compute_descriptor(mb_img, mThreshold, desc);

                                cv::KeyPoint kp = cv::KeyPoint(ptR, mb.width);
                                _keypoints.push_back(kp);

                                VideoFeature vf;
                                vf.trackId = trackIds[i];
                                vf.qIndx = i;
                                vf.dIndx = _keypoints.size() - 1;
                                vf.pt = ptR;
                                vf.mb = mb;
                                vf.desc = desc;
                                vf.age = 0;
                                _vf.push_back(vf);
                                _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                                descriptors.push_back(vf.desc);
                            }
                        }
                    }
                }
            }

            // Project forward the previous frame keypoints
            std::vector<VideoFeature> covFeat;
            if (_prev_frame->mvVF.size() > 0)
            {
                sort(begin(_prev_frame->mvVF),
                     end(_prev_frame->mvVF),
                     [](VideoFeature &a, VideoFeature &b)
                     { return a.age == b.age ? a.desc.count() > b.desc.count() : a.age > b.age; });

                for (size_t i = 0; i < _prev_frame->mvVF.size(); i++)
                {
                    VideoFeature &pvf = _prev_frame->mvVF[i];

                    if (pvf.coverage)
                    {
                        covFeat.push_back(pvf);
                        continue;
                    }

                    int x = pvf.pt.x, y = pvf.pt.y;
                    if (_smv->mvi.at<cv::Vec4i>(y, x)[0] == -1)
                    {
                        continue;
                    }

                    int indx = _smv->mvi.at<cv::Vec4i>(y, x)[0];

                    if (_smv->mvi.at<cv::Vec4i>(y, x)[1] >= 0)
                    {
                        int bestDesc = 256;
                        for (int j = 0; j < 4; j++)
                        {
                            if (_smv->mvi.at<cv::Vec4i>(y, x)[j] == -1)
                                break;

                            const MotionVector &mv = _smv->mvs[_smv->mvi.at<cv::Vec4i>(y, x)[j]];

                            // Shift to the destination
                            cv::Point2f pt = pvf.pt + mv.pt;
                            cv::Rect mb(pt.x - (pvf.mb.width / 2), pt.y - (pvf.mb.height / 2), pvf.mb.width, pvf.mb.height);

                            if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < imGrey.cols && (mb.y + mb.height) < imGrey.rows)
                            {
                                cv::Mat mb_img = imGrey(mb);
                                bitset<256> desc;
                                compute_descriptor(mb_img, mThreshold, desc);
                                int dist = compute_distance(pvf.desc, desc);
                                if (dist < bestDesc)
                                {
                                    bestDesc = dist;
                                    indx = _smv->mvi.at<cv::Vec4i>(y, x)[j];
                                }
                            }
                        }
                    }

                    const MotionVector &mv = _smv->mvs[indx];

                    cv::Point2f pt = pvf.pt + mv.pt;
                    cv::Rect mb(pt.x - (pvf.mb.width / 2), pt.y - (pvf.mb.height / 2), pvf.mb.width, pvf.mb.height);

                    if ((mv.dIndx == -1 || !lbFound[mv.dIndx]) && mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < _prev_frame->imageCols && (mb.y + mb.height) < _prev_frame->imageRows)
                    {
                        if (mv.dIndx >= 0)
                            lbFound[mv.dIndx] = true;

                        cv::Mat mb_img = imGrey(mb);
                        bitset<256> desc;
                        compute_descriptor(mb_img, mThreshold, desc);
                        int dist = compute_distance(pvf.desc, desc);

                        if (dist <= 40)
                        {
                            cv::KeyPoint kp(pt, mb.width);
                            _keypoints.push_back(kp);

                            VideoFeature vf;
                            vf.trackId = pvf.trackId;
                            vf.qIndx = i;
                            vf.dIndx = _keypoints.size() - 1;
                            vf.pt = kp.pt;
                            vf.mb = mb;
                            vf.age = pvf.age + 1;
                            vf.desc = desc; // pvf.desc; // desc
                            _vf.push_back(vf);
                            _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                            descriptors.push_back(vf.desc);
                        }
                    }
                }
            }

            if (covFeat.size() > 0)
            {
                std::vector<uchar> status;
                std::vector<float> err;
                vector<cv::Point2f> pts, pts_out;

                for (auto &pvf : covFeat)
                {
                    pts.push_back(pvf.pt);
                }
                cv::calcOpticalFlowPyrLK(_prev_frame->imgLeft, imGrey, pts, pts_out, status, err,
                                         cv::Size(31, 31), 3, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

                for (size_t i = 0; i < pts_out.size(); i++)
                {
                    const cv::Point2f &ptR = pts_out[i];

                    if (status[i] == 0 || ptR.x < 0 || ptR.y < 0 || ptR.x >= imGrey.cols || ptR.y >= imGrey.rows)
                    {
                        continue;
                    }

                    VideoFeature &pvf = covFeat[i];
                    cv::KeyPoint kp(ptR, pvf.mb.width);
                    _keypoints.push_back(kp);

                    VideoFeature vf;
                    vf.trackId = pvf.trackId;
                    vf.qIndx = i;
                    vf.dIndx = _keypoints.size() - 1;
                    vf.pt = kp.pt;
                    vf.mb = pvf.mb;
                    vf.age = pvf.age + 1;
                    vf.coverage = true;
                    vf.desc = pvf.desc;

                    _vf.push_back(vf);
                    _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                    descriptors.push_back(vf.desc);
                }
            }

            double duration = 0.0;
            for (size_t i = 0; i < _smv->kps.size(); i++)
            {
                if (lbFound[i])
                    continue;

                cv::Rect mb = _smv->kps[i];
                cv::Point2f pt = (mb.br() + mb.tl()) * 0.5;

                if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < _prev_frame->imageCols && (mb.y + mb.height) < _prev_frame->imageRows)
                {
                    cv::Mat mb_img = imGrey(mb);
                    if (compute_express(mb_img, mThreshold))
                    {
                        bitset<256> desc;

                        compute_descriptor(mb_img, mThreshold, desc);

                        cv::KeyPoint kp = cv::KeyPoint(pt, mb.width);
                        _keypoints.push_back(kp);

                        mCurrentId++;

                        VideoFeature vf;
                        vf.trackId = mCurrentId;
                        vf.dIndx = _keypoints.size() - 1;
                        vf.qIndx = -1;
                        vf.pt = kp.pt;
                        vf.mb = mb;
                        vf.age = 0;
                        vf.desc = desc;
                        _vf.push_back(vf);
                        _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                        descriptors.push_back(vf.desc);
                        mov_cnt++;
                    }
                }
            }

            if (_smv->coverageArea < mCoverageThreshold || mov_cnt < 60)
            {
                std::vector<cv::KeyPoint> mkp;
                std::vector<std::bitset<256>> descriptors;
                extract_moves(imGrey, mkp, descriptors, mThreshold);
                for (size_t i = 0; i < mkp.size(); i++)
                {
                    auto &kp = mkp[i];

                    cv::Rect mb = cv::Rect(kp.pt.x - 8, kp.pt.y - 8, 16, 16);

                    if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < _prev_frame->imageCols && (mb.y + mb.height) < _prev_frame->imageRows)
                    {
                        if (_smv->mvi.at<cv::Vec4i>(kp.pt.y, kp.pt.x)[0] >= 0)
                            continue;

                        auto &kp = mkp[i];
                        _keypoints.push_back(kp);

                        mCurrentId++;

                        VideoFeature vf;
                        vf.trackId = mCurrentId;
                        vf.qIndx = -1;
                        vf.dIndx = _keypoints.size() - 1;
                        vf.pt = kp.pt;
                        vf.mb = mb;
                        vf.desc = descriptors[i];
                        vf.coverage = true;
                        _vf.push_back(vf);
                        _vfmap.insert(std::pair<int, int>(vf.trackId, _vf.size() - 1));
                    }
                }
            }
        }

        return _keypoints.size();
    }
}
