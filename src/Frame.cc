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

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "GeometricCamera.h"
#include "MOVExtractor.h"
#include "EXPRESS.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>

namespace MOV_SLAM
{

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    // For stereo fisheye matching
    cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

    Frame::Frame() : mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mbHasPose(false), mbHasVelocity(false)
    {
    }

    // Copy Constructor
    Frame::Frame(const Frame &frame)
        : mpcpi(frame.mpcpi), mpMOVExtractor(frame.mpMOVExtractor),
          mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mK_(Converter::toMatrix3f(frame.mK)), mDistCoef(frame.mDistCoef.clone()),
          mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
          mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
          mvDepth(frame.mvDepth),
          mDescriptors(frame.mDescriptors),
          mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib), mnCloseMPs(frame.mnCloseMPs),
          mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame), mImuBias(frame.mImuBias),
          mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
          mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
          mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors), mNameFile(frame.mNameFile), mnDataset(frame.mnDataset),
          mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2), mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
          mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
          mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2), Nleft(frame.Nleft), Nright(frame.Nright),
          monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
          mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
          mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl), imgLeft(frame.imgLeft), mLost(frame.mLost),
          mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false), mFT(frame.mFT), mvVF(frame.mvVF), mvVFMap(frame.mvVFMap), mvVFRight(frame.mvVFRight), imageCols(frame.imageCols), imageRows(frame.imageRows)
    {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j] = frame.mGrid[i][j];
                if (frame.Nleft > 0)
                {
                    mGridRight[i][j] = frame.mGridRight[i][j];
                }
            }

        if (frame.mbHasPose)
            SetPose(frame.GetPose());

        if (frame.HasVelocity())
        {
            SetVelocity(frame.GetVelocity());
        }

        mmProjectPoints = frame.mmProjectPoints;
        mmMatchedInImage = frame.mmMatchedInImage;
    }

    Frame::Frame(const double &timeStamp, MOVExtractor *extractor,
                 GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth,
                 const shared_ptr<MotionVectorImage> &smv,
                 Frame *pPrevF, const IMU::Calib &ImuCalib)
        : mpcpi(NULL), mpMOVExtractor(extractor),
          mTimeStamp(timeStamp), mK(static_cast<Pinhole *>(pCamera)->toK()), mK_(static_cast<Pinhole *>(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mLost(false),
          mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false), mFT(smv->ft), imageCols(smv->imGray.cols), imageRows(smv->imGray.rows)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mvScaleFactors.resize(8);
        mvLevelSigma2.resize(8);
        mvScaleFactors[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < 8; i++)
        {
            mvScaleFactors[i] = mvScaleFactors[i - 1] * 1.2;
            mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];
        }

        mvInvScaleFactors.resize(8);
        mvInvLevelSigma2.resize(8);
        for (int i = 0; i < 8; i++)
        {
            mvInvScaleFactors[i] = 1.0f / mvScaleFactors[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        imgLeft = smv->imGray.clone();
        ExtractMOV(smv);

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);
        mnCloseMPs = 0;

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));

        mmProjectPoints.clear(); // = map<long unsigned int, cv::Point2f>(N, static_cast<cv::Point2f>(NULL));
        mmMatchedInImage.clear();

        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(smv->imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = mK.at<float>(0, 0);
            fy = mK.at<float>(1, 1);
            cx = mK.at<float>(0, 2);
            cy = mK.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
            {
                SetVelocity(pPrevF->GetVelocity());
            }
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();
    }

    Frame::Frame(const double &timeStamp, MOVExtractor *extractor,
                 GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth,
                 const shared_ptr<MotionVectorImage> &smv,
                 const shared_ptr<MotionVectorImage> &smvRight,
                 Frame *pPrevF, const IMU::Calib &ImuCalib) : mpcpi(NULL), mpMOVExtractor(extractor),
                                                              mTimeStamp(timeStamp), mK(static_cast<Pinhole *>(pCamera)->toK()), mK_(static_cast<Pinhole *>(pCamera)->toK_()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
                                                              mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL), mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false), mpCamera(pCamera), mLost(false),
                                                              mbHasPose(false), mbHasVelocity(false), mFT(smv->ft), imageCols(smv->imGray.cols), imageRows(smv->imGray.rows), mpCamera2(nullptr)
    {
        imgLeft = smv->imGray.clone();
        imgRight = smvRight->imGray.clone();

        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mvScaleFactors.resize(8);
        mvLevelSigma2.resize(8);
        mvScaleFactors[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < 8; i++)
        {
            mvScaleFactors[i] = mvScaleFactors[i - 1] * 1.2;
            mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];
        }

        mvInvScaleFactors.resize(8);
        mvInvLevelSigma2.resize(8);
        for (int i = 0; i < 8; i++)
        {
            mvInvScaleFactors[i] = 1.0f / mvScaleFactors[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        ExtractMOV(smv);

        N = mvKeys.size();
        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoMatches();

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imgLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = mK.at<float>(0, 0);
            fy = mK.at<float>(1, 1);
            cx = mK.at<float>(0, 2);
            cy = mK.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        if (pPrevF)
        {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        }
        else
        {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        // Set no stereo fisheye information
        Nleft = -1;
        Nright = -1;
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        AssignFeaturesToGrid();
    }

    void Frame::ComputeStereoMatches()
    {
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        std::vector<uchar> status;
        std::vector<float> err;
        vector<cv::Point2f> pts, pts_out;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 20, 0.01);

        for (auto &kp : mvKeys)
        {
            pts.push_back(kp.pt);
        }

        cv::calcOpticalFlowPyrLK(imgLeft, imgRight, pts, pts_out, status, err,
                                 cv::Size(21, 21), 3, criteria, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
        for (size_t i = 0; i < pts_out.size(); i++)
        {
            const cv::KeyPoint &kpL = mvKeys[i];
            const cv::Point2f &ptL = kpL.pt;
            const cv::Point2f &ptR = pts_out[i];

            if (status[i] == 0 || ptR.x < 0 || ptR.y < 0 || ptR.x >= imgRight.cols || ptR.y >= imgRight.rows)
            {
                continue;
            }

            if (abs(ptR.y - ptL.y) > 4)
            {
                continue;
            }

            float bestUr = ptR.x;
            float disparity = ptL.x - bestUr;

            if (disparity >= minD && disparity < maxD)
            {
                if (disparity <= 0)
                {
                    disparity = 0.01;
                    bestUr = ptL.x - 0.01;
                }

                mvDepth[i] = mbf / disparity;
                mvuRight[i] = bestUr;
                float dist = cv::norm(ptL - ptR);
                vDistIdx.push_back(pair<int, int>(dist, i));
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }

    void Frame::AssignFeaturesToGrid()
    {
        // Fill matrix with points
        const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

        int nReserve = 0.5f * N / (nCells);

        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
            {
                mGrid[i][j].reserve(nReserve);
                if (Nleft != -1)
                {
                    mGridRight[i][j].reserve(nReserve);
                }
            }

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                                     : (i < Nleft) ? mvKeys[i]
                                                   : mvKeysRight[i - Nleft];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
            {
                if (Nleft == -1 || i < Nleft)
                    mGrid[nGridPosX][nGridPosY].push_back(i);
                else
                    mGridRight[nGridPosX][nGridPosY].push_back(i - Nleft);
            }
        }
    }

    void Frame::ExtractMOV(const shared_ptr<MotionVectorImage> &smv)
    {
        monoLeft = (*mpMOVExtractor)(smv, mvKeys, mvVF, mvVFMap, mDescriptors, mpPrevFrame);
    }

    bool Frame::isSet() const
    {
        return mbIsSet;
    }

    void Frame::SetPose(const Sophus::SE3<float> &Tcw)
    {
        mTcw = Tcw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

    void Frame::SetNewBias(const IMU::Bias &b)
    {
        mImuBias = b;
        if (mpImuPreintegrated)
            mpImuPreintegrated->SetNewBias(b);
    }

    void Frame::SetVelocity(Eigen::Vector3f Vwb)
    {
        mVw = Vwb;
        mbHasVelocity = true;
    }

    Eigen::Vector3f Frame::GetVelocity() const
    {
        return mVw;
    }

    void Frame::UpdatePoseMatrices()
    {
        Sophus::SE3<float> Twc = mTcw.inverse();
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
    }

    Sophus::SE3f Frame::GetRelativePoseTrl()
    {
        return mTrl;
    }

    Sophus::SE3f Frame::GetRelativePoseTlr()
    {
        return mTlr;
    }

    Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation()
    {
        return mTlr.rotationMatrix();
    }

    Eigen::Vector3f Frame::GetRelativePoseTlr_translation()
    {
        return mTlr.translation();
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        if (Nleft == -1)
        {
            pMP->mbTrackInView = false;
            pMP->mTrackProjX = -1;
            pMP->mTrackProjY = -1;

            // 3D in absolute coordinates
            Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

            // 3D in camera coordinates
            const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
            const float Pc_dist = Pc.norm();

            // Check positive depth
            const float &PcZ = Pc(2);
            const float invz = 1.0f / PcZ;
            if (PcZ < 0.0f)
                return false;

            const Eigen::Vector2f uv = mpCamera->project(Pc);

            if (uv(0) < mnMinX || uv(0) > mnMaxX)
                return false;
            if (uv(1) < mnMinY || uv(1) > mnMaxY)
                return false;

            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);

            // Check distance is in the scale invariance region of the MapPoint
            const float maxDistance = pMP->GetMaxDistanceInvariance();
            const float minDistance = pMP->GetMinDistanceInvariance();
            const Eigen::Vector3f PO = P - mOw;
            const float dist = PO.norm();

            if (dist < minDistance || dist > maxDistance)
                return false;

            // Check viewing angle
            Eigen::Vector3f Pn = pMP->GetNormal();

            const float viewCos = PO.dot(Pn) / dist;

            if (viewCos < viewingCosLimit)
                return false;

            // Predict scale in the image
            const int nPredictedLevel = pMP->PredictScale(dist, this);

            // Data used by the tracking
            pMP->mbTrackInView = true;
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjXR = uv(0) - mbf * invz;

            pMP->mTrackDepth = Pc_dist;

            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;

            return true;
        }
        else
        {
            pMP->mbTrackInView = false;
            pMP->mbTrackInViewR = false;
            pMP->mnTrackScaleLevel = -1;
            pMP->mnTrackScaleLevelR = -1;

            pMP->mbTrackInView = isInFrustumChecks(pMP, viewingCosLimit);
            pMP->mbTrackInViewR = isInFrustumChecks(pMP, viewingCosLimit, true);

            return pMP->mbTrackInView || pMP->mbTrackInViewR;
        }
    }

    bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v)
    {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Vector3f Pc = mRcw * P + mtcw;
        const float &PcX = Pc(0);
        const float &PcY = Pc(1);
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
        {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        float u_distort, v_distort;

        float x = (u - cx) * invfx;
        float y = (v - cy) * invfy;
        float r2 = x * x + y * y;
        float k1 = mDistCoef.at<float>(0);
        float k2 = mDistCoef.at<float>(1);
        float p1 = mDistCoef.at<float>(2);
        float p2 = mDistCoef.at<float>(3);
        float k3 = 0;
        if (mDistCoef.total() == 5)
        {
            k3 = mDistCoef.at<float>(4);
        }

        // Radial distorsion
        float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        u_distort = x_distort * fx + cx;
        v_distort = y_distort * fy + cy;

        u = u_distort;
        v = v_distort;

        kp = cv::Point2f(u, v);

        return true;
    }

    Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw)
    {
        return mRcw * pCw + mtcw;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel, const bool bRight) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0, (int)floor((x - mnMinX - factorX) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + factorX) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0, (int)floor((y - mnMinY - factorY) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + factorY) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[vCell[j]]
                                               : (!bRight)   ? mvKeys[vCell[j]]
                                                             : mvKeysRight[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < factorX && fabs(disty) < factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        // Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::UndistortKeyPoints()
    {
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);

        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mK);
            mat = mat.reshape(1);

            // Undistort corners
            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight)
    {
        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        Eigen::Matrix3f mR;
        Eigen::Vector3f mt, twc;
        if (bRight)
        {
            Eigen::Matrix3f Rrl = mTrl.rotationMatrix();
            Eigen::Vector3f trl = mTrl.translation();
            mR = Rrl * mRcw;
            mt = Rrl * mtcw + trl;
            twc = mRwc * mTlr.translation() + mOw;
        }
        else
        {
            mR = mRcw;
            mt = mtcw;
            twc = mOw;
        }

        // 3D in camera coordinates
        Eigen::Vector3f Pc = mR * P + mt;
        const float Pc_dist = Pc.norm();
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        Eigen::Vector2f uv;
        if (bRight)
            uv = mpCamera2->project(Pc);
        else
            uv = mpCamera->project(Pc);

        if (uv(0) < mnMinX || uv(0) > mnMaxX)
            return false;
        if (uv(1) < mnMinY || uv(1) > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const Eigen::Vector3f PO = P - twc;
        const float dist = PO.norm();

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        Eigen::Vector3f Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        if (bRight)
        {
            pMP->mTrackProjXR = uv(0);
            pMP->mTrackProjYR = uv(1);
            pMP->mnTrackScaleLevelR = nPredictedLevel;
            pMP->mTrackViewCosR = viewCos;
            pMP->mTrackDepthR = Pc_dist;
        }
        else
        {
            pMP->mTrackProjX = uv(0);
            pMP->mTrackProjY = uv(1);
            pMP->mnTrackScaleLevel = nPredictedLevel;
            pMP->mTrackViewCos = viewCos;
            pMP->mTrackDepth = Pc_dist;
        }

        return true;
    }

    Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i)
    {
        return mRwc * mvStereo3Dpoints[i] + mOw;
    }

    bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D)
    {
        const float z = mvDepth[i];
        if (z > 0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Eigen::Vector3f x3Dc(x, y, z);
            x3D = mRwc * x3Dc + mOw;
            return true;
        }
        else
            return false;
    }

} // namespace MOV_SLAM
