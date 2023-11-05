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

#include "TwoViewReconstruction.h"

#include "Converter.h"
#include "GeometricTools.h"

#include <opencv2/core/eigen.hpp>
#include <thread>

using namespace std;
namespace MOV_SLAM
{
    TwoViewReconstruction::TwoViewReconstruction(const Eigen::Matrix3f &k, float sigma, int iterations)
    {
        mK = k;

        mSigma = sigma;
        mSigma2 = sigma * sigma;
        mMaxIterations = iterations;
    }

    bool TwoViewReconstruction::Reconstruct(const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2, const vector<int> &vMatches12,
                                            Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
    {
        float minParallax = 1.0;

        mvKeys1.clear();
        mvKeys2.clear();

        mvKeys1 = vKeys1;
        mvKeys2 = vKeys2;

        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        mvMatches12.clear();
        mvMatches12.reserve(mvKeys2.size());
        mvbMatched1.resize(mvKeys1.size());
        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
        {
            if (vMatches12[i] >= 0)
            {
                mvMatches12.push_back(make_pair(i, vMatches12[i]));
                mvbMatched1[i] = true;
            }
            else
                mvbMatched1[i] = false;
        }

        return Reconstruct(T21, vP3D, vbTriangulated, minParallax, 50);
    }

    bool TwoViewReconstruction::Reconstruct(Sophus::SE3f &T21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
    {
        float parallax;
        Eigen::Matrix3f R1;
        Eigen::Vector3f t;
        std::vector<cv::Point2f> pt1;
        std::vector<cv::Point2f> pt2;
        cv::Mat R_;
        cv::Mat t_;
        cv::Mat mask;

        for (size_t i = 0; i < mvMatches12.size(); i++)
        {
            pt1.push_back(mvKeys1[mvMatches12[i].first].pt);
            pt2.push_back(mvKeys2[mvMatches12[i].second].pt);
        }

        double focal_length = 0.5 * (mK(0, 0) + mK(1, 1));
        cv::Point2d principle_point(mK(0, 2), mK(1, 2));
        cv::Mat K_ = (cv::Mat_<double>(3, 3) << mK(0, 0), mK(0, 1), mK(0, 2), mK(1, 0), mK(1, 1), mK(1, 2), mK(2, 0), mK(2, 1), mK(2, 2));

        cv::Mat E = cv::findEssentialMat(pt1, pt2, focal_length, principle_point, cv::USAC_MAGSAC, 0.999, 1.0, mask);
        int n = cv::countNonZero(mask);
        if(n == 0) return false;
        int minGood = max(static_cast<int>(0.75 * n), minTriangulated);

        int pass = cv::recoverPose(E, pt1, pt2, R_, t_, focal_length, principle_point, mask);

        if (pass < minGood)
        {
            return false;
        }

        cv2eigen(R_, R1);
        cv2eigen(t_, t);

        vector<bool> vbMatchesInliers(mvMatches12.size(), false);
        for (int i = 0; i < mask.rows; i++)
        {
            vbMatchesInliers[i] = (mask.at<uchar>(0, i) == 1);
        }

        CheckRT(R1, t, mvKeys1, mvKeys2, mvMatches12, vbMatchesInliers, mK, vP3D, 4.0 * mSigma2, vbTriangulated, parallax);

        if (parallax > minParallax)
        {
            T21 = Sophus::SE3f(R1, t);
            return true;
        }
        return false;
    }

    int TwoViewReconstruction::CheckRT(const Eigen::Matrix3f &R, const Eigen::Vector3f &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                                       const Eigen::Matrix3f &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
    {
        // Calibration parameters
        const float fx = K(0, 0);
        const float fy = K(1, 1);
        const float cx = K(0, 2);
        const float cy = K(1, 2);

        vbGood = vector<bool>(vKeys1.size(), false);
        vP3D.resize(vKeys1.size());

        vector<float> vCosParallax;
        vCosParallax.reserve(vKeys1.size());

        // Camera 1 Projection Matrix K[I|0]
        Eigen::Matrix<float, 3, 4> P1;
        P1.setZero();
        P1.block<3, 3>(0, 0) = K;

        Eigen::Vector3f O1;
        O1.setZero();

        // Camera 2 Projection Matrix K[R|t]
        Eigen::Matrix<float, 3, 4> P2;
        P2.block<3, 3>(0, 0) = R;
        P2.block<3, 1>(0, 3) = t;
        P2 = K * P2;

        Eigen::Vector3f O2 = -R.transpose() * t;

        cv::Mat proj1(3, 4, CV_64FC1);
        cv::Mat proj2(3, 4, CV_64FC1);

        eigen2cv(P1, proj1);
        eigen2cv(P2, proj2);

        int nGood = 0;

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++)
        {

            if (!vbMatchesInliers[i])
                continue;

            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

            Eigen::Vector3f x_p1(kp1.pt.x, kp1.pt.y, 1);
            Eigen::Vector3f x_p2(kp2.pt.x, kp2.pt.y, 1);

            std::vector<cv::Point2f> p1{cv::Point2f(x_p1(0), x_p1(1))};
            std::vector<cv::Point2f> p2{cv::Point2f(x_p2(0), x_p2(1))};

            cv::Mat p3d(4, 1, CV_64FC1);

            cv::triangulatePoints(proj1, proj2, p1, p2, p3d);

            if (p3d.at<float>(3, 0) == 0)
                continue;

            Eigen::Vector3f p3dC1(p3d.at<float>(0, 0), p3d.at<float>(1, 0), p3d.at<float>(2, 0));
            p3dC1 = p3dC1 / p3d.at<float>(3, 0);

            // Check parallax
            Eigen::Vector3f normal1 = p3dC1 - O1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = p3dC1 - O2;
            float dist2 = normal2.norm();

            float cosParallax = normal1.dot(normal2) / (dist1 * dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if (p3dC1(2) <= 0 && cosParallax < 0.99998)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            Eigen::Vector3f p3dC2 = R * p3dC1 + t;

            if (p3dC2(2) <= 0 && cosParallax < 0.99998)
                continue;

            // Check reprojection error in first image
            float im1x, im1y;
            float invZ1 = 1.0 / p3dC1(2);
            im1x = fx * p3dC1(0) * invZ1 + cx;
            im1y = fy * p3dC1(1) * invZ1 + cy;

            float squareError1 = (im1x - kp1.pt.x) * (im1x - kp1.pt.x) + (im1y - kp1.pt.y) * (im1y - kp1.pt.y);

            if (squareError1 > th2)
                continue;

            // Check reprojection error in second image
            float im2x, im2y;
            float invZ2 = 1.0 / p3dC2(2);
            im2x = fx * p3dC2(0) * invZ2 + cx;
            im2y = fy * p3dC2(1) * invZ2 + cy;

            float squareError2 = (im2x - kp2.pt.x) * (im2x - kp2.pt.x) + (im2y - kp2.pt.y) * (im2y - kp2.pt.y);

            if (squareError2 > th2)
                continue;

            vCosParallax.push_back(cosParallax);
            vP3D[vMatches12[i].first] = cv::Point3f(p3dC1(0), p3dC1(1), p3dC1(2));
            nGood++;

            if (cosParallax < 0.99998)
                vbGood[vMatches12[i].first] = true;
        }

        if (nGood > 0)
        {
            sort(vCosParallax.begin(), vCosParallax.end());

            size_t idx = min(50, int(vCosParallax.size() - 1));
            parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
        }
        else
            parallax = 0;

        return nGood;
    }
} // namespace MOV_SLAM
