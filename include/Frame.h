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

#ifndef FRAME_H
#define FRAME_H

#include <vector>

#include "sophus/geometry.hpp"

#include "ImuTypes.h"

#include "Converter.h"
#include "Settings.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include "sophus/se3.hpp"

using namespace std;

namespace MOV_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;
    class ConstraintPoseImu;
    class GeometricCamera;
    class MOVExtractor;

    enum class FrameType
    {
        I_FRAME,
        P_FRAME
    };

    struct MotionVector
    {
    public:
        int indx = -1;
        bool occupied = false;
        cv::Point2f pt;
        int dIndx = -1;
        cv::Rect mb;

        MotionVector()
        {
        }

        // Copy constructor
        MotionVector(const MotionVector &mv)
        {
            indx = mv.indx;
            occupied = mv.occupied;
            pt = mv.pt;
            dIndx = mv.dIndx;
            mb = mv.mb;
        }
    };

    struct VideoFeature
    {
    public:
        int trackId = -1;
        int qIndx = -1;
        int dIndx = -1;
        cv::Point2f pt;
        cv::Rect mb;
        int age = 0;
        bitset<256> desc;
        bool coverage = false;

        VideoFeature()
        {
        }

        // Copy constructor
        VideoFeature(const VideoFeature &vf)
        {
            trackId = vf.trackId;
            qIndx = vf.qIndx;
            dIndx = vf.dIndx;
            pt = vf.pt;
            mb = vf.mb;
            age = vf.age;
            desc = vf.desc;
            coverage = vf.coverage;
        }
    };

    struct VideoImage
    {
    public:
        cv::Mat imGray;
        cv::Mat imRGB;
        cv::Mat mvi;
        std::vector<cv::Rect> kps;
        std::vector<MotionVector> mvs;
        FrameType ft;
        double coverageArea;
        int frame;

        VideoImage(int width, int height)
        {
            mvi = cv::Mat(height,width,CV_32SC4, cv::Scalar(-1,-1,-1,-1));
            kps.reserve(3000);
            mvs.reserve(3000);
        }

        VideoImage(const VideoImage &vi)
        {
            mvi = vi.mvi.clone();
            kps = vi.kps;
            mvs = vi.mvs;
            ft = vi.ft;
            coverageArea = vi.coverageArea;
            frame = vi.frame;
            imGray = vi.imGray;
            imRGB = vi.imRGB;
        }

        ~VideoImage()
        {
            mvi.release();
        }

        inline void clear()
        {
            mvi.setTo(cv::Scalar(-1,-1,-1,-1));
            kps.clear();
            mvs.clear();
            coverageArea = 0.0;
        }
    };

    // typedef std::vector<std::vector<MotionVector>> MotionVectorImage;
    typedef std::pair<cv::KeyPoint, cv::Mat> KeyPointPair;
    typedef VideoImage MotionVectorImage;

    class Frame
    {
    public:
        Frame();

        // Copy constructor.
        Frame(const Frame &frame);

        // Constructor for Monocular Video cameras
        Frame(const double &timeStamp, MOVExtractor *extractor,
              GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth,
              const shared_ptr<MotionVectorImage> &smv,
              Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        Frame(const double &timeStamp, MOVExtractor *extractor,
              GeometricCamera *pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth,
              const shared_ptr<MotionVectorImage> &smv,
              const shared_ptr<MotionVectorImage> &smvRight,
              Frame *pPrevF = static_cast<Frame *>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

        // Destructor
        // ~Frame();

        void ExtractMOV(const shared_ptr<MotionVectorImage> &smv);

        // Set the camera pose. (Imu pose is not modified!)
        void SetPose(const Sophus::SE3<float> &Tcw);

        // Set IMU velocity
        void SetVelocity(Eigen::Vector3f Vw);

        Eigen::Vector3f GetVelocity() const;

        Sophus::SE3f GetRelativePoseTrl();
        Sophus::SE3f GetRelativePoseTlr();
        Eigen::Matrix3f GetRelativePoseTlr_rotation();
        Eigen::Vector3f GetRelativePoseTlr_translation();

        void SetNewBias(const IMU::Bias &b);

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        bool ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v);

        Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1, const bool bRight = false) const;

        ConstraintPoseImu *mpcpi;

        bool isSet() const;

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();

        // Returns the camera center.
        inline Eigen::Vector3f GetCameraCenter()
        {
            return mOw;
        }

        // Returns inverse of rotation
        inline Eigen::Matrix3f GetRotationInverse()
        {
            return mRwc;
        }

        inline Sophus::SE3<float> GetPose() const
        {
            // TODO: can the Frame pose be accsessed from several threads? should this be protected somehow?
            return mTcw;
        }

        inline Eigen::Matrix3f GetRwc() const
        {
            return mRwc;
        }

        inline Eigen::Vector3f GetOw() const
        {
            return mOw;
        }

        inline bool HasPose() const
        {
            return mbHasPose;
        }

        inline bool HasVelocity() const
        {
            return mbHasVelocity;
        }

    private:
        // Sophus/Eigen migration
        Sophus::SE3<float> mTcw;
        Eigen::Matrix<float, 3, 3> mRwc;
        Eigen::Matrix<float, 3, 1> mOw;
        Eigen::Matrix<float, 3, 3> mRcw;
        Eigen::Matrix<float, 3, 1> mtcw;
        bool mbHasPose;

        FrameType mFT;

        // Rcw_ not necessary as Sophus has a method for extracting the rotation matrix: Tcw_.rotationMatrix()
        // tcw_ not necessary as Sophus has a method for extracting the translation vector: Tcw_.translation()
        // Twc_ not necessary as Sophus has a method for easily computing the inverse pose: Tcw_.inverse()

        Sophus::SE3<float> mTlr, mTrl;
        Eigen::Matrix<float, 3, 3> mRlr;
        Eigen::Vector3f mtlr;

        // IMU linear velocity
        Eigen::Vector3f mVw;
        bool mbHasVelocity;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MOVExtractor *mpMOVExtractor;

        // Frame timestamp.
        double mTimeStamp;

        bool mLost;

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // Stereo baseline multiplied by fx.
        float mbf;

        // Stereo baseline in meters.
        float mb;

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;

        // Number of KeyPoints.
        int N;

        int imageCols;
        int imageRows;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        std::vector<cv::KeyPoint> mvKeysUn;

        std::vector<VideoFeature> mvVF;
        std::map<int, int> mvVFMap;
        std::vector<VideoFeature> mvVFRight;

        // Corresponding stereo coordinate and depth for each keypoint.
        std::vector<MapPoint *> mvpMapPoints;
        // "Monocular" keypoints have a negative value.
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // MOVE descriptor, each row associated to a keypoint.
        std::vector<std::bitset<256>> mDescriptors;

        // MapPoints associated to keypoints, NULL pointer if no association.
        // Flag to identify outlier associations.
        std::vector<bool> mvbOutlier;
        int mnCloseMPs;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        IMU::Bias mPredBias;

        // IMU bias
        IMU::Bias mImuBias;

        // Imu calibration
        IMU::Calib mImuCalib;

        // Imu preintegration from last keyframe
        IMU::Preintegrated *mpImuPreintegrated;
        KeyFrame *mpLastKeyFrame;

        // Pointer to previous frame
        Frame *mpPrevFrame;
        IMU::Preintegrated *mpImuPreintegratedFrame;

        // Current and Next Frame id.
        static long unsigned int nNextId;
        long unsigned int mnId;

        // Reference Keyframe.
        KeyFrame *mpReferenceKF;

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        map<long unsigned int, cv::Point2f> mmProjectPoints;
        map<long unsigned int, cv::Point2f> mmMatchedInImage;

        string mNameFile;

        int mnDataset;

    private:
        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        bool mbIsSet;

        bool mbImuPreintegrated;

        std::mutex *mpMutexImu;

    public:
        GeometricCamera *mpCamera, *mpCamera2;

        // Number of KeyPoints extracted in the left and right images
        int Nleft, Nright;
        // Number of Non Lapping Keypoints
        int monoLeft, monoRight;

        // For stereo matching
        std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

        // For stereo fisheye matching
        static cv::BFMatcher BFmatcher;

        // Triangulated stereo observations using as reference the left camera. These are
        // computed during ComputeStereoFishEyeMatches
        std::vector<Eigen::Vector3f> mvStereo3Dpoints;

        // Grid for the right image
        std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // Stereo fisheye
        void ComputeStereoMatches();

        bool isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight = false);

        Eigen::Vector3f UnprojectStereoFishEye(const int &i);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);

        cv::Mat imgLeft;
        cv::Mat imgRight;

        void PrintPointDistribution()
        {
            int left = 0, right = 0;
            int Nlim = (Nleft != -1) ? Nleft : N;
            for (int i = 0; i < N; i++)
            {
                if (mvpMapPoints[i] && !mvbOutlier[i])
                {
                    if (i < Nlim)
                        left++;
                    else
                        right++;
                }
            }
            cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
        }

        Sophus::SE3<double> T_test;
    };

} // namespace MOV_SLAM

#endif // FRAME_H
