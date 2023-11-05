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

#ifndef SYSTEM_H
#define SYSTEM_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "Settings.h"
#include "Frame.h"

namespace MOV_SLAM
{

    class Verbose
    {
    public:
        enum eLevel
        {
            VERBOSITY_QUIET = 0,
            VERBOSITY_NORMAL = 1,
            VERBOSITY_VERBOSE = 2,
            VERBOSITY_VERY_VERBOSE = 3,
            VERBOSITY_DEBUG = 4
        };

        static eLevel th;

    public:
        static void PrintMess(std::string str, eLevel lev)
        {
            if (lev <= th)
                cout << str << endl;
        }

        static void SetTh(eLevel _th)
        {
            th = _th;
        }
    };

    class Viewer;
    class FrameDrawer;
    class MapDrawer;
    class Atlas;
    class Tracking;
    class LocalMapping;
    class Settings;

    class System
    {
    public:
        // Input sensor
        enum eSensor
        {
            MONOCULAR = 0,
            STEREO = 1,
        };

        // File type
        enum FileType
        {
            TEXT_FILE = 0,
            BINARY_FILE = 1,
        };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Initialize the SLAM system. It launches the Local Mapping, and Viewer threads.
        System(const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string(), const bool bVideoTracking = false);

        Sophus::SE3f TrackMonocular(const double &timestamp, const shared_ptr<MotionVectorImage> &smv);

        Sophus::SE3f TrackStereo(const double &timestamp, const shared_ptr<MotionVectorImage> &smv, const shared_ptr<MotionVectorImage> &smvRight);

        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();
        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();

        // Returns true if there have been a big map change (global BA)
        // since last call to this function
        bool MapChanged();

        // Reset the system (clear Atlas or the active map)
        void Reset();
        void ResetActiveMap();

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();
        bool isShutDown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        void SaveTrajectoryEuRoC(const string &filename);
        void SaveKeyFrameTrajectoryEuRoC(const string &filename);

        void SaveTrajectoryEuRoC(const string &filename, Map *pMap);
        void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap);

        // Save data used for initialization debug
        void SaveDebugData(const int &iniIdx);

        void SavePointCloud(const string &filename);
        void ply_write2file(const string &filename, vector<Eigen::Matrix<float, 6, 1>> &points_3d);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);
        void saveKeyFrameTrajectoryKITTI(const string &filename);

        // TODO: Save/Load functions
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();
        std::vector<MapPoint *> GetTrackedMapPoints();
        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        // For debugging
        double GetTimeFromIMUInit();
        bool isLost();
        bool isFinished();

        void ChangeDataset();

        float GetImageScale();

        inline cv::Size GetImageSize()
        {
            return settings_->newImSize();
        }

        inline float GetFPS()
        {
            return settings_->fps();
        }

        int GetTotalLost();

        void GlobalBundleAdjustment();

#ifdef REGISTER_TIMES
        void InsertRectTime(double &time);
        void InsertResizeTime(double &time);
        void InsertTrackTime(double &time);
#endif

    private:
        void SaveAtlas(int type);
        bool LoadAtlas(int type);

        // Input sensor
        eSensor mSensor;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        // Map* mpMap;
        Atlas *mpAtlas;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
        Tracking *mpTracker;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
        LocalMapping *mpLocalMapper;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer;

        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;

        // System threads: Local Mapping, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptLocalMapping;
        std::thread *mptViewer;

        // Reset flag
        std::mutex mMutexReset;
        bool mbReset;
        bool mbResetActiveMap;

        // Change mode flags
        std::mutex mMutexMode;
        bool mbActivateLocalizationMode;
        bool mbDeactivateLocalizationMode;

        // Shutdown flag
        bool mbShutDown;

        // Tracking state
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;
        std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
        std::mutex mMutexState;

        //
        string mStrLoadAtlasFromFile;
        string mStrSaveAtlasToFile;

        string mStrVocabularyFilePath;

        Settings *settings_;

        bool mbVideoTracking;
    };

} // namespace MOV_SLAM

#endif // SYSTEM_H
