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

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <opencv2/core/eigen.hpp>

namespace MOV_SLAM
{

    Verbose::eLevel Verbose::th = Verbose::VERBOSITY_DEBUG;

    System::System(const string &strSettingsFile, const eSensor sensor,
                   const bool bUseViewer, const int initFr, const string &strSequence, const bool bVideoTracking) : mSensor(sensor), mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), mbResetActiveMap(false),
                                                                                                                    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false), mbVideoTracking(bVideoTracking)
    {
        // Output welcome message
        cout << endl
             << "MOV_SLAM Copyright (C) 2022." << endl
             << "This software pipeline is based on ORB-SLAM3." << endl
             << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
             << "This is free software, and you are welcome to redistribute it" << endl
             << "under certain conditions. See LICENSE.txt." << endl
             << endl;

        cout << "Input sensor was set to: ";

        if (mSensor == MONOCULAR)
            cout << "Monocular" << endl;
        else if (mSensor == STEREO)
            cout << "Stereo" << endl;

        // Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

        cv::FileNode node = fsSettings["File.version"];
        if (!node.empty() && node.isString() && node.string() == "1.0")
        {
            settings_ = new Settings(strSettingsFile, mSensor);

            mStrLoadAtlasFromFile = settings_->atlasLoadFile();
            mStrSaveAtlasToFile = settings_->atlasSaveFile();

            cout << (*settings_) << endl;
        }
        else
        {
            settings_ = nullptr;
            cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
            if (!node.empty() && node.isString())
            {
                mStrLoadAtlasFromFile = (string)node;
            }

            node = fsSettings["System.SaveAtlasToFile"];
            if (!node.empty() && node.isString())
            {
                mStrSaveAtlasToFile = (string)node;
            }
        }

        if (mStrLoadAtlasFromFile.empty())
        {
            // Create the Atlas
            cout << "Initialization of Atlas from scratch " << endl;
            mpAtlas = new Atlas(0);
        }
        else
        {
            cout << "Load File" << endl;

            // Load the file with an earlier session
            // clock_t start = clock();
            cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
            bool isRead = LoadAtlas(FileType::BINARY_FILE);

            if (!isRead)
            {
                cout << "Error to load the file, please try with other session file" << endl;
                exit(-1);
            }
            mpAtlas->CreateNewMap();
        }

        // Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpAtlas);
        mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

        // Initialize the Tracking thread
        //(it will live in the main thread of execution, the one that called this constructor)
        cout << "Seq. Name: " << strSequence << endl;
        mpTracker = new Tracking(this, mpFrameDrawer, mpMapDrawer,
                                 mpAtlas, strSettingsFile, mSensor, settings_, strSequence);
        mpTracker->mbVideoTracking = mbVideoTracking;

        // Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor == MONOCULAR, false);
        mptLocalMapping = new thread(&MOV_SLAM::LocalMapping::Run, mpLocalMapper);
        mpLocalMapper->mInitFr = initFr;
        if (settings_)
            mpLocalMapper->mThFarPoints = settings_->thFarPoints();
        else
            mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
        if (mpLocalMapper->mThFarPoints != 0)
        {
            cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
            mpLocalMapper->mbFarPoints = true;
        }
        else
            mpLocalMapper->mbFarPoints = false;

        // Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);

        mpLocalMapper->SetTracker(mpTracker);

        // Initialize the Viewer thread and launch
        if (bUseViewer)
        // if(false) // TODO
        {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile, settings_);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpViewer->both = mpFrameDrawer->both;
        }

        // Fix verbosity
        Verbose::SetTh(Verbose::VERBOSITY_DEBUG);
    }

    void System::GlobalBundleAdjustment()
    {
        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        for (Map *pMap : vpMaps)
        {
            Optimizer::GlobalBundleAdjustemnt(pMap, 20);
        }
    }

    Sophus::SE3f System::TrackMonocular(const double &timestamp, const shared_ptr<MotionVectorImage> &smv)
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbShutDown)
                return Sophus::SE3f();
        }

        if (mSensor != MONOCULAR)
        {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular Video" << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                cout << "SYSTEM-> Reseting active map in monocular case" << endl;
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(timestamp, smv);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }


    Sophus::SE3f System::TrackStereo(const double &timestamp, const shared_ptr<MotionVectorImage> &smv, const shared_ptr<MotionVectorImage> &smvRight)
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbShutDown)
                return Sophus::SE3f();
        }

        if (mSensor != STEREO)
        {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo Video" << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode)
            {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
                mbResetActiveMap = false;
            }
            else if (mbResetActiveMap)
            {
                cout << "SYSTEM-> Reseting active map in stereo case" << endl;
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
            }
        }

        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(timestamp, smv, smvRight);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

        return Tcw;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged()
    {
        static int n = 0;
        int curn = mpAtlas->GetLastBigChangeIdx();
        if (n < curn)
        {
            n = curn;
            return true;
        }
        else
            return false;
    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::ResetActiveMap()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    void System::Shutdown()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbShutDown = true;
        }

        cout << "Shutdown" << endl;

        mpLocalMapper->RequestFinish();
        
        if (!mStrSaveAtlasToFile.empty())
        {
            Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
            SaveAtlas(FileType::BINARY_FILE);
        }
    }

    bool System::isShutDown()
    {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDown;
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        // if (mSensor == MONOCULAR)
        //{
        //     cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        //     return;
        // }

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<MOV_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                          lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad())
            {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
        // cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveKeyFrameTrajectoryTUM(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpAtlas->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
              << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }

        f.close();
    }

    void System::SaveTrajectoryEuRoC(const string &filename)
    {

        cout << endl
             << "Saving trajectory to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        size_t numMaxKFs = 0;
        Map *pBiggerMap = nullptr;
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
        for (Map *pMap : vpMaps)
        {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
            if (pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
        Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<MOV_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pBiggerMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            Sophus::SE3f Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveTrajectoryEuRoC(const string &filename, Map *pMap)
    {

        cout << endl
             << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;
        /*if(mSensor==MONOCULAR)
        {
            cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." << endl;
            return;
        }*/

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
        Twb = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        // cout << "file open" << endl;
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<MOV_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
        // cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.size() << endl;
        // cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl;
        // cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                  lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++, lbL++)
        {
            // cout << "1" << endl;
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;
            // cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            // cout << "2.5" << endl;

            while (pKF->isBad())
            {
                // cout << " 2.bad" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
                // cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pMap)
            {
                // cout << "--Parent KF is from another map" << endl;
                continue;
            }

            // cout << "3" << endl;

            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            // cout << "4" << endl;

            Sophus::SE3f Twc = ((*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

            // cout << "5" << endl;
        }
        // cout << "end saving trajectory" << endl;
        f.close();
        cout << endl
             << "End of saving trajectory to " << filename << " ..." << endl;
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBiggerMap;
        size_t numMaxKFs = 0;
        for (Map *pMap : vpMaps)
        {
            if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
            {
                numMaxKFs = pMap->GetAllKeyFrames().size();
                pBiggerMap = pMap;
            }
        }

        if (pBiggerMap == nullptr)
        {
            std::cout << "There is not a map!!" << std::endl;
            return;
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (!pKF || pKF->isBad())
                continue;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << pKF->mnFrameId << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
    }

    void System::SaveKeyFrameTrajectoryEuRoC(const string &filename, Map *pMap)
    {
        cout << endl
             << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];

            if (!pKF || pKF->isBad())
                continue;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        f.close();
    }

    void System::saveKeyFrameTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving keyframe trajectory to " << filename << " ..." << endl;

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        vector<Map *> maps = mpAtlas->GetAllMaps();
        for (Map *map : maps)
        {
            if(!map) continue;

            vector<KeyFrame *> vpKFs = map->GetAllKeyFrames();
            if(vpKFs.size() == 0) continue;

            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

            // Transform all keyframes so that the first keyframe is at the origin.
            // After a loop closure the first keyframe might not be at the origin.
            
            Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

            for (size_t i = 0; i < vpKFs.size(); i++)
            { 
                KeyFrame *pKF = vpKFs[i];

                if(!pKF) continue;
               

                // pKF->SetPose(pKF->GetPose()*Two);

                Sophus::SE3f Trw;

                while (pKF->isBad())
                {
                    Trw = Trw * pKF->mTcp;
                    pKF = pKF->GetParent();
                }

                Trw = Trw * pKF->GetPose() * Tow;

                Sophus::SE3f Twc = Trw.inverse();
                Eigen::Matrix3f Rwc = Twc.rotationMatrix();
                Eigen::Vector3f twc = Twc.translation();

                f << setprecision(9) << pKF->mnFrameId << " " << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
            }
        }

        f.close();
    }

    void System::SaveTrajectoryKITTI(const string &filename)
    {
        cout << endl
             << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR)
        {
             cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
             return;
        }
        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        vector<Map *> maps = mpAtlas->GetAllMaps();
        for (Map *map : maps)
        {
            vector<KeyFrame *> vpKFs = map->GetAllKeyFrames();

            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

            // Transform all keyframes so that the first keyframe is at the origin.
            // After a loop closure the first keyframe might not be at the origin.
            Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

            // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
            // We need to get first the keyframe pose and then concatenate the relative transformation.
            // Frames not localized (tracking failure) are not saved.

            // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
            // which is true when tracking failed (lbL).
            list<MOV_SLAM::KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
            list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
            for (list<Sophus::SE3f>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
                                              lend = mpTracker->mlRelativeFramePoses.end();
                 lit != lend; lit++, lRit++, lT++)
            {
                MOV_SLAM::KeyFrame *pKF = *lRit;

                Sophus::SE3f Trw;

                if (!pKF)
                    continue;

                while (pKF->isBad())
                {
                    Trw = Trw * pKF->mTcp;
                    pKF = pKF->GetParent();
                }

                Trw = Trw * pKF->GetPose() * Tow;

                Sophus::SE3f Tcw = (*lit) * Trw;
                Sophus::SE3f Twc = Tcw.inverse();
                Eigen::Matrix3f Rwc = Twc.rotationMatrix();
                Eigen::Vector3f twc = Twc.translation();

                f << setprecision(9) << pKF->mnFrameId << " " << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2) << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " " << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1) << " " << Rwc(2, 2) << " " << twc(2) << endl;
            }
        }
        f.close();
    }

    void System::SavePointCloud(const string &filename)
    {
        std::vector<Eigen::Matrix<float, 6, 1>> vertices;
        vector<Map *> maps = mpAtlas->GetAllMaps();
        for (Map *map : maps)
        {
            const vector<MapPoint *> &vpMPs = map->GetAllMapPoints();

            for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
            {
                if (vpMPs[i]->isBad())
                    continue;

                const std::map<KeyFrame *, std::tuple<int, int>> &observations = vpMPs[i]->GetObservations();
                std::map<KeyFrame *, std::tuple<int, int>>::const_iterator beginObs = observations.begin();
                cv::Point2f pt = beginObs->first->mvKeysUn[get<0>(beginObs->second)].pt;

                Eigen::Matrix<float, 6, 1> vertex;
                cv::Vec3b rgb = beginObs->first->mImage.at<cv::Vec3b>(pt);
                Eigen::Matrix<float, 3, 1> wPT = vpMPs[i]->GetWorldPos();
                vertex(0) = wPT(0);
                vertex(1) = wPT(1);
                vertex(2) = wPT(2);
                vertex(3) = rgb[0];
                vertex(4) = rgb[1];
                vertex(5) = rgb[2];
                vertices.push_back(vertex);
            }
        }
        ply_write2file(filename, vertices);
    }

    void System::ply_write2file(const string &filename, vector<Eigen::Matrix<float, 6, 1>> &points_3d)
    {
        ofstream f;
        f.open(filename.c_str());
        f << fixed;
        f << "ply" << std::endl;
        f << "format ascii 1.0" << std::endl;
        f << "element vertex " << points_3d.size() << std::endl;
        f << "property float x " << std::endl;
        f << "property float y " << std::endl;
        f << "property float z " << std::endl;
        f << "property uchar red " << std::endl;
        f << "property uchar green" << std::endl;
        f << "property uchar blue" << std::endl;

        f << "end_header " << std::endl;
        for (size_t i = 0; i < points_3d.size(); i++)
        {
            f << points_3d[i](0) << " " << points_3d[i](1) << " " << points_3d[i](2) << " " << (int)points_3d[i](3) << " " << (int)points_3d[i](4) << " " << (int)points_3d[i](5) << std::endl;
        }
        f.close();
    }

    void System::SaveDebugData(const int &initIdx)
    {
        // 0. Save initialization trajectory
        SaveTrajectoryEuRoC("init_FrameTrajectoy_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt");

        // 1. Save scale
        ofstream f;
        f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mScale << endl;
        f.close();

        // 2. Save gravity direction
        f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << "," << mpLocalMapper->mRwg(0, 2) << endl;
        f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << "," << mpLocalMapper->mRwg(1, 2) << endl;
        f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << "," << mpLocalMapper->mRwg(2, 2) << endl;
        f.close();

        // 3. Save computational cost
        f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mCostTime << endl;
        f.close();

        // 4. Save biases
        f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << "," << mpLocalMapper->mbg(2) << endl;
        f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << "," << mpLocalMapper->mba(2) << endl;
        f.close();

        // 5. Save covariance matrix
        f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" + to_string(initIdx) + ".txt", ios_base::app);
        f << fixed;
        for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++)
        {
            for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++)
            {
                if (j != 0)
                    f << ",";
                f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
            }
            f << endl;
        }
        f.close();

        // 6. Save initialization time
        f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt", ios_base::app);
        f << fixed;
        f << mpLocalMapper->mInitTime << endl;
        f.close();
    }

    int System::GetTrackingState()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
    {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPointsUn;
    }

    double System::GetTimeFromIMUInit()
    {
        double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        if ((aux > 0.) && mpAtlas->isImuInitialized())
            return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        else
            return 0.f;
    }

    bool System::isLost()
    {
        if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
            return true;
        else
            return false;
    }

    bool System::isFinished()
    {
        return (GetTimeFromIMUInit() > 0.1);
    }

    void System::ChangeDataset()
    {
        if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
        {
            mpTracker->ResetActiveMap();
        }
        else
        {
            mpTracker->CreateMapInAtlas();
        }

        mpTracker->NewDataset();
    }

    float System::GetImageScale()
    {
        return mpTracker->GetImageScale();
    }

    int System::GetTotalLost()
    {
        return mpTracker->lost;
    }

    void System::SaveAtlas(int type)
    {
        if (!mStrSaveAtlasToFile.empty())
        {
            // clock_t start = clock();

            // Save the current session
            mpAtlas->PreSave();

            string pathSaveFileName = "./";
            pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
            pathSaveFileName = pathSaveFileName.append(".osa");

            if (type == TEXT_FILE) // File text
            {
                cout << "Starting to write the save text file " << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::text_oarchive oa(ofs);

                oa << mpAtlas;
                cout << "End to write the save text file" << endl;
            }
            else if (type == BINARY_FILE) // File binary
            {
                cout << "Starting to write the save binary file" << endl;
                std::remove(pathSaveFileName.c_str());
                std::ofstream ofs(pathSaveFileName, std::ios::binary);
                boost::archive::binary_oarchive oa(ofs);
                oa << mpAtlas;
                cout << "End to write save binary file" << endl;
            }
        }
    }

    bool System::LoadAtlas(int type)
    {
        string strFileVoc, strVocChecksum;
        bool isRead = false;

        string pathLoadFileName = "./";
        pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
        pathLoadFileName = pathLoadFileName.append(".osa");

        if (type == TEXT_FILE) // File text
        {
            cout << "Starting to read the save text file " << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save text file " << endl;
            isRead = true;
        }
        else if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(pathLoadFileName, std::ios::binary);
            if (!ifs.good())
            {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> strFileVoc;
            ia >> strVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }

        if (isRead)
        {
            mpAtlas->PostLoad();

            return true;
        }
        return false;
    }
} // namespace MOV_SLAM
