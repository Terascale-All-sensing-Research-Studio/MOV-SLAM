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

#ifndef ATLAS_H
#define ATLAS_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GeometricCamera.h"
#include "Pinhole.h"

#include <set>
#include <mutex>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

namespace MOV_SLAM
{
    class Viewer;
    class Map;
    class MapPoint;
    class KeyFrame;
    class Frame;
    class Pinhole;

    // BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

    class Atlas
    {
        friend class boost::serialization::access;

        template <class Archive>
        void serialize(Archive &ar, const unsigned int version)
        {
            ar.template register_type<Pinhole>();

            // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
            // ar & mspMaps;
            ar &mvpBackupMaps;
            ar &mvpCameras;
            // Need to save/load the static Id from Frame, KeyFrame, MapPoint and Map
            ar &Map::nNextId;
            ar &Frame::nNextId;
            ar &KeyFrame::nNextId;
            ar &MapPoint::nNextId;
            ar &GeometricCamera::nNextId;
            ar &mnLastInitKFidMap;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Atlas();
        Atlas(int initKFid); // When its initialization the first map is created
        ~Atlas();

        void CreateNewMap();
        void ChangeMap(Map *pMap);

        unsigned long int GetLastInitKFid();

        void SetViewer(Viewer *pViewer);

        // Method for change components in the current map
        void AddKeyFrame(KeyFrame *pKF);
        void AddMapPoint(MapPoint *pMP);
        // void EraseMapPoint(MapPoint* pMP);
        // void EraseKeyFrame(KeyFrame* pKF);

        GeometricCamera *AddCamera(GeometricCamera *pCam);
        std::vector<GeometricCamera *> GetAllCameras();

        /* All methods without Map pointer work on current map */
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
        void InformNewBigChange();
        int GetLastBigChangeIdx();

        long unsigned int MapPointsInMap();
        long unsigned KeyFramesInMap();

        // Method for get data in current map
        std::vector<KeyFrame *> GetAllKeyFrames();
        std::vector<MapPoint *> GetAllMapPoints();
        std::vector<MapPoint *> GetReferenceMapPoints();

        vector<Map *> GetAllMaps();

        int CountMaps();

        void clearMap();

        void clearAtlas();

        Map *GetCurrentMap();

        void SetMapBad(Map *pMap);
        void RemoveBadMaps();

        bool isInertial();
        void SetInertialSensor();
        void SetImuInitialized();
        bool isImuInitialized();

        // Function for garantee the correction of serialization of this object
        void PreSave();
        void PostLoad();

        map<long unsigned int, KeyFrame *> GetAtlasKeyframes();

        long unsigned int GetNumLivedKF();

        long unsigned int GetNumLivedMP();

    protected:
        std::set<Map *> mspMaps;
        std::set<Map *> mspBadMaps;
        // Its necessary change the container from set to vector because libboost 1.58 and Ubuntu 16.04 have an error with this cointainer
        std::vector<Map *> mvpBackupMaps;

        Map *mpCurrentMap;

        std::vector<GeometricCamera *> mvpCameras;

        unsigned long int mnLastInitKFidMap;

        Viewer *mpViewer;
        bool mHasViewer;

        // Mutex
        std::mutex mMutexAtlas;
    }; // class Atlas

} // namespace MOV_SLAM

#endif // ATLAS_H
