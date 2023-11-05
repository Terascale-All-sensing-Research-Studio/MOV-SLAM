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

#include "Settings.h"

#include "CameraModels/Pinhole.h"

#include "System.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace MOV_SLAM
{

    template <>
    float Settings::readParameter<float>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if (!node.isReal())
        {
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.real();
        }
    }

    template <>
    int Settings::readParameter<int>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if (!node.isInt())
        {
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.operator int();
        }
    }

    template <>
    string Settings::readParameter<string>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if (!node.isString())
        {
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else
        {
            found = true;
            return node.string();
        }
    }

    template <>
    cv::Mat Settings::readParameter<cv::Mat>(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required)
    {
        cv::FileNode node = fSettings[name];
        if (node.empty())
        {
            if (required)
            {
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else
        {
            found = true;
            return node.mat();
        }
    }

    Settings::Settings(const std::string &configFile, const int &sensor) : bNeedToUndistort_(false), bNeedToRectify_(false), bNeedToResize1_(false), bNeedToResize2_(false)
    {
        sensor_ = sensor;

        // Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened())
        {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else
        {
            cout << "Loading settings from " << configFile << endl;
        }

        // Read first camera
        readCamera1(fSettings);
        cout << "\t-Loaded camera 1" << endl;

         //Read second camera if stereo (not rectified)
        if(sensor_ == System::STEREO){
            readCamera2(fSettings);
            cout << "\t-Loaded camera 2" << endl;
        }

        // Read image info
        readImageInfo(fSettings);
        cout << "\t-Loaded image info" << endl;

        readViewer(fSettings);
        cout << "\t-Loaded viewer settings" << endl;
        readLoadAndSave(fSettings);
        cout << "\t-Loaded Atlas settings" << endl;
        readOtherParameters(fSettings);
        cout << "\t-Loaded misc parameters" << endl;
        readMOV(fSettings);
        cout << "\t-Loaded MOV parameters" << endl;
        readOptimization(fSettings);
        cout << "\t-Loaded optimize parameters" << endl;

        if (bNeedToRectify_)
        {
            precomputeRectificationMaps();
            cout << "\t-Computed rectification maps" << endl;
        }

        cout << "----------------------------------" << endl;
    }

    void Settings::readCamera1(cv::FileStorage &fSettings)
    {
        bool found;

        // Read camera model
        string cameraModel = readParameter<string>(fSettings, "Camera.type", found);

        vector<float> vCalibration;
        if (cameraModel == "PinHole")
        {
            cameraType_ = PinHole;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera1.k1", found, false);
            if (found)
            {
                readParameter<float>(fSettings, "Camera1.k3", found, false);
                if (found)
                {
                    vPinHoleDistorsion1_.resize(5);
                    vPinHoleDistorsion1_[4] = readParameter<float>(fSettings, "Camera1.k3", found);
                }
                else
                {
                    vPinHoleDistorsion1_.resize(4);
                }
                vPinHoleDistorsion1_[0] = readParameter<float>(fSettings, "Camera1.k1", found);
                vPinHoleDistorsion1_[1] = readParameter<float>(fSettings, "Camera1.k2", found);
                vPinHoleDistorsion1_[2] = readParameter<float>(fSettings, "Camera1.p1", found);
                vPinHoleDistorsion1_[3] = readParameter<float>(fSettings, "Camera1.p2", found);
            }

            // Check if we need to correct distortion from the images
            if ((sensor_ == System::MONOCULAR) && vPinHoleDistorsion1_.size() != 0)
            {
                bNeedToUndistort_ = true;
            }
        }
        else if (cameraModel == "Rectified")
        {
            cameraType_ = Rectified;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera1.fx", found);
            float fy = readParameter<float>(fSettings, "Camera1.fy", found);
            float cx = readParameter<float>(fSettings, "Camera1.cx", found);
            float cy = readParameter<float>(fSettings, "Camera1.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration1_ = new Pinhole(vCalibration);
            originalCalib1_ = new Pinhole(vCalibration);

            // Rectified images are assumed to be ideal PinHole images (no distortion)
        }
        else
        {
            cerr << "Error: " << cameraModel << " not known" << endl;
            exit(-1);
        }
    }

    void Settings::readCamera2(cv::FileStorage &fSettings)
    {
        bool found;
        vector<float> vCalibration;
        if (cameraType_ == PinHole)
        {
            bNeedToRectify_ = true;

            // Read intrinsic parameters
            float fx = readParameter<float>(fSettings, "Camera2.fx", found);
            float fy = readParameter<float>(fSettings, "Camera2.fy", found);
            float cx = readParameter<float>(fSettings, "Camera2.cx", found);
            float cy = readParameter<float>(fSettings, "Camera2.cy", found);

            vCalibration = {fx, fy, cx, cy};

            calibration2_ = new Pinhole(vCalibration);
            originalCalib2_ = new Pinhole(vCalibration);

            // Check if it is a distorted PinHole
            readParameter<float>(fSettings, "Camera2.k1", found, false);
            if (found)
            {
                readParameter<float>(fSettings, "Camera2.k3", found, false);
                if (found)
                {
                    vPinHoleDistorsion2_.resize(5);
                    vPinHoleDistorsion2_[4] = readParameter<float>(fSettings, "Camera2.k3", found);
                }
                else
                {
                    vPinHoleDistorsion2_.resize(4);
                }
                vPinHoleDistorsion2_[0] = readParameter<float>(fSettings, "Camera2.k1", found);
                vPinHoleDistorsion2_[1] = readParameter<float>(fSettings, "Camera2.k2", found);
                vPinHoleDistorsion2_[2] = readParameter<float>(fSettings, "Camera2.p1", found);
                vPinHoleDistorsion2_[3] = readParameter<float>(fSettings, "Camera2.p2", found);
            }
        }

        // Load stereo extrinsic calibration
        if (cameraType_ == Rectified)
        {
            b_ = readParameter<float>(fSettings, "Stereo.b", found);
            bf_ = b_ * calibration1_->getParameter(0);
        }
        else
        {
            cv::Mat cvTlr = readParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
            Tlr_ = Converter::toSophus(cvTlr);

            // TODO: also search for Trl and invert if necessary

            b_ = Tlr_.translation().norm();
            bf_ = b_ * calibration1_->getParameter(0);
        }

        thDepth_ = readParameter<float>(fSettings, "Stereo.ThDepth", found);
    }

    void Settings::readImageInfo(cv::FileStorage &fSettings)
    {
        bool found;
        // Read original and desired image dimensions
        int originalRows = readParameter<int>(fSettings, "Camera.height", found);
        int originalCols = readParameter<int>(fSettings, "Camera.width", found);
        originalImSize_.width = originalCols;
        originalImSize_.height = originalRows;

        newImSize_ = originalImSize_;
        int newHeigh = readParameter<int>(fSettings, "Camera.newHeight", found, false);
        if (found)
        {
            bNeedToResize1_ = true;
            newImSize_.height = newHeigh;

            if(!bNeedToRectify_){
                //Update calibration
                float scaleRowFactor = (float)newImSize_.height / (float)originalImSize_.height;
                calibration1_->setParameter(calibration1_->getParameter(1) * scaleRowFactor, 1);
                calibration1_->setParameter(calibration1_->getParameter(3) * scaleRowFactor, 3);
            }
        }

        int newWidth = readParameter<int>(fSettings, "Camera.newWidth", found, false);
        if (found)
        {
            bNeedToResize1_ = true;
            newImSize_.width = newWidth;

           if(!bNeedToRectify_){
                //Update calibration
                float scaleColFactor = (float)newImSize_.width /(float) originalImSize_.width;
                calibration1_->setParameter(calibration1_->getParameter(0) * scaleColFactor, 0);
                calibration1_->setParameter(calibration1_->getParameter(2) * scaleColFactor, 2);
            }
        }

        fps_ = readParameter<int>(fSettings, "Camera.fps", found);
        bRGB_ = (bool)readParameter<int>(fSettings, "Camera.RGB", found);
    }

    void Settings::readMOV(cv::FileStorage &fSettings)
    {
        bool found;
        threshold_ = readParameter<int>(fSettings, "MOVExtractor.threshold", found);
        coverageThreshold_ = readParameter<double>(fSettings, "MOVExtractor.coverageThreshold", found);
        relocalizationDistance_ = readParameter<double>(fSettings, "MOVExtractor.relocalizationDistance", found);
    }

    void Settings::readOptimization(cv::FileStorage &fSettings)
    {
        bool found;

        iterationCount_ = readParameter<int>(fSettings, "Optimizer.iterationCount", found);
        reprojectionError_ = readParameter<double>(fSettings, "Optimizer.reprojectionError", found);
        reprojectionErrorLost_ = readParameter<double>(fSettings, "Optimizer.reprojectionErrorLost", found);
        confidence_ = readParameter<double>(fSettings, "Optimizer.confidence", found);
        algorithm_ = readParameter<double>(fSettings, "Optimizer.algorithm", found);
    }

    void Settings::readViewer(cv::FileStorage &fSettings)
    {
        bool found;

        keyFrameSize_ = readParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
        keyFrameLineWidth_ = readParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
        graphLineWidth_ = readParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
        pointSize_ = readParameter<float>(fSettings, "Viewer.PointSize", found);
        cameraSize_ = readParameter<float>(fSettings, "Viewer.CameraSize", found);
        cameraLineWidth_ = readParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
        viewPointX_ = readParameter<float>(fSettings, "Viewer.ViewpointX", found);
        viewPointY_ = readParameter<float>(fSettings, "Viewer.ViewpointY", found);
        viewPointZ_ = readParameter<float>(fSettings, "Viewer.ViewpointZ", found);
        viewPointF_ = readParameter<float>(fSettings, "Viewer.ViewpointF", found);
        imageViewerScale_ = readParameter<float>(fSettings, "Viewer.imageViewScale", found, false);

        if (!found)
            imageViewerScale_ = 1.0f;
    }

    void Settings::readLoadAndSave(cv::FileStorage &fSettings)
    {
        bool found;

        sLoadFrom_ = readParameter<string>(fSettings, "System.LoadAtlasFromFile", found, false);
        sSaveto_ = readParameter<string>(fSettings, "System.SaveAtlasToFile", found, false);
    }

    void Settings::readOtherParameters(cv::FileStorage &fSettings)
    {
        bool found;

        thFarPoints_ = readParameter<float>(fSettings, "System.thFarPoints", found, false);
    }

    void Settings::precomputeRectificationMaps()
    {
        // Precompute rectification maps, new calibrations, ...
        cv::Mat K1 = static_cast<Pinhole *>(calibration1_)->toK();
        K1.convertTo(K1, CV_64F);
        cv::Mat K2 = static_cast<Pinhole *>(calibration2_)->toK();
        K2.convertTo(K2, CV_64F);

        cv::Mat cvTlr;
        cv::eigen2cv(Tlr_.inverse().matrix3x4(), cvTlr);
        cv::Mat R12 = cvTlr.rowRange(0, 3).colRange(0, 3);
        R12.convertTo(R12, CV_64F);
        cv::Mat t12 = cvTlr.rowRange(0, 3).col(3);
        t12.convertTo(t12, CV_64F);

        cv::Mat R_r1_u1, R_r2_u2;
        cv::Mat P1, P2, Q;

        cv::stereoRectify(K1, camera1DistortionCoef(), K2, camera2DistortionCoef(), newImSize_,
                          R12, t12,
                          R_r1_u1, R_r2_u2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY, -1, newImSize_);
        cv::initUndistortRectifyMap(K1, camera1DistortionCoef(), R_r1_u1, P1.rowRange(0, 3).colRange(0, 3),
                                    newImSize_, CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K2, camera2DistortionCoef(), R_r2_u2, P2.rowRange(0, 3).colRange(0, 3),
                                    newImSize_, CV_32F, M1r_, M2r_);

        // Update calibration
        calibration1_->setParameter(P1.at<double>(0, 0), 0);
        calibration1_->setParameter(P1.at<double>(1, 1), 1);
        calibration1_->setParameter(P1.at<double>(0, 2), 2);
        calibration1_->setParameter(P1.at<double>(1, 2), 3);

        // Update bf
        bf_ = b_ * P1.at<double>(0, 0);
    }

    ostream &operator<<(std::ostream &output, const Settings &settings)
    {
        output << "SLAM settings: " << endl;

        output << "\t-Camera 1 parameters (";
        if (settings.cameraType_ == Settings::PinHole || settings.cameraType_ == Settings::Rectified)
        {
            output << "Pinhole";
        }
        else
        {
            output << "Kannala-Brandt";
        }
        output << ")"
               << ": [";
        for (size_t i = 0; i < settings.originalCalib1_->size(); i++)
        {
            output << " " << settings.originalCalib1_->getParameter(i);
        }
        output << " ]" << endl;

        if (!settings.vPinHoleDistorsion1_.empty())
        {
            output << "\t-Camera 1 distortion parameters: [ ";
            for (float d : settings.vPinHoleDistorsion1_)
            {
                output << " " << d;
            }
            output << " ]" << endl;
        }

        output << "\t-Original image size: [ " << settings.originalImSize_.width << " , " << settings.originalImSize_.height << " ]" << endl;
        output << "\t-Current image size: [ " << settings.newImSize_.width << " , " << settings.newImSize_.height << " ]" << endl;

        if (settings.bNeedToRectify_)
        {
            output << "\t-Camera 1 parameters after rectification: [ ";
            for (size_t i = 0; i < settings.calibration1_->size(); i++)
            {
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;
        }
        else if (settings.bNeedToResize1_)
        {
            output << "\t-Camera 1 parameters after resize: [ ";
            for (size_t i = 0; i < settings.calibration1_->size(); i++)
            {
                output << " " << settings.calibration1_->getParameter(i);
            }
            output << " ]" << endl;
        }

        output << "\t-Sequence FPS: " << settings.fps_ << endl;

        if(settings.sensor_ == System::STEREO){
            output << "\t-Stereo baseline: " << settings.b_ << endl;
            output << "\t-Stereo depth threshold : " << settings.thDepth_ << endl;
        }

        return output;
    }
};
