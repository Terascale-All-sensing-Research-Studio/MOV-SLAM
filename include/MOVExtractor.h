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

#ifndef MOVEXTRACTOR_H
#define MOVEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include "Frame.h"

namespace MOV_SLAM
{
    class MOVExtractor
    {
    public:
        MOVExtractor(int threshold = 20, double coverageThreshold = 0.60, double relocalizationDistance = 0.25);

        ~MOVExtractor() {}

        int operator()(const shared_ptr<MotionVectorImage> &_smv, 
            std::vector<cv::KeyPoint> &_keypoints, std::vector<VideoFeature> &_vf, std::map<int,int> &_vfmap, std::vector<std::bitset<256>> &descriptors, Frame *_prev_frame);

        int mCurrentId;
        int mThreshold;
        double mCoverageThreshold;
        double mRelocalizationDistance;
    };

} // namespace MOV_SLAM

#endif
