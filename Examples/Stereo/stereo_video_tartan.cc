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

///// ***************** VALID RESULTS WITHOUT LOOP CLOSURE

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "Frame.h"
#include "System.h"
#include "VideoDecoder.h"

using namespace std;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << endl
             << "Usage: ./stereo_kitti_tartanair path_to_settings path_to_stream" << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    MOV_SLAM::System SLAM(argv[1], MOV_SLAM::System::STEREO, true, 0, std::string(), true);

    double fps = SLAM.GetFPS();

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop

    string path(argv[2]);
    MOV_SLAM::VideoDecoder decoder(path, 12);

    std::cout << "Initializing the decoder" << std::endl;

    if (!decoder.Init())
    {
        cout << "Failed initializing the video decoder";
        return -1;
    }

    std::cout << "OpenCV Threads is " << cv::getNumThreads() << std::endl;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    int imageCnt = 0;
    double imageDur = 0.0;

    while (true)
    {
        shared_ptr<MOV_SLAM::MotionVectorImage> smv = decoder.NextImage();

        if (smv) // Grab the left image
        {
            shared_ptr<MOV_SLAM::MotionVectorImage> smvRight = decoder.NextImage(false);
            if (smvRight) // Grab the right image
            {
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

                double tframe = std::chrono::duration_cast<std::chrono::duration<double, std::chrono::seconds::period>>(std::chrono::steady_clock::now() - start).count();
                // Pass the image to the SLAM system
                SLAM.TrackStereo(tframe, smv, smvRight);

                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

                // Wait to load the next frame
                double duration = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
                imageDur += duration;
                //std::cout << "duration is " << duration << std::endl;
                double sleep = (((1000 / fps) - duration) * 1000);
                // std::cout << "sleep is " << sleep << std::endl;
                if (sleep < 0)
                    sleep = 0;
                usleep(sleep);
                imageCnt++;
            }
        }
        else
        {
            std::cout << "Received false from next image" << std::endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    int totalLost = SLAM.GetTotalLost();

    std::cout << "Saving Key Frame Trajectory" << std::endl;
    SLAM.saveKeyFrameTrajectoryKITTI("TrajectoryKITTIKeyFrame.txt");
    std::cout << "Completed Save Trajectory" << std::endl;

    std::cout << "Saving results" << std::endl;
    std::ofstream out("results.txt");
    out << imageCnt << "," << totalLost << "," << (imageDur / imageCnt) << std::endl;
    out.close();

    std::cout << "All Done." << std::endl;

    return 0;
}
