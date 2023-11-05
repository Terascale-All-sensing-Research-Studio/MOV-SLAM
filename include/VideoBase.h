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

#ifndef VIDEOBASE_H
#define VIDEOBASE_H

#include "Frame.h"
#include <opencv2/core/core.hpp>

#ifdef __cplusplus
extern "C"
{
#endif
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavdevice/avdevice.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/motion_vector.h>
#include <libavutil/frame.h>
#ifdef __cplusplus
}
#endif

namespace MOV_SLAM
{
    class VideoBase
    {
    public:
        inline float fps(void)
        {
            return mFPS;
        }

    protected:
        void avframeToCvmatGrey(const AVFrame *frame, cv::Mat &img)
        {
            cv::Mat image(frame->height, frame->width, CV_8UC1);
            int cvLinesizes[1];
            cvLinesizes[0] = image.step1();
            sws_scale(conversion_grey, frame->data, frame->linesize, 0, frame->height, &image.data,
                      cvLinesizes);
            img = image.clone();
        }

        void avframeToCvmatRGB(const AVFrame *frame, cv::Mat &img)
        {
            cv::Mat image(frame->height, frame->width, CV_8UC3);
            int cvLinesizes[1];
            cvLinesizes[0] = image.step1();
            sws_scale(conversion_rgb, frame->data, frame->linesize, 0, frame->height, &image.data,
                      cvLinesizes);
            img = image.clone();
        }

        AVFrame *cvmatToAvframe(cv::Mat *image, AVFrame *frame)
        {
            int width = image->cols;
            int height = image->rows;
            int cvLinesizes[1];
            cvLinesizes[0] = image->step1();
            if (frame == NULL)
            {
                frame = av_frame_alloc();
                av_image_alloc(frame->data, frame->linesize, width, height,
                               AVPixelFormat::AV_PIX_FMT_YUV420P, 1);
            }
            sws_scale(conversion_yuv, &image->data, cvLinesizes, 0, height, frame->data,
                      frame->linesize);
            return frame;
        }

        SwsContext *conversion_rgb;
        SwsContext *conversion_grey;
        SwsContext *conversion_yuv;
        float mFPS;
    };

}

#endif