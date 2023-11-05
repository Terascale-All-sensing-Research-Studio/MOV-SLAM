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

#ifndef VIDEODECODER_H
#define VIDEODECODER_H

#include "Frame.h"
#include "VideoBase.h"
#include <opencv2/core/core.hpp>
#include <deque>

namespace MOV_SLAM
{
    class VideoDecoder : public VideoBase
    {
    public:
        VideoDecoder(const std::string &path, int qlen);
        virtual ~VideoDecoder(void);

        // Initialize the video decoder
        bool Init(void);

        // Grab the next image from the video source
        shared_ptr<MotionVectorImage> NextImage(bool mv = true);

        int GetWidth();

        int GetHeight();

    private:
        const std::string dataset_path;
        int video_stream_index;
        int qlen;
        int frames;
        
        AVFormatContext *pFormatContext;
        AVInputFormat *inputFormat;
        AVCodec *pCodec;
        AVCodecParameters *pCodecParameters;
        AVCodecContext *pCodecContext;
        AVFrame *pFrame;
        AVPacket *pPacket;

        std::deque<std::shared_ptr<MotionVectorImage>> vqueue;
        std::map<int,std::vector<MotionVector>> bmap;
    };
}

#endif
