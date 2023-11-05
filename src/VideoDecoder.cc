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

#include "VideoDecoder.h"
#include "Frame.h"

namespace MOV_SLAM
{
    VideoDecoder::VideoDecoder(const std::string &path, int qlen) : dataset_path(path), video_stream_index(-1), qlen(qlen), frames(0)
    {
    }

    VideoDecoder::~VideoDecoder(void)
    {
        avformat_close_input(&pFormatContext);
        av_packet_free(&pPacket);
        av_frame_free(&pFrame);
        avcodec_free_context(&pCodecContext);
        sws_freeContext(conversion_rgb);
        sws_freeContext(conversion_grey);
    }

    bool VideoDecoder::Init()
    {
        // Init the media library
        avdevice_register_all();

        pFormatContext = avformat_alloc_context();
        if (!pFormatContext)
        {
            std::cerr << "ERROR could not allocate memory for Format Context" << std::endl;
            return false;
        }

        inputFormat = av_find_input_format("libx264");

        AVDictionary *options = NULL;
        av_dict_set(&options, "flags2", "+export_mvs", 0); // Needed for mb extraction

        if (avformat_open_input(&pFormatContext, (dataset_path).c_str(), inputFormat, &options) != 0)
        {
            std::cerr << "ERROR could not open the file" << std::endl;
            return false;
        }

        if (avformat_find_stream_info(pFormatContext, NULL) < 0)
        {
            std::cerr << "ERROR could not get the stream info" << std::endl;
            return false;
        }

        for (unsigned int i = 0; i < pFormatContext->nb_streams; i++)
        {
            AVCodecParameters *pLocalCodecParameters = NULL;
            pLocalCodecParameters = pFormatContext->streams[i]->codecpar;

            AVCodec *pLocalCodec = avcodec_find_decoder(pLocalCodecParameters->codec_id);

            if (pLocalCodec == NULL)
            {
                std::cerr << "ERROR unsupported codec!" << std::endl;
                // In this example if the codec is not found we just skip it
                continue;
            }

            // when the stream is a video we store its index, codec parameters and codec
            if (pLocalCodecParameters->codec_type == AVMEDIA_TYPE_VIDEO)
            {
                if (video_stream_index == -1)
                {
                    video_stream_index = i;
                    pCodec = pLocalCodec;
                    pCodecParameters = pLocalCodecParameters;
                }
            }
        }

        if (video_stream_index == -1)
        {
            std::cerr << "File does not contain a video stream! " << std::endl;
            return false;
        }

        pCodecContext = avcodec_alloc_context3(pCodec);
        if (!pCodecContext)
        {
            std::cerr << "failed to allocated memory for AVCodecContext" << std::endl;
            return false;
        }

        if (avcodec_parameters_to_context(pCodecContext, pCodecParameters) < 0)
        {
            std::cerr << "failed to copy codec params to codec context" << std::endl;
            return false;
        }

        if (avcodec_open2(pCodecContext, pCodec, &options) < 0)
        {
            std::cerr << "failed to open codec through avcodec_open2" << std::endl;
            return false;
        }

        pFrame = av_frame_alloc();
        if (!pFrame)
        {
            std::cerr << "failed to allocated memory for AVFrame" << std::endl;
            return false;
        }

        pPacket = av_packet_alloc();
        if (!pPacket)
        {
            std::cerr << "failed to allocated memory for AVPacket" << std::endl;
            return false;
        }

        conversion_rgb = sws_getContext(
            pCodecContext->width, pCodecContext->height,
            pCodecContext->pix_fmt,
            pCodecContext->width, pCodecContext->height,
            AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR,
            NULL, NULL, NULL);

        conversion_grey = sws_getContext(
            pCodecContext->width, pCodecContext->height,
            pCodecContext->pix_fmt,
            pCodecContext->width, pCodecContext->height,
            AV_PIX_FMT_GRAY8, SWS_FAST_BILINEAR,
            NULL, NULL, NULL);

        /* get video fps */
        mFPS = av_q2d(pFormatContext->streams[video_stream_index]->r_frame_rate);

        return true;
    }

    int VideoDecoder::GetWidth()
    {
        return pCodecContext->width;
    }

    int VideoDecoder::GetHeight()
    {
        return pCodecContext->height;
    }

    shared_ptr<MotionVectorImage> VideoDecoder::NextImage(bool mv)
    {
        while (vqueue.size() < qlen)
        {
            if (av_read_frame(pFormatContext, pPacket) >= 0)
            {
            next:
                int response = avcodec_send_packet(pCodecContext, pPacket);
                if (response < 0 || response == AVERROR(EAGAIN) || response == AVERROR_EOF)
                {
                    std::cerr << "Error while sending a packet to the decoder " << std::endl;
                    goto done;
                }
                if (pPacket->stream_index == video_stream_index)
                {
                    response = avcodec_receive_frame(pCodecContext, pFrame);
                    if (response == AVERROR(EAGAIN) || response == AVERROR_EOF)
                    {
                        goto next;
                    }
                    else if (response < 0)
                    {
                        std::cout << "Error while receiving a frame from the decoder " << std::endl;
                        goto done;
                    }
                    int width = pFrame->width;
                    int height = pFrame->height;
                    int dIndx = -1;

                    frames++;
                    shared_ptr<MotionVectorImage> smv(new MotionVectorImage(width, height));
                    smv->frame = frames;

                    smv->ft = pFrame->pict_type != AV_PICTURE_TYPE_I ? FrameType::P_FRAME : FrameType::I_FRAME;
                    avframeToCvmatGrey(pFrame, smv->imGray);
                    avframeToCvmatRGB(pFrame, smv->imRGB);

                    AVFrameSideData *sd = av_frame_get_side_data(pFrame, AV_FRAME_DATA_MOTION_VECTORS);

                    if (sd && mv)
                    {
                        int av_size = sizeof(AVMotionVector);
                        int num_mv = sd->size / av_size;
                        float coverage = 0;
                        float mb_h, mb_w, mb_h_half, mb_w_half, mv_x, mv_y, dst_x, dst_y,
                            d_x_top, d_y_top, d_x_bottom, d_y_bottom, src_x, src_y,
                            s_x_top, s_y_top, s_x_bottom, s_y_bottom;
                        int sMB_size;
                        cv::Vec4i *v, *p;

                        for (int i = 0; i < num_mv; i++)
                        {
                            const AVMotionVector *mv = (AVMotionVector *)&sd->data[i * av_size];

                            mb_h = mv->h * 1;
                            mb_w = mv->w * 1;
                            mb_h_half = mb_h / 2;
                            mb_w_half = mb_w / 2;

                            mv_x = mv->dst_x - mv->src_x;
                            mv_y = mv->dst_y - mv->src_y;

                            mv_x = mv_x / (mv->ref + 1);
                            mv_y = mv_y / (mv->ref + 1);

                            // Calculate dst mb
                            dst_x = mv->ref > 0 && mv->source < 0 ? mv->src_x : mv->dst_x;
                            dst_y = mv->ref > 0 && mv->source < 0 ? mv->src_y : mv->dst_y;

                            d_x_top = dst_x - mb_w_half;
                            if (d_x_top < 0)
                                d_x_top = 0;
                            d_y_top = dst_y - mb_h_half;
                            if (d_y_top < 0)
                                d_y_top = 0;
                            d_x_bottom = dst_x + mb_w_half;
                            if (d_x_bottom >= width)
                                continue;
                            d_y_bottom = dst_y + mb_h_half;
                            if (d_y_bottom >= height)
                                continue;

                            dIndx = -1;
                            cv::Rect dMB(d_x_top, d_y_top, mb_w, mb_h);
                            if (mv->ref > 0 && mv->source < 0)
                            {
                                vqueue[(vqueue.size() - 1) - (mv->ref)]->kps.push_back(dMB);
                            }
                            else
                            {
                                smv->kps.push_back(dMB);
                                dIndx = smv->kps.size() - 1;
                            }

                            if (mv->source > 0) // B Frames
                            {
                                for (int j = 1; j <= (mv->ref + 1); j++)
                                {
                                    src_x = mv->dst_x - (mv_x * j);
                                    src_y = mv->dst_y - (mv_y * j);

                                    // Calculate src mb
                                    s_x_top = src_x - mb_w_half;
                                    if (s_x_top < 0)
                                        s_x_top = 0;
                                    s_y_top = src_y - mb_h_half;
                                    if (s_y_top < 0)
                                        s_y_top = 0;
                                    s_x_bottom = src_x + mb_w_half;
                                    if (s_x_bottom >= width)
                                        s_x_bottom = width - 1;
                                    s_y_bottom = src_y + mb_h_half;
                                    if (s_y_bottom >= height)
                                        s_y_bottom = height - 1;

                                    cv::Rect sMB(s_x_top, s_y_top, mb_w, mb_h);

                                    MotionVector mvc;
                                    mvc.occupied = true;
                                    mvc.pt = cv::Point2f(mv_x, mv_y);
                                    mvc.dIndx = dIndx;
                                    mvc.mb = sMB;

                                    bmap[frames].push_back(mvc);
                                }
                            }
                            else // P Frames
                            {
                                for (int j = (mv->ref + 1); j > 0; j--)
                                {
                                    src_x = mv->dst_x + (mv_x * j * -1);
                                    src_y = mv->dst_y + (mv_y * j * -1);

                                    // Calculate src mb
                                    s_x_top = src_x - mb_w_half;
                                    if (s_x_top < 0)
                                        s_x_top = 0;
                                    s_y_top = src_y - mb_h_half;
                                    if (s_y_top < 0)
                                        s_y_top = 0;
                                    s_x_bottom = src_x + mb_w_half;
                                    if (s_x_bottom >= width)
                                        s_x_bottom = width - 1;
                                    s_y_bottom = src_y + mb_h_half;
                                    if (s_y_bottom >= height)
                                        s_y_bottom = height - 1;

                                    cv::Rect sMB(s_x_top, s_y_top, mb_w, mb_h);

                                    MotionVector mvc;
                                    mvc.occupied = true;
                                    mvc.pt = cv::Point2f(mv_x, mv_y);
                                    mvc.dIndx = dIndx;

                                    shared_ptr<MotionVectorImage> sp;
                                    if (j == 1)
                                    {
                                        sp = smv;
                                    }
                                    else
                                    {
                                        sp = vqueue[vqueue.size() - (j - 1)];
                                    }

                                    sp->mvs.push_back(mvc);

                                    sMB_size = sp->mvs.size() - 1;

                                    // Used for forward predicting
                                    for (int h = s_y_top; h <= s_y_bottom; h++)
                                    {
                                        p = sp->mvi.ptr<cv::Vec4i>(h);
                                        for (int w = s_x_top; w <= s_x_bottom; w++)
                                        {
                                            v = p + w;
                                            if ((*v)[0] == -1)
                                                (*v)[0] = sMB_size;
                                            else if ((*v)[1] == -1)
                                                (*v)[1] = sMB_size;
                                            else if ((*v)[2] == -1)
                                                (*v)[2] = sMB_size;
                                            else
                                                (*v)[3] = sMB_size;
                                        }
                                    }
                                }
                                coverage += dMB.area();
                            }
                        }
                        smv->coverageArea = coverage / (double)(width * height);
                    }
                    vqueue.push_back(smv);
                }
            }
            else
            {
                goto done;
            }
            av_packet_unref(pPacket);
        }

    done:
        if (vqueue.size() > 0)
        {
            shared_ptr<MotionVectorImage> ret = vqueue.front();
            vqueue.pop_front();
            return ret;
        }
        return nullptr;
    }
}
