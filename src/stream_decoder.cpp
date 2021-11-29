/****************************************************************************
 *
 * rtsp_image_transport
 * Copyright © 2021 Fraunhofer FKIE
 * Author: Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
#include "stream_decoder.h"

#include "log_level.h"

#include <boost/format.hpp>
#include <sensor_msgs/image_encodings.h>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
}

#include <ros/console.h>

#include <map>
#include <vector>

namespace rtsp_image_transport
{

namespace
{

void free_context(AVCodecContext* ctx)
{
    avcodec_free_context(&ctx);
}

void free_frame(AVFrame* frame)
{
    av_frame_free(&frame);
}

void free_packet(AVPacket* packet)
{
    av_packet_free(&packet);
}

const std::map<VideoCodec, std::vector<std::string>> FFMPEG_DECODERS{
    {VideoCodec::H264, {"h264_cuvid", "h264"}},
    {VideoCodec::H265, {"hevc_cuvid", "h265"}},
    {VideoCodec::MPEG4, {"mpeg4_cuvid", "mpeg4"}},
    {VideoCodec::VP8, {"vp8_cuvid", "vp8"}},
    {VideoCodec::VP9, {"vp9_cuvid", "vp9"}},
    {VideoCodec::MJPEG, {"mjpeg_cuvid", "mjpeg"}}};

}  // namespace

StreamDecoder::StreamDecoder(VideoCodec codec, bool use_hw_decoder)
    : codec_(codec), initialized_(false), width_(0), height_(0),
      last_pixel_format_(AV_PIX_FMT_NONE)
{
    auto decoders = FFMPEG_DECODERS.find(codec);
    if (decoders == FFMPEG_DECODERS.end())
        throw StreamingError(
            (boost::format("no decoder support available for %1%")
             % videoCodecName(codec))
                .str());
    for (const std::string& codec_name : decoders->second)
    {
        if (!use_hw_decoder && codec_name.find("_") != std::string::npos)
            continue;
        AVCodec* decoder = nullptr;
        try
        {
            decoder = avcodec_find_decoder_by_name(codec_name.c_str());
            if (decoder)
            {
                ROS_DEBUG_STREAM("[" << codec_name
                                     << "] attempting to initialize decoder");
                setupDecoder(decoder);
                break;
            }
            else
            {
                ROS_DEBUG_STREAM("["
                                 << codec_name
                                 << "] not available in your FFmpeg library");
            }
        }
        catch (const std::exception& e)
        {
            ctx_.reset();
            ROS_DEBUG_STREAM("[" << decoder->name << "] " << e.what());
        }
    }
    if (!ctx_)
        throw StreamingError(
            (boost::format("no usable decoder available for %1%")
             % videoCodecName(codec))
                .str());
}

void StreamDecoder::setupDecoder(AVCodec* decoder)
{
    ctx_.reset(avcodec_alloc_context3(decoder), free_context);
    if (!ctx_)
        throw StreamingError("failed to initialize decoder context");
    ctx_->log_level_offset = 8;  // Turn errors into warnings
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        ctx_->flags2 |= AV_CODEC_FLAG2_CHUNKS;
        // The error concealment spams the log with spurious errors
        ctx_->error_concealment = 0;
    }
    if (avcodec_open2(ctx_.get(), nullptr, 0) < 0)
        throw StreamingError("failed to open decoder");
}

void StreamDecoder::setDecodeFrames(DecodeFrames which) noexcept
{
    switch (which)
    {
        case DecodeFrames::All:
            ctx_->skip_frame = AVDISCARD_DEFAULT;
            break;
        case DecodeFrames::Intra:
            ctx_->skip_frame = AVDISCARD_NONINTRA;
            break;
        case DecodeFrames::Key:
            ctx_->skip_frame = AVDISCARD_NONKEY;
            break;
        case DecodeFrames::None:
            ctx_->skip_frame = AVDISCARD_ALL;
            break;
    }
}

std::size_t StreamDecoder::decodeVideo(const FrameDataPtr& data)
{
    if (!initialized_)
    {
        pkt_.reset(av_packet_alloc(), free_packet);
        pkt_->pts = AV_NOPTS_VALUE;
        pkt_->dts = AV_NOPTS_VALUE;
        frm_.reset(av_frame_alloc(), free_frame);
        frames_.clear();
        initialized_ = true;
    }
    pkt_->data = const_cast<unsigned char*>(data->data());
    pkt_->size = data->length();
    ctx_->reordered_opaque = data->stamp().toNSec();
    int result;
    if ((result = avcodec_send_packet(ctx_.get(), pkt_.get())) != 0)
    {
        char errbuf[80];
        throw DecodingError(
            (boost::format("failed to send bitstream packet to decoder: %1%")
             % av_make_error_string(errbuf, sizeof(errbuf), result))
                .str());
    }
    std::size_t count = 0;
    while ((result = avcodec_receive_frame(ctx_.get(), frm_.get())) == 0)
    {
        if (!sws_ || frm_->width != width_ || frm_->height != height_
            || frm_->format != last_pixel_format_)
        {
            width_ = frm_->width;
            height_ = frm_->height;
            last_pixel_format_ = static_cast<AVPixelFormat>(frm_->format);
            sws_.reset(sws_getContext(width_, height_, last_pixel_format_,
                                      width_, height_, AV_PIX_FMT_BGR24,
                                      SWS_FAST_BILINEAR, nullptr, nullptr,
                                      nullptr),
                       sws_freeContext);
        }
        sensor_msgs::ImagePtr img{new sensor_msgs::Image()};
        img->encoding = sensor_msgs::image_encodings::BGR8;
        img->header.stamp = data->stamp();
        img->width = width_;
        img->height = height_;
        img->is_bigendian = false;
        img->step = 3 * width_;
        img->data.resize(3 * width_ * height_);
        unsigned char* rgb_data[] = {img->data.data()};
        int rgb_linesize[] = {3 * width_};
        sws_scale(sws_.get(), frm_->data, frm_->linesize, 0, height_, rgb_data,
                  rgb_linesize);
        count++;
        frames_.push_back(img);
    }
    av_frame_unref(frm_.get());
    if (result != AVERROR(EAGAIN))
    {
        char errbuf[80];
        throw DecodingError(
            (boost::format("failed to receive frames from decoder: %1%")
             % av_make_error_string(errbuf, sizeof(errbuf), result))
                .str());
    }
    return count;
}

sensor_msgs::ImageConstPtr StreamDecoder::nextFrame() noexcept
{
    if (frames_.empty())
        return sensor_msgs::ImageConstPtr();
    sensor_msgs::ImageConstPtr img = frames_.front();
    frames_.pop_front();
    return img;
}

VideoCodec StreamDecoder::codec() const noexcept
{
    return codec_;
}

AVCodecContext* StreamDecoder::context() noexcept
{
    return ctx_.get();
}

}  // namespace rtsp_image_transport
