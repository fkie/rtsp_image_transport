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
#include "stream_encoder.h"

#include "log_level.h"

#include <boost/format.hpp>
#include <sensor_msgs/image_encodings.h>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <ros/console.h>

#include <map>
#include <vector>

namespace rtsp_image_transport
{

namespace
{

void set_codec_option(std::shared_ptr<AVCodecContext> ctx,
                      const std::string& option, const std::string& value,
                      bool silent = false)
{
    int result = av_opt_set(ctx->priv_data, option.c_str(), value.c_str(), 0);
    if (result != 0 && !silent)
        ROS_WARN_NAMED("ffmpeg", "[%s] cannot set codec option %s=\"%s\"",
                       ctx->codec->name, option.c_str(), value.c_str());
}

void set_codec_option(std::shared_ptr<AVCodecContext> ctx,
                      const std::string& option, int value, bool silent = false)
{
    int result = av_opt_set_int(ctx->priv_data, option.c_str(), value, 0);
    if (result != 0 && !silent)
        ROS_WARN_NAMED("ffmpeg", "[%s] cannot set codec option %s=%d",
                       ctx->codec->name, option.c_str(), value);
}

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

#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
AVPixelFormat getVAAPIFormat(AVCodecContext* ctx, const AVPixelFormat* formats)
{
    for (const AVPixelFormat* p = formats; *p != AV_PIX_FMT_NONE; ++p)
    {
        if (*p == AV_PIX_FMT_VAAPI)
            return *p;
    }
    ROS_ERROR("Hardware encoder does not support VAAPI format");
    return AV_PIX_FMT_NONE;
}

void free_buffer(AVBufferRef* buffer)
{
    av_buffer_unref(&buffer);
}
#endif

AVPixelFormat toAVPixelFormat(const sensor_msgs::Image& image)
{
    if (image.encoding == sensor_msgs::image_encodings::BGR8)
        return AV_PIX_FMT_BGR24;
    if (image.encoding == sensor_msgs::image_encodings::RGB8)
        return AV_PIX_FMT_RGB24;
    if (image.encoding == sensor_msgs::image_encodings::RGBA8)
        return AV_PIX_FMT_RGBA;
    if (image.encoding == sensor_msgs::image_encodings::BGRA8)
        return AV_PIX_FMT_BGRA;
    if (image.encoding == sensor_msgs::image_encodings::MONO8)
        return AV_PIX_FMT_GRAY8;
    if (image.encoding == sensor_msgs::image_encodings::RGB16)
        return image.is_bigendian ? AV_PIX_FMT_RGB48BE : AV_PIX_FMT_RGB48LE;
    if (image.encoding == sensor_msgs::image_encodings::BGR16)
        return image.is_bigendian ? AV_PIX_FMT_BGR48BE : AV_PIX_FMT_BGR48LE;
    if (image.encoding == sensor_msgs::image_encodings::RGBA16)
        return image.is_bigendian ? AV_PIX_FMT_RGBA64BE : AV_PIX_FMT_RGBA64LE;
    if (image.encoding == sensor_msgs::image_encodings::BGRA16)
        return image.is_bigendian ? AV_PIX_FMT_BGRA64BE : AV_PIX_FMT_BGRA64LE;
    if (image.encoding == sensor_msgs::image_encodings::MONO16)
        return image.is_bigendian ? AV_PIX_FMT_GRAY16BE : AV_PIX_FMT_GRAY16LE;
    if (image.encoding == sensor_msgs::image_encodings::YUV422)
        return AV_PIX_FMT_UYVY422;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_RGGB8)
        return AV_PIX_FMT_BAYER_RGGB8;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_BGGR8)
        return AV_PIX_FMT_BAYER_BGGR8;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_GBRG8)
        return AV_PIX_FMT_BAYER_GBRG8;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_GRBG8)
        return AV_PIX_FMT_BAYER_GRBG8;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_RGGB16)
        return image.is_bigendian ? AV_PIX_FMT_BAYER_RGGB16BE
                                  : AV_PIX_FMT_BAYER_RGGB16LE;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_BGGR16)
        return image.is_bigendian ? AV_PIX_FMT_BAYER_BGGR16BE
                                  : AV_PIX_FMT_BAYER_BGGR16LE;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_GBRG16)
        return image.is_bigendian ? AV_PIX_FMT_BAYER_GBRG16BE
                                  : AV_PIX_FMT_BAYER_GBRG16LE;
    if (image.encoding == sensor_msgs::image_encodings::BAYER_GRBG16)
        return image.is_bigendian ? AV_PIX_FMT_BAYER_GRBG16BE
                                  : AV_PIX_FMT_BAYER_GRBG16LE;
    return AV_PIX_FMT_NONE;
}

bool scanForStartCode(const unsigned char* data, std::size_t length,
                      std::size_t offset, std::size_t& start_code_pos,
                      std::size_t& start_code_length)
{
    std::size_t i = offset, last = length - 4;
    while (i < last)
    {
        if (data[i + 2] == 0x01)
        {
            if (data[i] == 0x00 && data[i + 1] == 0x00)
            {
                start_code_pos = i;
                start_code_length = 3;
                return true;
            }
            else
            {
                i += 3;
            }
        }
        else if (data[i + 3] == 0x01)
        {
            if (data[i] == 0x00 && data[i + 1] == 0x00 && data[i + 2] == 0x00)
            {
                start_code_pos = i;
                start_code_length = 4;
                return true;
            }
            else
            {
                i += 4;
            }
        }
        else if (data[i + 3] != 0x00)
        {
            i += 4;
        }
        else if (data[i + 2] != 0x00)
        {
            i += 3;
        }
        else if (data[i + 1] != 0x00)
        {
            i += 2;
        }
        else
        {
            i += 1;
        }
    }
    return false;
}

template<class FrameContainer>
std::size_t splitNALs(FrameContainer& output, const unsigned char* data,
                      std::size_t length, const ros::Time& stamp)
{
    std::size_t nal_start, sc_len, nal_end = 0, count = 0;
    if (!scanForStartCode(data, length, 0, nal_start, sc_len))
        return 0;
    nal_start += sc_len;
    while (scanForStartCode(data, length, nal_start, nal_end, sc_len))
    {
        output.push_back(std::make_shared<FrameData>(
            data + nal_start, nal_end - nal_start, stamp));
        nal_start = nal_end + sc_len;
        count++;
    }
    output.push_back(std::make_shared<FrameData>(data + nal_start,
                                                 length - nal_start, stamp));
    return count + 1;
}

const std::map<VideoCodec, std::vector<std::string>> FFMPEG_ENCODERS{
    {VideoCodec::H264,
     {"h264_nvenc", "h264_omx", "h264_vaapi", "libx264", "h264"}},
    {VideoCodec::H265, {"hevc_nvenc", "hevc_vaapi", "libx265", "h265"}},
    {VideoCodec::MPEG4, {"mpeg4_omx", "libxvid", "mpeg4"}},
    {VideoCodec::VP8, {"vp8_vaapi", "libvpx", "vp8"}},
    {VideoCodec::VP9, {"vp9_vaapi", "libvpx-vp9", "vp9"}}};

}  // namespace

StreamEncoder::StreamEncoder(VideoCodec codec, bool use_hw_encoder)
    : codec_(codec), initialized_(false), is_vaapi_(false),
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
      hw_device_ctx_(nullptr), hw_frames_ctx_(nullptr),
#endif
      last_pixel_format_(AV_PIX_FMT_NONE), last_pts_(-1), picture_number_(0)
{
    auto encoders = FFMPEG_ENCODERS.find(codec);
    if (encoders == FFMPEG_ENCODERS.end())
        throw StreamingError(
            (boost::format("no encoder support available for %1%")
             % videoCodecName(codec))
                .str());
    for (const std::string& codec_name : encoders->second)
    {
        if (!use_hw_encoder && codec_name.find("_") != std::string::npos)
            continue;
        AVCodec* encoder = nullptr;
        try
        {
            encoder = avcodec_find_encoder_by_name(codec_name.c_str());
            if (encoder)
            {
                /* Setup encoder to see if it actually works */
                ROS_DEBUG_STREAM("[" << codec_name
                                     << "] attempting to initialize encoder");
                {
                    TemporaryAvLogLevel ll(AV_LOG_PANIC);
                    setupEncoder(encoder, true);
                    openEncoder(640, 480);
                }
                /* Encoder seems fine, now initialize it for real */
                setupEncoder(encoder, false);
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
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
            hw_device_.reset();
            hw_frames_.reset();
#endif
            ROS_DEBUG_STREAM("[" << encoder->name << "] " << e.what());
        }
    }
    if (!ctx_)
        throw StreamingError(
            (boost::format("no usable encoder available for %1%")
             % videoCodecName(codec))
                .str());
}

void StreamEncoder::setupEncoder(AVCodec* encoder, bool silent)
{
    ctx_.reset();
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    hw_device_.reset();
    hw_frames_.reset();
#endif
    is_vaapi_ = strstr(encoder->name, "vaapi") != nullptr;
    if (is_vaapi_)
    {
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
        char errbuf[80];
        AVBufferRef* device;
        int result;
        if ((result = av_hwdevice_ctx_create(&device, AV_HWDEVICE_TYPE_VAAPI,
                                             "/dev/dri/renderD128", nullptr, 0))
            != 0)
            throw StreamingError(
                (boost::format("failed to allocate VAAPI device context: %1%")
                 % av_make_error_string(errbuf, sizeof(errbuf), result))
                    .str());
        hw_device_.reset(device, free_buffer);
        hw_device_ctx_ = reinterpret_cast<AVHWDeviceContext*>(hw_device_->data);
#else
        throw StreamingError("FFmpeg library lacks hwframe support for VAAPI");
#endif
    }
    ctx_.reset(avcodec_alloc_context3(encoder), free_context);
    if (!ctx_)
        throw StreamingError("failed to initialize encoder context");
    ctx_->log_level_offset = 8;  // Turn errors into warnings
    ctx_->time_base = AVRational{1, 300};
    ctx_->gop_size = 30;
    ctx_->bit_rate = 1000000;  // 1MBit/s
    ctx_->framerate = AVRational{30, 1};
    ctx_->rc_min_rate = 0;
    ctx_->rc_max_rate = ctx_->bit_rate;
    ctx_->rc_buffer_size = 2 * ctx_->bit_rate;
    ctx_->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
    ctx_->flags |= AV_CODEC_FLAG_CLOSED_GOP;
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        set_codec_option(ctx_, "profile", "main", silent);
        if (strstr(encoder->name, "nvenc"))
        {
            set_codec_option(ctx_, "preset", "llhp", silent);
            set_codec_option(ctx_, "zerolatency", 1, silent);
        }
        else if (strstr(encoder->name, "x26"))
        {
            set_codec_option(ctx_, "preset", "fast", silent);
            set_codec_option(ctx_, "tune", "zerolatency", silent);
        }
        if (is_vaapi_)
        {
            set_codec_option(ctx_, "rc_mode", "CQP", silent);
        }
    }
    if (codec_ == VideoCodec::H264)
    {
        if (strstr(encoder->name, "x264"))
            set_codec_option(ctx_, "b-pyramid", 0, silent);
    }
    if (codec_ == VideoCodec::VP8 || codec_ == VideoCodec::VP9)
    {
        set_codec_option(ctx_, "deadline", "realtime", silent);
    }
    if (codec_ == VideoCodec::MPEG4)
    {
        ctx_->max_b_frames = 0;
        ctx_->trellis = 1;
        ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    }
}

void StreamEncoder::setBitrate(unsigned long bit_rate)
{
    if (initialized_)
        throw StreamingError(
            "cannot modify bitrate after encoding has started");
    ctx_->bit_rate = bit_rate;
    ctx_->rc_max_rate = bit_rate;
}

void StreamEncoder::setFramerate(unsigned fps)
{
    if (initialized_)
        throw StreamingError(
            "cannot modify frame rate after encoding has started");
    ctx_->gop_size = fps;
    ctx_->framerate = AVRational{static_cast<int>(fps), 1};
}

void StreamEncoder::setPackageSizeHint(unsigned size)
{
    if (initialized_)
        throw StreamingError(
            "cannot modify package size hint after encoding has started");
    if (codec_ == VideoCodec::H264 && strstr(ctx_->codec->name, "x264"))
    {
        set_codec_option(ctx_, "slice-max-size", size);
    }
}

void StreamEncoder::openEncoder(int width, int height)
{
    int result;
    char errbuf[80];

    ctx_->width = width;
    ctx_->height = height;
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    if (is_vaapi_)
    {
        AVBufferRef* frames = av_hwframe_ctx_alloc(hw_device_.get());
        if (!frames)
            throw StreamingError("failed to allocate VAAPI frame buffer");
        hw_frames_.reset(frames, free_buffer);
        hw_frames_ctx_ = reinterpret_cast<AVHWFramesContext*>(hw_frames_->data);
        hw_frames_ctx_->format = AV_PIX_FMT_VAAPI;
        hw_frames_ctx_->sw_format = AV_PIX_FMT_NV12;
        hw_frames_ctx_->width = width;
        hw_frames_ctx_->height = height;
        hw_frames_ctx_->initial_pool_size = 10;
        if ((result = av_hwframe_ctx_init(hw_frames_.get())) != 0)
        {
            throw StreamingError(
                (boost::format("failed to initialize VAAPI frame buffer: %1%")
                 % av_make_error_string(errbuf, sizeof(errbuf), result))
                    .str());
        }
        ctx_->hw_frames_ctx = av_buffer_ref(hw_frames_.get());
        ctx_->get_format = getVAAPIFormat;
        ctx_->pix_fmt = AV_PIX_FMT_VAAPI;
    }
    else
    {
        ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
    }
#else
    ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
#endif
    if ((result = avcodec_open2(ctx_.get(), nullptr, 0)) != 0)
    {
        throw StreamingError(
            (boost::format("failed to open encoder: %1%")
             % av_make_error_string(errbuf, sizeof(errbuf), result))
                .str());
    }
}

std::size_t StreamEncoder::encodeVideo(const sensor_msgs::Image& image)
{
    int result;
    if (!initialized_)
    {
        openEncoder(image.width, image.height);
        sw_frm_.reset(av_frame_alloc(), free_frame);
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
        if (is_vaapi_)
        {
            hw_frm_.reset(av_frame_alloc(), free_frame);
            if (av_hwframe_get_buffer(hw_frames_.get(), hw_frm_.get(), 0) != 0)
                throw StreamingError(
                    "failed to allocate HW frame from VAAPI frame buffer");
        }
#endif
        pkt_.reset(av_packet_alloc(), free_packet);
        first_ts_ = image.header.stamp;
        last_pts_ = -1;
        picture_number_ = 0;
        packets_.clear();
        last_pixel_format_ = AV_PIX_FMT_NONE;
        initialized_ = true;
    }
    if (image.width != static_cast<unsigned>(ctx_->width)
        || image.height != static_cast<unsigned>(ctx_->height))
        throw StreamingError("image size changed unexpectedly");

    AVPixelFormat av_format = toAVPixelFormat(image);
    if (av_format == AV_PIX_FMT_NONE)
        throw StreamingError(
            (boost::format("unsupported image format %1%") % image.encoding)
                .str());

    const uint8_t* input_data[] = {image.data.data()};
    int input_linesize[] = {int(image.step)};
    av_frame_unref(sw_frm_.get());
    sw_frm_->width = image.width;
    sw_frm_->height = image.height;
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    sw_frm_->format = is_vaapi_ ? hw_frames_ctx_->sw_format : ctx_->pix_fmt;
#else
    sw_frm_->format = ctx_->pix_fmt;
#endif
    if (av_frame_get_buffer(sw_frm_.get(), 0) != 0)
        throw StreamingError("failed to allocate encoding frame buffer");

    if (!sws_ || av_format != last_pixel_format_)
    {
        sws_.reset(sws_getContext(image.width, image.height, av_format,
                                  image.width, image.height,
                                  AVPixelFormat(sw_frm_->format),
                                  SWS_FAST_BILINEAR, nullptr, nullptr, nullptr),
                   sws_freeContext);
        last_pixel_format_ = av_format;
    }
    sws_scale(sws_.get(), input_data, input_linesize, 0, image.height,
              sw_frm_->data, sw_frm_->linesize);

    AVFrame* encoder_input = sw_frm_.get();
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    if (is_vaapi_)
    {
        av_hwframe_transfer_data(hw_frm_.get(), sw_frm_.get(), 0);
        encoder_input = hw_frm_.get();
    }
#endif
    ros::Duration d = image.header.stamp - first_ts_;
    std::int64_t pts = static_cast<std::int64_t>(300 * d.toSec());
    if (pts <= last_pts_)
        pts = last_pts_ + 1;
    encoder_input->pts = pts;
    encoder_input->display_picture_number = picture_number_++;
    last_pts_ = pts;
    if ((result = avcodec_send_frame(ctx_.get(), encoder_input)) != 0)
    {
        char errbuf[80];
        throw EncodingError(
            (boost::format("failed to send video frame to encoder: %1%")
             % av_make_error_string(errbuf, sizeof(errbuf), result))
                .str());
    }
    std::size_t count = 0;
    while ((result = avcodec_receive_packet(ctx_.get(), pkt_.get())) == 0)
    {
        if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
        {
            count +=
                splitNALs(packets_, pkt_->data, pkt_->size, image.header.stamp);
        }
        else
        {
            count++;
            FrameDataPtr data = std::make_shared<FrameData>(
                pkt_->data, pkt_->size, image.header.stamp);
            packets_.push_back(data);
        }
    }
    av_packet_unref(pkt_.get());
    if (result != AVERROR(EAGAIN))
    {
        char errbuf[80];
        throw EncodingError(
            (boost::format("failed to receive packets from encoder: %1%")
             % av_make_error_string(errbuf, sizeof(errbuf), result))
                .str());
    }
    return count;
}

FrameDataPtr StreamEncoder::nextPacket() noexcept
{
    if (packets_.empty())
        return FrameDataPtr();
    FrameDataPtr data = packets_.front();
    packets_.pop_front();
    return data;
}

VideoCodec StreamEncoder::codec() const noexcept
{
    return codec_;
}

AVCodecContext* StreamEncoder::context() noexcept
{
    return ctx_.get();
}

}  // namespace rtsp_image_transport
