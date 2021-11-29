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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_ENCODER_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_ENCODER_H_

#include "frame_data.h"
#include "streaming_error.h"
#include "video_codec.h"

#include <sensor_msgs/Image.h>

#include <cstdint>
#include <deque>
#include <memory>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

namespace rtsp_image_transport
{

class StreamEncoder
{
public:
    StreamEncoder(VideoCodec codec, bool use_hw_encoder = true);
    void setBitrate(unsigned long bit_rate);
    void setFramerate(unsigned fps);
    void setPackageSizeHint(unsigned size);
    bool hwAccel() const noexcept;
    VideoCodec codec() const noexcept;
    std::size_t encodeVideo(const sensor_msgs::Image& image);
    FrameDataPtr nextPacket() noexcept;
    AVCodecContext* context() noexcept;

private:
    void setupEncoder(AVCodec* encoder, bool silent);
    void openEncoder(int width, int height);

    VideoCodec codec_;
    bool initialized_, is_vaapi_;
    std::shared_ptr<AVCodecContext> ctx_;
#ifdef FFMPEG_HAS_HWFRAME_SUPPORT
    std::shared_ptr<AVBufferRef> hw_device_, hw_frames_;
    AVHWDeviceContext* hw_device_ctx_;
    AVHWFramesContext* hw_frames_ctx_;
    std::shared_ptr<AVFrame> hw_frm_;
#endif
    std::shared_ptr<AVFrame> sw_frm_;
    std::shared_ptr<AVPacket> pkt_;
    AVPixelFormat last_pixel_format_;
    std::shared_ptr<SwsContext> sws_;
    ros::Time first_ts_;
    std::int64_t last_pts_;
    int picture_number_;
    std::deque<FrameDataPtr> packets_;
};

}  // namespace rtsp_image_transport

#endif