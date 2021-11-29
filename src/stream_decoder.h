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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_DECODER_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_DECODER_H_

#include "frame_data.h"
#include "streaming_error.h"
#include "video_codec.h"

extern "C"
{
#include <libswscale/swscale.h>
}

#include <sensor_msgs/Image.h>

#include <cstdint>
#include <deque>
#include <memory>

namespace rtsp_image_transport
{

class StreamDecoder
{
public:
    enum class DecodeFrames
    {
        All,
        Intra,
        Key,
        None
    };
    StreamDecoder(VideoCodec codec, bool use_hw_decoder = true);
    VideoCodec codec() const noexcept;
    std::size_t decodeVideo(const FrameDataPtr& data);
    sensor_msgs::ImageConstPtr nextFrame() noexcept;
    AVCodecContext* context() noexcept;
    void setDecodeFrames(DecodeFrames which) noexcept;

private:
    void setupDecoder(AVCodec* decoder);

    VideoCodec codec_;
    bool initialized_;
    int width_, height_;
    AVPixelFormat last_pixel_format_;
    std::shared_ptr<AVCodecContext> ctx_;
    std::shared_ptr<AVPacket> pkt_;
    std::shared_ptr<AVFrame> frm_;
    std::shared_ptr<SwsContext> sws_;
    std::deque<sensor_msgs::ImageConstPtr> frames_;
};

}  // namespace rtsp_image_transport

#endif
