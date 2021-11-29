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
#ifndef RTSP_IMAGE_TRANSPORT_VIDEO_CODEC_H_
#define RTSP_IMAGE_TRANSPORT_VIDEO_CODEC_H_

#include <string>

extern "C"
{
#include <libavcodec/avcodec.h>
}

namespace rtsp_image_transport
{

// IMPORTANT: The following enum values 0...4 must match those in
// RTSPPublisher.cfg
enum class VideoCodec
{
    Unknown = -1,
    H264 = 0,
    H265 = 1,
    MPEG4 = 2,
    VP8 = 3,
    VP9 = 4,
    MJPEG = 1001 /* Decoding only */
};

std::string videoCodecName(VideoCodec codec);
VideoCodec fromRTSPCodecName(const std::string& name);
AVCodec* findEncoderFor(VideoCodec codec, bool hwAccel);
AVCodec* findDecoderFor(VideoCodec codec, bool hwAccel);

}  // namespace rtsp_image_transport

#endif
