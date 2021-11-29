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
#include "video_codec.h"

#include <vector>

namespace rtsp_image_transport
{

std::string videoCodecName(VideoCodec codec)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return "H.264";
        case VideoCodec::H265:
            return "H.265";
        case VideoCodec::MPEG4:
            return "MPEG-4";
        case VideoCodec::VP8:
            return "VP8";
        case VideoCodec::VP9:
            return "VP9";
        case VideoCodec::MJPEG:
            return "Motion JPEG";
        default:
            return "unsupported codec";
    }
}

VideoCodec fromRTSPCodecName(const std::string& name)
{
    if (name == "H264")
        return VideoCodec::H264;
    if (name == "H265")
        return VideoCodec::H265;
    if (name == "MP4V-ES")
        return VideoCodec::MPEG4;
    if (name == "VP8")
        return VideoCodec::VP8;
    if (name == "VP9")
        return VideoCodec::VP9;
    if (name == "JPEG")
        return VideoCodec::MJPEG;
    return VideoCodec::Unknown;
}

}  // namespace rtsp_image_transport
