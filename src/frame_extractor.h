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
#ifndef RTSP_IMAGE_TRANSPORT_FRAME_EXTRACTOR_H_
#define RTSP_IMAGE_TRANSPORT_FRAME_EXTRACTOR_H_

#include "streaming_error.h"
#include "video_codec.h"

#include <liveMedia.hh>
#include <ros/time.h>

#include <array>
#include <cstdint>
#include <memory>

namespace rtsp_image_transport
{

class StreamClient;

class FrameExtractor : public MediaSink
{
public:
    static FrameExtractor*
    createNew(const std::weak_ptr<StreamClient>& stream_client,
              UsageEnvironment& env, MediaSubsession* subsession);
    VideoCodec codec() const;

protected:
    Boolean continuePlaying();

private:
    FrameExtractor(const std::weak_ptr<StreamClient>& stream_client,
                   UsageEnvironment& env, MediaSubsession* subsession);

    static void newFrameCallback(void* self, unsigned frameSize,
                                 unsigned numTruncatedBytes,
                                 struct timeval presentationTime,
                                 unsigned durationInMicroseconds);
    void deliverFrame(unsigned frameSize, unsigned numTruncatedBytes,
                      struct timeval presentationTime,
                      unsigned durationInMicroseconds);

    std::weak_ptr<StreamClient> stream_client_;
    MediaSubsession* subsession_;
    VideoCodec codec_;
    std::array<unsigned char, 131072> buffer_;
    std::size_t buffer_length_;
};

}  // namespace rtsp_image_transport

#endif