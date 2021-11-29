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
#include "frame_extractor.h"

#include "frame_data.h"
#include "stream_client.h"

#include <boost/format.hpp>
#include <ros/console.h>

namespace rtsp_image_transport
{

namespace
{

const unsigned char MPEG_START_CODE[] = {0x00, 0x00, 0x00, 0x01};

}

FrameExtractor*
FrameExtractor::createNew(const std::weak_ptr<StreamClient>& stream_client,
                          UsageEnvironment& env, MediaSubsession* subsession)
{
    return new FrameExtractor(stream_client, env, subsession);
}

FrameExtractor::FrameExtractor(const std::weak_ptr<StreamClient>& stream_client,
                               UsageEnvironment& env,
                               MediaSubsession* subsession)
    : MediaSink(env), stream_client_(stream_client), subsession_(subsession),
      codec_(fromRTSPCodecName(subsession_->codecName())), buffer_length_(0)
{
    if (codec_ == VideoCodec::Unknown)
        throw StreamingError((boost::format("unsupported video codec %1%")
                              % subsession->codecName())
                                 .str());
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        /* Pass out-of-band PPS and SPS NAL units */
        char const* sprops = subsession->fmtp_spropparametersets();
        if (sprops)
        {
            std::shared_ptr<StreamClient> sc = stream_client_.lock();
            if (sc)
            {
                unsigned num_nals;
                SPropRecord* nals = parseSPropParameterSets(sprops, num_nals);
                for (unsigned i = 0; i < num_nals; ++i)
                {
                    if (buffer_length_ + nals[i].sPropLength + 4
                        > buffer_.size())
                        throw StreamingError("frame buffer overflow");
                    std::copy_n(MPEG_START_CODE, sizeof(MPEG_START_CODE),
                                buffer_.data() + buffer_length_);
                    std::copy_n(nals[i].sPropBytes, nals[i].sPropLength,
                                buffer_.data() + buffer_length_
                                    + sizeof(MPEG_START_CODE));
                    buffer_length_ +=
                        nals[i].sPropLength + sizeof(MPEG_START_CODE);
                }
                delete[] nals;
            }
        }
    }
}

VideoCodec FrameExtractor::codec() const
{
    return codec_;
}

Boolean FrameExtractor::continuePlaying()
{
    if (codec_ == VideoCodec::H264 || codec_ == VideoCodec::H265)
    {
        if (buffer_.size() - sizeof(MPEG_START_CODE) >= buffer_length_)
        {
            std::copy_n(MPEG_START_CODE, sizeof(MPEG_START_CODE),
                        buffer_.data() + buffer_length_);
            buffer_length_ += sizeof(MPEG_START_CODE);
        }
    }
    fSource->getNextFrame(buffer_.data() + buffer_length_,
                          buffer_.size() - buffer_length_, newFrameCallback,
                          this, onSourceClosure, this);
    return True;
}

void FrameExtractor::deliverFrame(unsigned frameSize,
                                  unsigned numTruncatedBytes,
                                  struct timeval presentationTime,
                                  unsigned durationInMicroseconds)
{
    std::shared_ptr<StreamClient> sc = stream_client_.lock();
    if (sc)
    {
        if (numTruncatedBytes)
        {
            ROS_WARN_STREAM("FrameExtractor buffer is " << numTruncatedBytes
                                                        << " bytes too small");
        }
        buffer_length_ += frameSize;
        ros::Time ts(presentationTime.tv_sec,
                     1000ull * presentationTime.tv_usec);
        if (buffer_length_ > 48)
        {
            sc->receiveStreamData(codec_, subsession_,
                                  std::make_shared<FrameData>(
                                      buffer_.data(), buffer_length_, ts));
            buffer_length_ = 0;
        }
        continuePlaying();
    }
}

void FrameExtractor::newFrameCallback(void* obj, unsigned frameSize,
                                      unsigned numTruncatedBytes,
                                      struct timeval presentationTime,
                                      unsigned durationInMicroseconds)
{
    FrameExtractor* self = static_cast<FrameExtractor*>(obj);
    self->deliverFrame(frameSize, numTruncatedBytes, presentationTime,
                       durationInMicroseconds);
}

}  // namespace rtsp_image_transport
