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
#ifndef RTSP_IMAGE_TRANSPORT_FRAME_DATA_H_
#define RTSP_IMAGE_TRANSPORT_FRAME_DATA_H_

#include <ros/time.h>

#include <cstdint>
#include <memory>

namespace rtsp_image_transport
{

class FrameData
{
public:
    FrameData(const unsigned char* data, std::size_t length,
              const ros::Time& stamp);
    ~FrameData();
    FrameData(const FrameData&) = delete;
    FrameData& operator=(const FrameData&) = delete;
    const unsigned char* data() const
    {
        return data_;
    }
    std::size_t length() const
    {
        return length_;
    }
    const ros::Time& stamp() const
    {
        return stamp_;
    }
    void setStamp(const ros::Time& stamp)
    {
        stamp_ = stamp;
    }

private:
    unsigned char* data_;
    std::size_t length_;
    ros::Time stamp_;
};

using FrameDataPtr = std::shared_ptr<FrameData>;

}  // namespace rtsp_image_transport

#endif
