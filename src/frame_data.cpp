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
#include "frame_data.h"

#include <algorithm>

extern "C"
{
#include <libavcodec/avcodec.h>
}

namespace rtsp_image_transport
{

FrameData::FrameData(const unsigned char* data, std::size_t length,
                     const ros::Time& stamp)
    : data_(new unsigned char[length + AV_INPUT_BUFFER_PADDING_SIZE]),
      length_(length), stamp_(stamp)
{
    std::copy_n(data, length, data_);
    std::fill_n(data_ + length, AV_INPUT_BUFFER_PADDING_SIZE, 0);
}

FrameData::~FrameData()
{
    delete[] data_;
}

}  // namespace rtsp_image_transport