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
#ifndef RTSP_IMAGE_TRANSPORT_STREAMING_ERROR_H_
#define RTSP_IMAGE_TRANSPORT_STREAMING_ERROR_H_

#include <stdexcept>

namespace rtsp_image_transport
{

class StreamingError : public std::runtime_error
{
public:
    StreamingError(const std::string& what) : std::runtime_error(what.c_str())
    {
    }
};

class EncodingError : public StreamingError
{
    using StreamingError::StreamingError;
};

class DecodingError : public StreamingError
{
    using StreamingError::StreamingError;
};

}  // namespace rtsp_image_transport

#endif
