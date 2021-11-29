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
#ifndef RTSP_IMAGE_TRANSPORT_LOG_LEVEL_H_
#define RTSP_IMAGE_TRANSPORT_LOG_LEVEL_H_

extern "C"
{
#include <libavutil/log.h>
}

namespace rtsp_image_transport
{

class TemporaryAvLogLevel
{
public:
    TemporaryAvLogLevel(int level) : old_log_level_(av_log_get_level())
    {
        av_log_set_level(level);
    }
    ~TemporaryAvLogLevel()
    {
        av_log_set_level(old_log_level_);
    }

private:
    int old_log_level_;
};

}  // namespace rtsp_image_transport

#endif