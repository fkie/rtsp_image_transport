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
#include "init.h"

#include <GroupsockHelper.hh>
#include <netdb.h>
#include <ros/console.h>

#include <mutex>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/log.h>
}

namespace rtsp_image_transport
{

namespace
{

void ffmpeg_log_to_ros(void* avcl, int level, const char* fmt, va_list ap)
{
    if (level > av_log_get_level())
        return;
    char buf[256];
    const char* class_name = "misc";
    if (avcl)
        class_name = (*static_cast<AVClass**>(avcl))->class_name;
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    if (len <= 0)
        return;
    if (len >= static_cast<int>(sizeof(buf)))
        len = sizeof(buf) - 1;
    while (len > 0 && (buf[len - 1] == '\0' || buf[len - 1] == '\n'))
        len--;
    if (len == 0)
        return;
    buf[len] = '\0';
    switch (level)
    {
        case AV_LOG_PANIC:
        case AV_LOG_FATAL:
            ROS_FATAL_NAMED("ffmpeg", "[%s] %s", class_name, buf);
            break;
        case AV_LOG_ERROR:
            ROS_ERROR_NAMED("ffmpeg", "[%s] %s", class_name, buf);
            break;
        case AV_LOG_WARNING:
            ROS_WARN_NAMED("ffmpeg", "[%s] %s", class_name, buf);
            break;
        case AV_LOG_INFO:
            ROS_INFO_NAMED("ffmpeg", "[%s] %s", class_name, buf);
            break;
        default:
            ROS_DEBUG_NAMED("ffmpeg", "[%s] %s", class_name, buf);
            break;
    }
}

int ros_interface_socket_ = -1;

int create_local_socket(netAddressBits addr)
{
    struct sockaddr_in sa;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
        return -1;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = addr;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&sa), sizeof(sa)) < 0)
    {
        close(fd);
        return -1;
    }
    return fd;
}

void do_global_initialize()
{
#if LIBAVCODEC_VERSION_MAJOR < 58
    /* These functions are deprecated since FFmpeg 4.0 */
    av_register_all();
    avcodec_register_all();
#endif
    av_log_set_callback(ffmpeg_log_to_ros);
    av_log_set_level(AV_LOG_ERROR);
    const char* var;
    if ((var = getenv("ROS_HOSTNAME")))
    {
        struct hostent* he = gethostbyname(var);
        if (he && he->h_addr_list && he->h_addr_list[0]
            && he->h_addrtype == AF_INET)
        {
            netAddressBits addr =
                *reinterpret_cast<const netAddressBits*>(he->h_addr_list[0]);
            ros_interface_socket_ = create_local_socket(addr);
        }
    }
    else if ((var = getenv("ROS_IP")))
    {
        ros_interface_socket_ = create_local_socket(our_inet_addr(var));
    }
    if (ros_interface_socket_ >= 0)
    {
        struct sockaddr_in sa;
        socklen_t slen = sizeof(sockaddr_in);
        getsockname(ros_interface_socket_,
                    reinterpret_cast<struct sockaddr*>(&sa), &slen);
        ROS_INFO(
            "ROS_HOSTNAME/ROS_IP override: RTSP server will advertise IP "
            "address %s",
            inet_ntoa(sa.sin_addr));
    }
}

}  // namespace

std::once_flag global_init;

void global_initialize()
{
    std::call_once(global_init, do_global_initialize);
}

int ros_interface_socket()
{
    return ros_interface_socket_;
}

}  // namespace rtsp_image_transport
