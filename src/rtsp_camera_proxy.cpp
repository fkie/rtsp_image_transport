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
#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>
#include <cv_bridge/cv_bridge.h>
#include <liveMedia.hh>
#include <netdb.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <functional>
#include <thread>

extern char _binary_rtsp_only_png_start;
extern char _binary_rtsp_only_png_end;

static int create_local_socket(netAddressBits addr)
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rtsp_camera_proxy");
    int ros_sock = -1;
    const char* var;
    if ((var = getenv("ROS_HOSTNAME")))
    {
        struct hostent* he = gethostbyname(var);
        if (he && he->h_addr_list && he->h_addr_list[0]
            && he->h_addrtype == AF_INET)
        {
            netAddressBits addr =
                *reinterpret_cast<const netAddressBits*>(he->h_addr_list[0]);
            ros_sock = create_local_socket(addr);
        }
    }
    else if ((var = getenv("ROS_IP")))
    {
        ros_sock = create_local_socket(our_inet_addr(var));
    }
    if (ros_sock >= 0)
    {
        struct sockaddr_in sa;
        socklen_t slen = sizeof(sockaddr_in);
        getsockname(ros_sock, reinterpret_cast<struct sockaddr*>(&sa), &slen);
        ROS_INFO(
            "ROS_HOSTNAME/ROS_IP override: RTSP proxy will advertise IP "
            "address %s",
            inet_ntoa(sa.sin_addr));
    }
    int png_size = &_binary_rtsp_only_png_end - &_binary_rtsp_only_png_start;
    const cv::Mat rtsp_only_mat =
        cv::imdecode(cv::Mat(1, png_size, CV_8U, &_binary_rtsp_only_png_start),
                     cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr rtsp_only_img =
        cv_bridge::CvImage(std_msgs::Header(),
                           sensor_msgs::image_encodings::BGR8, rtsp_only_mat)
            .toImageMsg();
    OutPacketBuffer::maxSize = 500000;
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);

    ros::NodeHandle nh("~");
    int server_port, verbose, mtu;
    bool tcp;
    std::vector<ros::Publisher> camera_pub_raw, camera_pub_rtsp;
    nh.param<int>("server_port", server_port, 0);
    nh.param<bool>("tcp", tcp, false);
    nh.param<int>("verbose", verbose, 0);
    nh.param<int>("mtu", mtu, 1396);
    RTSPServer* rtspServer = RTSPServer::createNew(*env, server_port);
    if (!rtspServer)
    {
        ROS_FATAL_STREAM(
            "Failed to setup RTSP server instance: " << env->getResultMsg());
        return 1;
    }
    for (int id = 1;; id++)
    {
        std::stringstream s;
        std::string camera_basename, camera_topic, camera_uri_param, camera_uri;
        s << "camera" << id;
        camera_basename = s.str();
        camera_topic = nh.resolveName(camera_basename);
        camera_uri_param = camera_basename + "_uri";

        if (nh.getParam(camera_uri_param, camera_uri))
        {
            std::string camera_tcp_param, camera_tos_param, camera_mtu_param;
            bool camera_tcp;
            int camera_mtu;
            camera_tcp_param = camera_basename + "_tcp";
            camera_mtu_param = camera_basename + "_mtu";
            nh.param<bool>(camera_tcp_param, camera_tcp, tcp);
            nh.param<int>(camera_mtu_param, camera_mtu, mtu);
            portNumBits http_tunnel_port = camera_tcp ? 0xFFFF : 0;
            ProxyServerMediaSession* sms = ProxyServerMediaSession::createNew(
                *env, rtspServer, camera_uri.c_str(), camera_basename.c_str(),
                nullptr, nullptr, http_tunnel_port);
            rtspServer->addServerMediaSession(sms);
            char* proxyURL = rtspServer->rtspURL(sms, ros_sock);
            std_msgs::String url_msg;
            url_msg.data = proxyURL;
            delete[] proxyURL;
            camera_pub_raw.push_back(
                nh.advertise<sensor_msgs::Image>(camera_topic, 1, true));
            camera_pub_raw.back().publish(rtsp_only_img);
            camera_pub_rtsp.push_back(nh.advertise<std_msgs::String>(
                camera_topic + "/rtsp", 1, true));
            camera_pub_rtsp.back().publish(url_msg);
            ROS_INFO_STREAM(camera_basename << ": " << camera_uri << " via "
                                            << url_msg.data);
        }
        else
        {
            break;
        }
    }
    if (camera_pub_raw.empty())
    {
        ROS_FATAL("No camera URIs configured");
        return 1;
    }
    char shutdown = 0;
    std::thread t([&shutdown, &env]()
                  { env->taskScheduler().doEventLoop(&shutdown); });
    ros::spin();
    shutdown = 1;
    t.join();
    return 0;
}
