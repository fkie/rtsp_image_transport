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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

extern char _binary_rtsp_only_png_start;
extern char _binary_rtsp_only_png_end;

int main(int argc, char** argv)
{
    std::string self = argv[0];
    std::string::size_type n = self.rfind('/');
    if (n != std::string::npos)
        self = self.substr(n + 1);

    ros::init(argc, argv, self, ros::init_options::AnonymousName);
    std::string topic = ros::names::remap("image");

    if (argc < 2)
    {
        ROS_ERROR(
            "Missing stream URL. Typical command line usage:\n"
            "  $ rosrun " ROS_PACKAGE_NAME
            " %s image:=<image topic> rtsp://url",
            self.c_str());
        return 2;
    }
    if (topic == "image")
    {
        ROS_WARN(
            "Topic 'image' has not been remapped! Typical command line usage:\n"
            "  $ rosrun " ROS_PACKAGE_NAME
            " %s image:=<image topic> rtsp://url",
            self.c_str());
    }
    if (argc > 2)
    {
        ROS_WARN("Extra command line arguments ignored");
    }
    std_msgs::String url;
    url.data = argv[1];
    if (url.data.substr(0, 7) != "rtsp://")
    {
        ROS_WARN("URL does not begin with rtsp://");
    }
    ros::NodeHandle nh;
    int png_size = &_binary_rtsp_only_png_end - &_binary_rtsp_only_png_start;
    const cv::Mat rtsp_only_mat =
        cv::imdecode(cv::Mat(1, png_size, CV_8U, &_binary_rtsp_only_png_start),
                     cv::IMREAD_COLOR);
    sensor_msgs::ImagePtr rtsp_only_img =
        cv_bridge::CvImage(std_msgs::Header(),
                           sensor_msgs::image_encodings::BGR8, rtsp_only_mat)
            .toImageMsg();
    ros::Publisher pub0 = nh.advertise<sensor_msgs::Image>(topic, 1, true);
    pub0.publish(rtsp_only_img);
    ros::Publisher pub =
        nh.advertise<std_msgs::String>(topic + "/rtsp", 1, true);
    pub.publish(url);
    ros::spin();
}
