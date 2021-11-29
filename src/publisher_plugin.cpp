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
#include "publisher_plugin.h"

#include "init.h"
#include "stream_encoder.h"
#include "stream_server.h"
#include "video_codec.h"

#include <pluginlib/class_list_macros.h>

namespace rtsp_image_transport
{

static_assert(static_cast<int>(VideoCodec::H264) == RTSPPublisher_H264);
static_assert(static_cast<int>(VideoCodec::H265) == RTSPPublisher_H265);
static_assert(static_cast<int>(VideoCodec::MPEG4) == RTSPPublisher_MPEG4);
static_assert(static_cast<int>(VideoCodec::VP8) == RTSPPublisher_VP8);
static_assert(static_cast<int>(VideoCodec::VP9) == RTSPPublisher_VP9);

using SuperClass = image_transport::SimplePublisherPlugin<std_msgs::String>;

PublisherPlugin::PublisherPlugin()
    : SuperClass(), update_url_(false), failed_(false)
{
    global_initialize();
}

void PublisherPlugin::shutdown()
{
    SuperClass::shutdown();
    config_server_.reset();
    server_.reset();
    encoder_.reset();
}

std::string PublisherPlugin::getTransportName() const
{
    return "rtsp";
}

void PublisherPlugin::advertiseImpl(
    ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
    const image_transport::SubscriberStatusCallback& user_connect_cb,
    const image_transport::SubscriberStatusCallback& user_disconnect_cb,
    const ros::VoidPtr& tracked_object, bool latch)
{
    SuperClass::advertiseImpl(nh, base_topic, queue_size, user_connect_cb,
                              user_disconnect_cb, tracked_object, true);
    topic_name_ = base_topic;
    try
    {
        config_server_ = std::make_shared<ConfigServer>(this->nh());
        ConfigServer::CallbackType cb =
            std::bind(&PublisherPlugin::configUpdate, this,
                      std::placeholders::_1, std::placeholders::_2);
        config_server_->setCallback(cb);
    }
    catch (std::exception& e)
    {
        server_.reset();
        encoder_.reset();
        ROS_ERROR_STREAM("[" << topic_name_ << "] " << e.what());
        update_url_ = false;
        failed_ = true;
    }
}

void PublisherPlugin::publish(const sensor_msgs::Image& image,
                              const PublishFn& publish_fn) const
{
    try
    {
        if (!server_ || failed_)
            return;
        if (update_url_)
        {
            std_msgs::String url;
            url.data = server_->url();
            publish_fn(url);
            update_url_ = false;
        }
        if (!server_->hasActiveStreams())
        {
            // We need to call publish_fn at least once more after
            // the last client has disconnected, or the ROS topic networking
            // code may not notice that all clients have vanished.
            // As a beneficial side effect, if the RTP streams have timed out
            // for a different reason, the new URL message will cause all
            // remaining active clients to reconnect to the RTSP server.
            //
            // Note that this block will never execute if the RTSP server
            // runs in IP multicast mode, as the corresponding media session
            // will always be active and transmitting.
            if (encoder_)
            {
                ROS_INFO_STREAM("[" << topic_name_ << "] stop encoding for "
                                    << server_->url());
                encoder_.reset();
                std_msgs::String url;
                url.data = server_->url();
                publish_fn(url);
            }
            return;
        }
        if (!encoder_)
        {
            encoder_ = std::make_shared<StreamEncoder>(
                static_cast<VideoCodec>(config_.codec), config_.use_hw_encoder);
            encoder_->setBitrate(1000 * config_.bit_rate);
            encoder_->setFramerate(config_.frame_rate);
            encoder_->setPackageSizeHint(server_->maxPacketSize() - 24);
            ROS_INFO_STREAM("[" << topic_name_ << "] start encoding ("
                                << encoder_->context()->codec->name << "; "
                                << config_.bit_rate << " kbit/s; "
                                << config_.frame_rate << " fps) for "
                                << server_->url());
        }
        if (image.header.stamp.isZero())
            ROS_WARN_STREAM(
                "image header time stamp is not set, expect broken RTSP "
                "stream");
        if (encoder_->encodeVideo(image) > 0)
        {
            FrameDataPtr data;
            while (data = encoder_->nextPacket())
                server_->sendFrame(data);
        }
    }
    catch (const std::exception& e)
    {
        encoder_.reset();
        failed_ = true;
        ROS_ERROR_STREAM("[" << topic_name_ << "] " << e.what());
    }
}

void PublisherPlugin::disconnectCallback(const ros::SingleSubscriberPublisher&)
{
    if (getNumSubscribers() == 0 && encoder_ && server_)
    {
        ROS_INFO_STREAM("[" << topic_name_ << "] stop encoding for "
                            << server_->url());
        encoder_.reset();
    }
}

void PublisherPlugin::configUpdate(RTSPPublisherConfig& cfg, uint32_t level)
{
    config_ = cfg;
    if (!server_)
        level |= 4;
    try
    {
        failed_ = false;
        if (level >= 4)
        {
            server_.reset();
            server_ = StreamServer::create(topic_name_, config_.udp_port,
                                           config_.udp_packet_size - 42);
        }
        if (level >= 2)
        {
            if (level < 4)
                server_->stop();
            server_->start(static_cast<VideoCodec>(config_.codec),
                           config_.use_ip_multicast);
            update_url_ = true;
        }
        if (level >= 1)
        {
            encoder_.reset();
        }
    }
    catch (const std::exception& e)
    {
        server_.reset();
        encoder_.reset();
        update_url_ = false;
        failed_ = true;
        ROS_ERROR_STREAM("[" << topic_name_ << "] " << e.what());
    }
}

}  // namespace rtsp_image_transport

PLUGINLIB_EXPORT_CLASS(rtsp_image_transport::PublisherPlugin,
                       image_transport::PublisherPlugin)
