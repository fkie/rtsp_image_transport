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
#ifndef RTSP_IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H_
#define RTSP_IMAGE_TRANSPORT_PUBLISHER_PLUGIN_H_

#include "rtsp_image_transport_export.h"

#include <dynamic_reconfigure/server.h>
#include <image_transport/simple_publisher_plugin.h>
#include <rtsp_image_transport/RTSPPublisherConfig.h>
#include <std_msgs/String.h>

namespace rtsp_image_transport
{

class StreamEncoder;
class StreamServer;

class RTSP_IMAGE_TRANSPORT_EXPORT PublisherPlugin
    : public image_transport::SimplePublisherPlugin<std_msgs::String>
{
public:
    PublisherPlugin();
    void shutdown() override;

    std::string getTransportName() const override;

protected:
    void advertiseImpl(
        ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
        const image_transport::SubscriberStatusCallback& user_connect_cb,
        const image_transport::SubscriberStatusCallback& user_disconnect_cb,
        const ros::VoidPtr& tracked_object, bool latch) override;
    void publish(const sensor_msgs::Image& image,
                 const PublishFn& publish_fn) const override;
    void disconnectCallback(const ros::SingleSubscriberPublisher& pub) override;

private:
    using ConfigServer = dynamic_reconfigure::Server<RTSPPublisherConfig>;

    void configUpdate(RTSPPublisherConfig& cfg, uint32_t level);

    std::string topic_name_;
    std::shared_ptr<StreamServer> server_;
    RTSPPublisherConfig config_;
    std::shared_ptr<ConfigServer> config_server_;
    mutable std::shared_ptr<StreamEncoder> encoder_;
    mutable bool update_url_, failed_;
};

}  // namespace rtsp_image_transport

#endif
