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
#ifndef RTSP_IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H_
#define RTSP_IMAGE_TRANSPORT_SUBSCRIBER_PLUGIN_H_

#include "frame_data.h"
#include "rtsp_image_transport_export.h"
#include "video_codec.h"

#include <dynamic_reconfigure/server.h>
#include <image_transport/simple_subscriber_plugin.h>
#include <rtsp_image_transport/RTSPSubscriberConfig.h>
#include <std_msgs/String.h>

#include <deque>
#include <mutex>

class MediaSubsession;

namespace rtsp_image_transport
{

class StreamClient;
class StreamDecoder;

class RTSP_IMAGE_TRANSPORT_EXPORT SubscriberPlugin
    : public image_transport::SimpleSubscriberPlugin<std_msgs::String>
{
public:
    SubscriberPlugin();
    void shutdown() override;
    std::string getTransportName() const override;

protected:
    void subscribeImpl(
        ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
        const Callback& callback, const ros::VoidPtr& tracked_object,
        const image_transport::TransportHints& transport_hints) override;
    void internalCallback(const std_msgs::String::ConstPtr& message,
                          const Callback& callback) override;

private:
    using ConfigServer = dynamic_reconfigure::Server<RTSPSubscriberConfig>;

    void configUpdate(RTSPSubscriberConfig& cfg, uint32_t level);
    void receiveDataStream(VideoCodec codec, MediaSubsession* subsession,
                           const FrameDataPtr& data);
    void subsessionStarted(VideoCodec codec, MediaSubsession* subsession);
    void sessionFailed(int code, const std::string& message);
    void sessionStarted();
    void sessionFinished();
    void sessionTimeout();
    void processFrame();
    void reconnect();
    void cooldownTimerCallback(const ros::WallTimerEvent& event);

    void pushFrame(const FrameDataPtr& frame);
    FrameDataPtr popFrame();
    ros::Duration frameLag() const noexcept;
    void clearQueuedFrames();

    std::shared_ptr<ConfigServer> config_server_;
    std::string topic_name_;
    bool failed_;
    double cooldown_;
    ros::Duration old_lag_;
    ros::WallTimer cooldown_timer_;
    std::shared_ptr<ros::NodeHandle> nh_;
    Callback callback_;
    std::shared_ptr<StreamClient> client_;
    std::shared_ptr<StreamDecoder> decoder_;
    RTSPSubscriberConfig config_;

    mutable std::mutex queue_mutex_;
    std::deque<FrameDataPtr> queue_;
};

}  // namespace rtsp_image_transport

#endif
