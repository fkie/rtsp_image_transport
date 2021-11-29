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
#include "subscriber_plugin.h"

#include "init.h"
#include "stream_client.h"
#include "stream_decoder.h"

#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue_interface.h>

#include <functional>

namespace rtsp_image_transport
{

namespace
{

class ScheduledCB : public ros::CallbackInterface
{
public:
    using Signature = std::function<void()>;

    ScheduledCB(const Signature& func);
    CallResult call() override;

private:
    Signature func_;
};

ScheduledCB::ScheduledCB(const Signature& func) : func_(func) {}

ScheduledCB::CallResult ScheduledCB::call()
{
    func_();
    return Success;
}

}  // namespace

using SuperClass = image_transport::SimpleSubscriberPlugin<std_msgs::String>;

SubscriberPlugin::SubscriberPlugin() : SuperClass()
{
    global_initialize();
}

void SubscriberPlugin::shutdown()
{
    cooldown_timer_.stop();
    if (nh_)
        nh_->getCallbackQueue()->removeByID(reinterpret_cast<uint64_t>(this));
    if (client_)
        client_->disconnect();
    decoder_.reset();
    config_server_.reset();
    clearQueuedFrames();
    SuperClass::shutdown();
}

std::string SubscriberPlugin::getTransportName() const
{
    return "rtsp";
}

void SubscriberPlugin::subscribeImpl(
    ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
    const Callback& callback, const ros::VoidPtr& tracked_object,
    const image_transport::TransportHints& transport_hints)
{
    SuperClass::subscribeImpl(nh, base_topic, queue_size, callback,
                              tracked_object, transport_hints);
    topic_name_ = base_topic;
    nh_ = std::make_shared<ros::NodeHandle>(nh);
    failed_ = false;
    config_server_ = std::make_shared<ConfigServer>(this->nh());
    ConfigServer::CallbackType cb =
        std::bind(&SubscriberPlugin::configUpdate, this, std::placeholders::_1,
                  std::placeholders::_2);
    config_server_->setCallback(cb);
}

void SubscriberPlugin::internalCallback(const std_msgs::String::ConstPtr& msg,
                                        const Callback& callback)
{
    using namespace std::chrono_literals;

    ROS_DEBUG_STREAM("[" << topic_name_
                         << "] received updated RTSP URL: " << msg->data);
    failed_ = false;
    old_lag_ = ros::Duration(0);
    callback_ = callback;
    cooldown_ = config_.reconnect_minwait;
    cooldown_timer_.stop();
    client_.reset();
    client_ = StreamClient::create(topic_name_, msg->data);
    client_->setSessionTimeout(std::chrono::milliseconds(
        static_cast<std::chrono::milliseconds::rep>(1000 * config_.timeout)));
    client_->setReceiveStreamDataHandler(std::bind(
        &SubscriberPlugin::receiveDataStream, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
    client_->setSessionTimeoutHandler(
        std::bind(&SubscriberPlugin::sessionTimeout, this));
    client_->setSubsessionStartedHandler(
        std::bind(&SubscriberPlugin::subsessionStarted, this,
                  std::placeholders::_1, std::placeholders::_2));
    client_->setSessionStartedHandler(
        std::bind(&SubscriberPlugin::sessionStarted, this));
    client_->setSessionFailedHandler(std::bind(&SubscriberPlugin::sessionFailed,
                                               this, std::placeholders::_1,
                                               std::placeholders::_2));
    client_->setSessionFinishedHandler(
        std::bind(&SubscriberPlugin::sessionFinished, this));
    client_->connect();
}

void SubscriberPlugin::subsessionStarted(VideoCodec codec,
                                         MediaSubsession* subsession)
{
    old_lag_ = ros::Duration(0);
    ROS_DEBUG_STREAM("[" << topic_name_ << "] setting up decoder for "
                         << videoCodecName(codec));
    decoder_ = std::make_shared<StreamDecoder>(codec, config_.use_hw_decoder);
    ROS_INFO_STREAM("[" << topic_name_ << "] start decoding ("
                        << decoder_->context()->codec->name << ") from "
                        << client_->url());
}

void SubscriberPlugin::receiveDataStream(VideoCodec codec,
                                         MediaSubsession* subsession,
                                         const FrameDataPtr& data)
{
    ROS_DEBUG_STREAM_THROTTLE(
        30, "[" << topic_name_ << "] receiving video frames from RTSP stream");
    pushFrame(data);
    ros::CallbackQueueInterface* cbq = nh_->getCallbackQueue();
    cbq->addCallback(boost::make_shared<ScheduledCB>(
                         std::bind(&SubscriberPlugin::processFrame, this)),
                     reinterpret_cast<uint64_t>(this));
}

void SubscriberPlugin::processFrame()
{
    std::shared_ptr<StreamDecoder> decoder = decoder_;
    if (failed_ || !decoder)
        return;
    try
    {
        FrameDataPtr frame = popFrame();
        ros::Duration lag = frameLag();
        if (lag >= ros::Duration(2))
        {
            if (old_lag_ < ros::Duration(2))
            {
                ROS_WARN_STREAM(
                    "[" << topic_name_
                        << "] decoder is too slow; discarding all frames");
                old_lag_ = lag;
            }
            decoder->setDecodeFrames(StreamDecoder::DecodeFrames::None);
        }
        else if (lag >= ros::Duration(1))
        {
            if (old_lag_ < ros::Duration(1))
            {
                ROS_WARN_STREAM(
                    "[" << topic_name_
                        << "] decoder is too slow; discarding non-key frames");
                old_lag_ = lag;
            }
            decoder->setDecodeFrames(StreamDecoder::DecodeFrames::Key);
        }
        if (lag >= ros::Duration(0.5))
        {
            if (old_lag_ < ros::Duration(0.5))
            {
                ROS_WARN_STREAM(
                    "["
                    << topic_name_
                    << "] decoder is too slow; discarding non-intra frames");
                old_lag_ = lag;
            }
            decoder->setDecodeFrames(StreamDecoder::DecodeFrames::Intra);
        }
        else
        {
            decoder->setDecodeFrames(StreamDecoder::DecodeFrames::All);
            if (lag.isZero())
                old_lag_ = ros::Duration(0);
        }
        if (decoder->decodeVideo(frame) > 0)
        {
            while (sensor_msgs::ImageConstPtr img = decoder->nextFrame())
            {
                callback_(img);
            }
        }
    }
    catch (const DecodingError& e)
    {
        ROS_WARN_STREAM("[" << topic_name_ << "] " << e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("[" << topic_name_ << "] " << e.what());
        failed_ = true;
        clearQueuedFrames();
        if (config_.reconnect_policy >= RTSPSubscriber_ReconnectOnFailure)
        {
            reconnect();
        }
    }
}

void SubscriberPlugin::sessionStarted()
{
    cooldown_ = config_.reconnect_minwait;
    cooldown_timer_.stop();
}

void SubscriberPlugin::sessionTimeout()
{
    ROS_ERROR_STREAM("[" << topic_name_ << "] session timeout for stream at "
                         << client_->url());
    if (config_.reconnect_policy >= RTSPSubscriber_ReconnectOnTimeout)
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFailed(int code, const std::string& message)
{
    ROS_ERROR_STREAM("[" << topic_name_ << "] " << client_->url() << " failed. "
                         << message << " (" << code << ")");
    if (config_.reconnect_policy >= RTSPSubscriber_ReconnectOnFailure)
    {
        reconnect();
    }
}

void SubscriberPlugin::sessionFinished()
{
    ROS_INFO_STREAM("[" << topic_name_ << "] end of video stream");
    if (config_.reconnect_policy >= RTSPSubscriber_ReconnectAlways)
    {
        reconnect();
    }
}

void SubscriberPlugin::reconnect()
{
    client_->disconnect();
    clearQueuedFrames();
    ROS_INFO_STREAM("[" << topic_name_ << "] new connection attempt in "
                        << std::fixed << std::setprecision(3) << cooldown_
                        << " seconds");
    cooldown_timer_ =
        nh_->createWallTimer(ros::WallDuration(cooldown_),
                             std::bind(&SubscriberPlugin::cooldownTimerCallback,
                                       this, std::placeholders::_1),
                             true);
    cooldown_ *= 2;
    if (cooldown_ > config_.reconnect_maxwait)
        cooldown_ = config_.reconnect_maxwait;
}

void SubscriberPlugin::cooldownTimerCallback(const ros::WallTimerEvent& event)
{
    cooldown_timer_.stop();  // just in case
    clearQueuedFrames();
    client_->connect();
}

void SubscriberPlugin::pushFrame(const FrameDataPtr& frame)
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    queue_.push_back(frame);
}

FrameDataPtr SubscriberPlugin::popFrame()
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty())
        return FrameDataPtr();
    FrameDataPtr frame = queue_.front();
    queue_.pop_front();
    return frame;
}

ros::Duration SubscriberPlugin::frameLag() const noexcept
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    if (queue_.empty())
        return ros::Duration(0);
    return queue_.back()->stamp() - queue_.front()->stamp();
}

void SubscriberPlugin::clearQueuedFrames()
{
    std::lock_guard<std::mutex> lock{queue_mutex_};
    queue_.clear();
}

void SubscriberPlugin::configUpdate(RTSPSubscriberConfig& cfg, uint32_t level)
{
    config_ = cfg;
    cooldown_ = config_.reconnect_minwait;
    if (config_.reconnect_maxwait < config_.reconnect_minwait)
        config_.reconnect_maxwait = config_.reconnect_minwait;
    if (client_)
    {
        client_->setSessionTimeout(std::chrono::milliseconds(
            static_cast<std::chrono::milliseconds::rep>(1000
                                                        * config_.timeout)));
        if (level >= 1)
        {
            decoder_ = std::make_shared<StreamDecoder>(client_->codec(),
                                                       config_.use_hw_decoder);
            ROS_INFO_STREAM("[" << topic_name_ << "] start decoding ("
                                << decoder_->context()->codec->name << ") from "
                                << client_->url());
        }
    }
}

}  // namespace rtsp_image_transport

PLUGINLIB_EXPORT_CLASS(rtsp_image_transport::SubscriberPlugin,
                       image_transport::SubscriberPlugin)
