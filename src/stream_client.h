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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_CLIENT_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_CLIENT_H_

#include "frame_data.h"
#include "streaming_error.h"
#include "video_codec.h"

#include <BasicUsageEnvironment.hh>
#include <liveMedia.hh>

#include <chrono>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace rtsp_image_transport
{

class Live555Client;
class FrameExtractor;

class StreamClient : public std::enable_shared_from_this<StreamClient>
{
    friend class Live555Client;
    friend class FrameExtractor;

public:
    using SubsessionStartedHandler =
        std::function<void(VideoCodec codec, MediaSubsession* subsession)>;
    using SubsessionFinishedHandler = std::function<void(MediaSubsession*)>;
    using SessionFailedHandler =
        std::function<void(int code, const std::string& message)>;
    using SessionReadyHandler = std::function<void()>;
    using SessionStartedHandler = std::function<void()>;
    using SessionFinishedHandler = std::function<void()>;
    using SessionTimeoutHandler = std::function<void()>;
    using ReceiveStreamDataHandler = std::function<void(
        VideoCodec codec, MediaSubsession* subsession, const FrameDataPtr&)>;

    ~StreamClient();
    StreamClient(const StreamClient&) = delete;
    StreamClient(StreamClient&&) = delete;
    StreamClient& operator=(const StreamClient&) = delete;
    StreamClient& operator=(StreamClient&&) = delete;
    VideoCodec codec() const noexcept;
    std::string url() const noexcept;
    std::string topicName() const noexcept;
    void connect();
    void disconnect();
    void setSessionTimeout(const std::chrono::milliseconds& timeout) noexcept;
    void setSubsessionStartedHandler(SubsessionStartedHandler handler) noexcept;
    void
    setSubsessionFinishedHandler(SubsessionFinishedHandler handler) noexcept;
    void setSessionFailedHandler(SessionFailedHandler handler) noexcept;
    void setSessionReadyHandler(SessionReadyHandler handler) noexcept;
    void setSessionStartedHandler(SessionStartedHandler handler) noexcept;
    void setSessionFinishedHandler(SessionFinishedHandler handler) noexcept;
    void setSessionTimeoutHandler(SessionTimeoutHandler handler) noexcept;
    void setReceiveStreamDataHandler(ReceiveStreamDataHandler handler) noexcept;

    static std::shared_ptr<StreamClient>
    create(const std::string& topic_name, const std::string& url) noexcept;

protected:
    void subsessionStarted(VideoCodec codec, MediaSubsession* subsession);
    void subsessionFinished(MediaSubsession* subsession);
    void sessionFailed(int code, const std::string& message);
    void sessionReady();
    void sessionStarted();
    void sessionFinished();
    void sessionTimeout();
    void receiveStreamData(VideoCodec codec, MediaSubsession* subsession,
                           const FrameDataPtr& data);

private:
    StreamClient(const std::string& topic_name,
                 const std::string& url) noexcept;

    std::string topic_name_, url_;
    VideoCodec codec_;
    char quit_flag_;
    bool retried_on_454_error_;
    std::chrono::milliseconds timeout_;
    std::mutex client_mutex_;

    SubsessionStartedHandler subsession_started_handler_;
    SubsessionFinishedHandler subsession_finished_handler_;
    SessionFailedHandler session_failed_handler_;
    SessionReadyHandler session_ready_handler_;
    SessionStartedHandler session_started_handler_;
    SessionFinishedHandler session_finished_handler_;
    SessionTimeoutHandler session_timeout_handler_;
    ReceiveStreamDataHandler receive_stream_data_handler_;
    /* The order of the following member variables is important,
       because they are interdependent and need to be constructed/
       destroyed in this particular order. */
    std::shared_ptr<TaskScheduler> scheduler_;
    std::shared_ptr<UsageEnvironment> env_;
    std::thread event_loop_thread_;
    Live555Client* client_;
};

}  // namespace rtsp_image_transport

#endif
