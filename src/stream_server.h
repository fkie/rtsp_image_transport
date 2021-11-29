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
#ifndef RTSP_IMAGE_TRANSPORT_STREAM_SERVER_H_
#define RTSP_IMAGE_TRANSPORT_STREAM_SERVER_H_

#include "frame_data.h"
#include "frame_injector.h"
#include "streaming_error.h"
#include "video_codec.h"

#include <BasicUsageEnvironment.hh>
#include <GroupsockHelper.hh>
#include <liveMedia.hh>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace rtsp_image_transport
{

class UnicastServerMediaSubsession;

class StreamServer : public std::enable_shared_from_this<StreamServer>
{
    friend class UnicastServerMediaSubsession;

public:
    ~StreamServer();
    StreamServer(const StreamServer&) = delete;
    StreamServer(StreamServer&&) = delete;
    StreamServer& operator=(const StreamServer&) = delete;
    StreamServer& operator=(StreamServer&&) = delete;
    void sendFrame(const FrameDataPtr& frame) noexcept;
    bool hasActiveStreams() const noexcept;
    VideoCodec codec() const noexcept;
    unsigned preferredPacketSize() const noexcept;
    unsigned maxPacketSize() const noexcept;
    std::string url() const noexcept;
    void start(VideoCodec codec, bool use_multicast);
    void stop();
    static std::shared_ptr<StreamServer> create(const std::string& topic_name,
                                                unsigned udp_port,
                                                unsigned udp_packet_size);

protected:
    void newStreamSource(FramedSource* source,
                         FrameInjector* injector) noexcept;
    void closeStreamSource(FramedSource* source) noexcept;
    RTPSink* activeSinkForSDP();

private:
    using StreamMapping = std::map<FramedSource*, FrameInjector*>;

    StreamServer(const std::string& topic_name, unsigned udp_port,
                 unsigned udp_packet_size);

    VideoCodec codec_;
    std::string topic_name_;
    char quit_flag_;
    unsigned udp_packet_size_;
    std::string url_;
    /* The order of the following member variables is important,
       because they are interdependent and need to be constructed/
       destroyed in this particular order. */
    mutable std::mutex streams_mutex_;
    StreamMapping streams_;
    std::thread event_loop_thread_;
    std::shared_ptr<TaskScheduler> scheduler_;
    std::shared_ptr<UsageEnvironment> env_;
    std::shared_ptr<Groupsock> rtp_mcast_, rtcp_mcast_;
    RTSPServer* rtsp_;
    ServerMediaSession* sms_;
    VideoRTPSink* sink_;
};

}  // namespace rtsp_image_transport

#endif
