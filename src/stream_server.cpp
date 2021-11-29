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
#include "stream_server.h"

#include "init.h"

#include <arpa/inet.h>
#include <boost/format.hpp>
#include <boost/scoped_array.hpp>
#include <ros/console.h>

namespace rtsp_image_transport
{

namespace
{

constexpr unsigned ESTIMATED_BITRATE = 500;

uint16_t sockToPort(const in_addr& addr)
{
    return 16384 + (ntohl(addr.s_addr) & 0x7ffe);
}

struct in_addr makeAddr(uint32_t addr)
{
    struct in_addr result;
    result.s_addr = addr;
    return result;
}

VideoRTPSink* createVideoRTPSink(VideoCodec codec, UsageEnvironment& env,
                                 Groupsock* rtpGroupsock,
                                 unsigned char rtpPayloadTypeIfDynamic)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return H264VideoRTPSink::createNew(env, rtpGroupsock,
                                               rtpPayloadTypeIfDynamic);
#ifdef LIVE555_HAS_H265_SUPPORT
        case VideoCodec::H265:
            return H265VideoRTPSink::createNew(env, rtpGroupsock,
                                               rtpPayloadTypeIfDynamic);
#endif
        case VideoCodec::MPEG4:
            return MPEG4ESVideoRTPSink::createNew(env, rtpGroupsock,
                                                  rtpPayloadTypeIfDynamic);
#ifdef LIVE555_HAS_VPX_SUPPORT
        case VideoCodec::VP8:
            return VP8VideoRTPSink::createNew(env, rtpGroupsock,
                                              rtpPayloadTypeIfDynamic);
        case VideoCodec::VP9:
            return VP9VideoRTPSink::createNew(env, rtpGroupsock,
                                              rtpPayloadTypeIfDynamic);
#endif
        default:
            return nullptr;
    }
}

FramedSource* createDiscreteFramer(VideoCodec codec, UsageEnvironment& env,
                                   FramedSource* source)
{
    switch (codec)
    {
        case VideoCodec::H264:
            return H264VideoStreamDiscreteFramer::createNew(env, source);
#ifdef LIVE555_HAS_H265_SUPPORT
        case VideoCodec::H265:
            return H265VideoStreamDiscreteFramer::createNew(env, source);
#endif
        case VideoCodec::MPEG4:
            return MPEG4VideoStreamDiscreteFramer::createNew(env, source);
        case VideoCodec::VP8:
            return source;
        case VideoCodec::VP9:
            return source;
        default:
            return nullptr;
    }
}

void reclaim_env(UsageEnvironment* env)
{
    if (env)
        env->reclaim();
}

void afterPlaying(void*) {}

}  // namespace

class MulticastServerMediaSubsession : public PassiveServerMediaSubsession
{
public:
    static MulticastServerMediaSubsession* createNew(RTPSink& sink,
                                                     RTCPInstance* rtcp);

protected:
    MulticastServerMediaSubsession(RTPSink& sink, RTCPInstance* rtcp);
    const char* sdpLines() override;
};

MulticastServerMediaSubsession*
MulticastServerMediaSubsession::createNew(RTPSink& sink, RTCPInstance* rtcp)
{
    return new MulticastServerMediaSubsession(sink, rtcp);
}

MulticastServerMediaSubsession::MulticastServerMediaSubsession(
    RTPSink& sink, RTCPInstance* rtcp)
    : PassiveServerMediaSubsession(sink, rtcp)
{
}

const char* MulticastServerMediaSubsession::sdpLines()
{
    delete[] fSDPLines;
    fSDPLines = nullptr;
    return PassiveServerMediaSubsession::sdpLines();
}

class UnicastServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
    static UnicastServerMediaSubsession*
    createNew(UsageEnvironment& env, std::weak_ptr<StreamServer> server,
              portNumBits initialPortNum, Boolean multiplexRTCPWithRTP);

protected:
    UnicastServerMediaSubsession(UsageEnvironment& env,
                                 std::weak_ptr<StreamServer> server,
                                 portNumBits initialPortNum,
                                 Boolean multiplexRTCPWithRTP);
    const char* sdpLines() override;
    FramedSource* createNewStreamSource(unsigned clientSessionId,
                                        unsigned& estBitrate) override;
    RTPSink* createNewRTPSink(Groupsock* rtpGroupsock,
                              unsigned char rtpPayloadTypeIfDynamic,
                              FramedSource* inputSource) override;
    void closeStreamSource(FramedSource* inputSource) override;

private:
    std::weak_ptr<StreamServer> server_;
    bool dummy_session_;
};

UnicastServerMediaSubsession* UnicastServerMediaSubsession::createNew(
    UsageEnvironment& env, std::weak_ptr<StreamServer> server,
    portNumBits initialPortNum, Boolean multiplexRTCPWithRTP)
{
    return new UnicastServerMediaSubsession(env, server, initialPortNum,
                                            multiplexRTCPWithRTP);
}

UnicastServerMediaSubsession::UnicastServerMediaSubsession(
    UsageEnvironment& env, std::weak_ptr<StreamServer> server,
    portNumBits initialPortNum, Boolean multiplexRTCPWithRTP)
    :
#if LIVEMEDIA_LIBRARY_VERSION_INT >= 1454976000
      OnDemandServerMediaSubsession(env, True, initialPortNum,
                                    multiplexRTCPWithRTP),
#else
      OnDemandServerMediaSubsession(env, True, initialPortNum),
#endif
      server_(server), dummy_session_(false)
{
}

const char* UnicastServerMediaSubsession::sdpLines()
{
#if LIVEMEDIA_LIBRARY_VERSION_INT >= 1575417600
    // setSDPLinesFromRTPSink() was private before version 2019.12.04
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s && s->sink_)
    {
        setSDPLinesFromRTPSink(s->sink_, nullptr, ESTIMATED_BITRATE);
    }
#endif
    dummy_session_ = true;
    const char* lines = OnDemandServerMediaSubsession::sdpLines();
    dummy_session_ = false;
    return lines;
}

RTPSink* UnicastServerMediaSubsession::createNewRTPSink(
    Groupsock* rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
    FramedSource* inputSource)
{
    VideoRTPSink* sink = nullptr;
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        sink = createVideoRTPSink(s->codec(), envir(), rtpGroupsock,
                                  rtpPayloadTypeIfDynamic);
        if (!dummy_session_)
            s->sink_ = sink;
        if (sink)
        {
            sink->setPacketSizes(s->preferredPacketSize(), s->maxPacketSize());
        }
    }
    return sink;
}

FramedSource*
UnicastServerMediaSubsession::createNewStreamSource(unsigned clientSessionId,
                                                    unsigned& estBitrate)
{
    FramedSource* source = nullptr;
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        estBitrate = ESTIMATED_BITRATE;
        FrameInjector* injector = FrameInjector::createNew(envir());
        source = createDiscreteFramer(s->codec(), envir(), injector);
        if (source)
        {
            if (!dummy_session_)
                s->newStreamSource(source, injector);
        }
        else
        {
            injector->shutdown();
        }
    }
    return source;
}

void UnicastServerMediaSubsession::closeStreamSource(FramedSource* inputSource)
{
    std::shared_ptr<StreamServer> s = server_.lock();
    if (s)
    {
        s->closeStreamSource(inputSource);
    }
    OnDemandServerMediaSubsession::closeStreamSource(inputSource);
}

std::shared_ptr<StreamServer>
StreamServer::create(const std::string& topic_name, unsigned udp_port,
                     unsigned udp_packet_size)
{
    return std::shared_ptr<StreamServer>(
        new StreamServer(topic_name, udp_port, udp_packet_size));
}

StreamServer::StreamServer(const std::string& topic_name, unsigned udp_port,
                           unsigned udp_packet_size)
    : codec_(VideoCodec::Unknown), topic_name_(topic_name), quit_flag_(0),
      udp_packet_size_(udp_packet_size),
      scheduler_(BasicTaskScheduler::createNew()),
      env_(BasicUsageEnvironment::createNew(*scheduler_), reclaim_env),
      rtsp_(nullptr), sms_(nullptr), sink_(nullptr)
{
    rtsp_ = RTSPServer::createNew(*env_, udp_port);
    if (!rtsp_)
        throw StreamingError(
            udp_port == 0
                ? "cannot create RTSP server"
                : (boost::format("cannot create RTSP server on port %1%")
                   % udp_port)
                      .str());
    event_loop_thread_ = std::thread(
        [this]()
        { this->env_->taskScheduler().doEventLoop(&this->quit_flag_); });
}

void StreamServer::start(VideoCodec codec, bool use_multicast)
{
    RTCPInstance* rtcp = nullptr;
    OutPacketBuffer::increaseMaxSizeTo(131072);
    stop();
    codec_ = codec;
    if (use_multicast)
    {
        if (!rtp_mcast_ || !rtcp_mcast_)
        {
            struct in_addr sockAddr =
                makeAddr(chooseRandomIPv4SSMAddress(*env_));
            rtp_mcast_ = std::make_shared<Groupsock>(std::ref(*env_), sockAddr,
                                                     sockToPort(sockAddr), 255);
            rtp_mcast_->multicastSendOnly();
            rtcp_mcast_ = std::make_shared<Groupsock>(
                std::ref(*env_), sockAddr, sockToPort(sockAddr) + 1, 255);
            rtcp_mcast_->multicastSendOnly();
        }
        sink_ = createVideoRTPSink(codec_, *env_, rtp_mcast_.get(), 96);
        if (!sink_)
            throw StreamingError(
                (boost::format("cannot instantiate VideoRTPSink for %1%")
                 % videoCodecName(codec_))
                    .str());
        sink_->setPacketSizes(preferredPacketSize(), maxPacketSize());
        char hostname[HOST_NAME_MAX + 1];
        if (gethostname(hostname, sizeof(hostname)) == 0)
        {
            hostname[HOST_NAME_MAX] = 0;
            rtcp = RTCPInstance::createNew(
                *env_, rtcp_mcast_.get(), ESTIMATED_BITRATE,
                reinterpret_cast<unsigned char*>(hostname), sink_, NULL, True);
        }
        else
            throw StreamingError("missing or invalid hostname on this system");
        sms_ = ServerMediaSession::createNew(*env_, "", "rtsp_image_transport",
                                             topic_name_.c_str(),
                                             /*multicast*/ True);
        sms_->addSubsession(
            MulticastServerMediaSubsession::createNew(*sink_, rtcp));
        rtsp_->addServerMediaSession(sms_);
        FrameInjector* injector = FrameInjector::createNew(*env_);
        FramedSource* source = createDiscreteFramer(codec_, *env_, injector);
        if (!source)
            throw StreamingError(
                (boost::format("cannot instantiate FramedSource for %1%")
                 % videoCodecName(codec_))
                    .str());
        newStreamSource(source, injector);
        sink_->startPlaying(*source, afterPlaying, this);
    }
    else
    {
        sms_ = ServerMediaSession::createNew(*env_, "", "rtsp_image_transport",
                                             topic_name_.c_str(),
                                             /*multicast*/ False);
        sms_->addSubsession(UnicastServerMediaSubsession::createNew(
            *env_, shared_from_this(), 1024 + random() % 58976, False));
        rtsp_->addServerMediaSession(sms_);
    }
    boost::scoped_array<char> tmp(rtsp_->rtspURL(sms_, ros_interface_socket()));
    url_ = std::string(tmp.get());
    ROS_INFO_STREAM("[" << topic_name_ << "] new RTSP session at " << url_);
}

void StreamServer::stop()
{
    std::unique_lock<std::mutex> lock{streams_mutex_};
    if (!url_.empty())
        ROS_DEBUG_STREAM("[" << topic_name_ << "] finished RTSP session at "
                             << url_);
    ServerMediaSession* old_sms = sms_;
    sms_ = nullptr;
    sink_ = nullptr;
    for (auto& stream : streams_)
    {
        if (stream.second)
            stream.second->shutdown();
    }
    streams_.clear();
    url_.clear();
    lock.unlock();

    if (old_sms && rtsp_)
    {
        rtsp_->deleteServerMediaSession(old_sms);
    }
}

StreamServer::~StreamServer()
{
    stop();
    if (rtsp_)
        Medium::close(rtsp_);
    quit_flag_ = 1;
    event_loop_thread_.join();
}

bool StreamServer::hasActiveStreams() const noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    return !streams_.empty();
}

void StreamServer::sendFrame(const FrameDataPtr& frame) noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    for (auto& stream : streams_)
    {
        stream.second->injectFrame(frame);
    }
}

VideoCodec StreamServer::codec() const noexcept
{
    return codec_;
}

unsigned StreamServer::preferredPacketSize() const noexcept
{
    return udp_packet_size_ < 982 ? udp_packet_size_ : 982;
}

unsigned StreamServer::maxPacketSize() const noexcept
{
    return udp_packet_size_;
}

std::string StreamServer::url() const noexcept
{
    return url_;
}

void StreamServer::newStreamSource(FramedSource* source,
                                   FrameInjector* injector) noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    if (source && injector)
    {
        if (streams_.insert(std::make_pair(source, injector)).second)
            ROS_DEBUG_STREAM("[" << topic_name_ << "] created media source "
                                 << source);
    }
}

void StreamServer::closeStreamSource(FramedSource* source) noexcept
{
    std::lock_guard<std::mutex> lock{streams_mutex_};
    StreamMapping::iterator it = streams_.find(source);
    if (it != streams_.end())
    {
        it->second->shutdown();
        ROS_DEBUG_STREAM("[" << topic_name_ << "] destroyed media source "
                             << source);
        streams_.erase(it);
    }
    if (streams_.empty())
        sink_ = nullptr;
}

}  // namespace rtsp_image_transport
