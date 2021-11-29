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
#include "frame_injector.h"

namespace rtsp_image_transport
{

FrameInjector* FrameInjector::createNew(UsageEnvironment& env)
{
    return new FrameInjector(env);
}

FrameInjector::FrameInjector(UsageEnvironment& env)
    : FramedSource(env), is_shutdown_(false),
      deliver_frame_trigger_(envir().taskScheduler().createEventTrigger(
          FrameInjector::deliverFrameEvent))
{
}

FrameInjector::~FrameInjector()
{
    shutdown();
    envir().taskScheduler().deleteEventTrigger(deliver_frame_trigger_);
}

void FrameInjector::shutdown()
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    is_shutdown_ = true;
    frame_queue_.clear();
}

void FrameInjector::injectFrame(const FrameDataPtr& frame)
{
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    if (is_shutdown_)
        return;
    frame_queue_.push_back(frame);
    envir().taskScheduler().triggerEvent(deliver_frame_trigger_, this);
}

void FrameInjector::doGetNextFrame()
{
    if (!is_shutdown_)
    {
        deliverFrame();
    }
    else
    {
        handleClosure(this);
    }
}

void FrameInjector::deliverFrame()
{
    if (!isCurrentlyAwaitingData())
        return;
    std::lock_guard<std::mutex> lock{frame_queue_mutex_};
    if (frame_queue_.empty())
        return;
    FrameDataPtr frame = frame_queue_.front();
    frame_queue_.pop_front();
    if (frame->length() <= fMaxSize)
    {
        fFrameSize = frame->length();
    }
    else
    {
        fFrameSize = fMaxSize;
        fNumTruncatedBytes = frame->length() - fMaxSize;
    }
    uint64_t nsec = frame->stamp().toNSec();
    fPresentationTime.tv_sec = nsec / 1000000000ull;
    fPresentationTime.tv_usec = (nsec % 1000000000ull) / 1000ull;
    std::copy_n(frame->data(), fFrameSize, fTo);
    FramedSource::afterGetting(this);
}

void FrameInjector::deliverFrameEvent(void* instance)
{
    static_cast<FrameInjector*>(instance)->deliverFrame();
}

}  // namespace rtsp_image_transport