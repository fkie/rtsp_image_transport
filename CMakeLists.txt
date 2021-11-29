############################################################################
#
# rtsp_image_transport
# Copyright © 2021 Fraunhofer FKIE
# Author: Timo Röhling
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
############################################################################
cmake_minimum_required(VERSION 3.10)
project(rtsp_image_transport LANGUAGES CXX)
include(GenerateExportHeader)
include(CheckCXXSourceCompiles)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    image_transport
    message_generation
    pluginlib
    std_msgs
)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
# Need at least FFmpeg 3
pkg_check_modules(ffmpeg REQUIRED IMPORTED_TARGET
    libavcodec>=57
    libavformat>=57
    libavutil>=55
    libswscale>=4
)
pkg_check_modules(live555 REQUIRED IMPORTED_TARGET live555)

# Check compatibility of found libraries
set(CMAKE_REQUIRED_LIBRARIES "PkgConfig::ffmpeg")
check_cxx_source_compiles([==[
extern "C"
{
#include <libavcodec/avcodec.h>
}

int main()
{
    AVHWFramesContext* c1 = nullptr;
    AVHWDeviceContext* c2 = nullptr;
    AVPixelFormat f = AV_PIX_FMT_VAAPI;
}
]==] FFMPEG_HAS_HWFRAME_SUPPORT)

# Older versions of live555 had a bug that prevented the
# proper inclusion of the H.265 classes
set(CMAKE_REQUIRED_LIBRARIES "PkgConfig::live555")
check_cxx_source_compiles([==[
#include <liveMedia.hh>

int main()
{
    H265VideoRTPSink* sink = nullptr;
    H265VideoStreamDiscreteFramer* framer = nullptr;
}
]==] LIVE555_HAS_H265_SUPPORT)

check_cxx_source_compiles([==[
#include <liveMedia.hh>

int main()
{
    VP8VideoRTPSink* sink1 = nullptr;
    VP9VideoRTPSink* sink2 = nullptr;
}
]==] LIVE555_HAS_VPX_SUPPORT)

unset(CMAKE_REQUIRED_LIBRARIES)


generate_dynamic_reconfigure_options(
  cfg/RTSPPublisher.cfg
  cfg/RTSPSubscriber.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        image_transport
        message_runtime
        std_msgs
)

add_library(${PROJECT_NAME}
    src/frame_data.cpp
    src/frame_extractor.cpp
    src/frame_injector.cpp
    src/init.cpp
    src/publisher_plugin.cpp
    src/stream_client.cpp
    src/stream_decoder.cpp
    src/stream_encoder.cpp
    src/stream_server.cpp
    src/subscriber_plugin.cpp
    src/video_codec.cpp
)
generate_export_header(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        $<$<BOOL:${LIVE555_HAS_H265_SUPPORT}>:LIVE555_HAS_H265_SUPPORT>
        $<$<BOOL:${LIVE555_HAS_VPX_SUPPORT}>:LIVE555_HAS_VPX_SUPPORT>
        $<$<BOOL:${FFMPEG_HAS_HWFRAME_SUPPORT}>:FFMPEG_HAS_HWFRAME_SUPPORT>
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${catkin_LIBRARIES}
        PkgConfig::ffmpeg
        PkgConfig::live555
        Boost::boost
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_VISIBILITY_PRESET "hidden"
    VISIBILITY_INLINES_HIDDEN ON
)

# Make the "RTSP Only" test image embeddable
add_custom_command(OUTPUT rtsp_only.o
    MAIN_DEPENDENCY rtsp_only.png
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMAND ld -r -b binary -o ${CMAKE_CURRENT_BINARY_DIR}/rtsp_only.o rtsp_only.png
)

add_executable(publish_rtsp_stream
    rtsp_only.o
    src/publish_rtsp_stream.cpp
)

target_include_directories(publish_rtsp_stream
    SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
)

target_link_libraries(publish_rtsp_stream
    PRIVATE
        ${catkin_LIBRARIES}
        ${cv_bridge_LIBRARIES}
)

add_executable(rtsp_camera_proxy
    rtsp_only.o
    src/rtsp_camera_proxy.cpp
)

target_include_directories(rtsp_camera_proxy
    SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
)

target_link_libraries(rtsp_camera_proxy
    PRIVATE
        ${catkin_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        PkgConfig::live555
)

install(TARGETS publish_rtsp_stream rtsp_camera_proxy ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ${PROJECT_NAME}_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
