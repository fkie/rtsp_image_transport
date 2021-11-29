****************************
RTSP Image Transport for ROS
****************************

Overview
========

This package allow ROS nodes to publish and subscribe image topics using the
`Real Time Streaming Protocol (RTSP)`_. Unlike regular `image_transport`_
plugins, the ``rtsp_image_transport`` does not transmit image data in-band. The
publisher merely publishes a latched `std_msgs/String`_ message with the URL of
an :RFC:`2326` compliant RTSP video stream server, from where the actual image
data is served. The subscriber will listen for URLs on the ROS topic and
automatically (re-)connect to the corresponding location.

Use Cases
=========

``rtsp_image_transport`` is intended for streaming live videos to one or more
users. The RTSP server supports multicast transmission, so it will potentially
save a huge amount of bandwidth if many clients are viewing the same video
stream.

Limitations
===========

If the image source is not a continuous video stream with roughly constant
frame rate, you may experience problems such as session timeouts or degraded
image quality.

``rtsp_image_transport`` is not suitable for image data that is to be consumed
by image processing algorithms. The lossy compression introduces artifacts
which may not be visible to the human eye but interfere with many algorithms
nevertheless.

You cannot use `rosbag`_ to record data from ``rtsp_image_transport``; you will
just end up with a bunch of useless URL string messages.

Supported Formats
=================

``rtsp_image_transport`` uses the `Live555`_ library for its RTSP server and
client implementation and the `FFmpeg`_ library for video compression. Starting
with FFmpeg 4, some codecs are hardware acceleratable. The following table
summarizes the options, subject to availability in your FFmpeg version and
compatible hardware:

+----------+----------+----------+----------+----------+
|          | Software | NVIDIA   | VAAPI    | OMX      |
+==========+==========+==========+==========+==========+
| H.264    | Yes      | Yes      | Encoding | Encoding |
+----------+----------+----------+----------+----------+
| H.265    | Yes      | Yes      | Encoding | No       |
+----------+----------+----------+----------+----------+
| MPEG-4   | Yes      | Decoding | Encoding | No       |
+----------+----------+----------+----------+----------+
| VP8      | Yes      | Decoding | Encoding | No       |
+----------+----------+----------+----------+----------+
| VP9      | Yes      | Decoding | Encoding | No       |
+----------+----------+----------+----------+----------+
| MJPEG    | Decoding | Decoding | No       | No       |
+----------+----------+----------+----------+----------+

.. note::
    ``rtsp_image_transport`` cannot create Motion JPEG (MJPEG) streams,
    only receive them for backwards compatibility with some ancient IP
    cameras. If you really want to have independently compressed JPEG
    frames for your video stream, you can use the
    `compressed_image_transport`_ with JPEG compression instead.


IP cameras
==========

The ``rtsp_image_transport`` plugin is compatible with many commercially
available IP cameras. You can use the ``publish_rtsp_stream`` node to create an
image topic with RTSP transport that reuses the existing RTSP stream.

.. _Real Time Streaming Protocol (RTSP): https://en.wikipedia.org/wiki/Real_Time_Streaming_Protocol

.. _image_transport: http://wiki.ros.org/image_transport

.. _compressed_image_transport: http://wiki.ros.org/compressed_image_transport

.. _std_msgs/String: http://docs.ros.org/en/api/std_msgs/html/msg/String.html

.. _Live555: http://www.live555.com/

.. _FFmpeg: https://ffmpeg.org/

.. _rosbag: http://wiki.ros.org/rosbag
