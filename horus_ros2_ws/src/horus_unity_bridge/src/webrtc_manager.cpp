// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/webrtc_manager.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstddef>

namespace horus_unity_bridge
{

WebRTCManager::WebRTCManager()
{
}

WebRTCManager::~WebRTCManager()
{
  stop_pipeline();
  
  if (appsrc_) {
    gst_object_unref(GST_OBJECT(appsrc_));
    appsrc_ = nullptr;
  }
  if (appsink_) {
    gst_object_unref(GST_OBJECT(appsink_));
    appsink_ = nullptr;
  }
  if (pipeline_) {
    gst_object_unref(GST_OBJECT(pipeline_));
    pipeline_ = nullptr;
  }
  
  if (peer_connection_) {
    peer_connection_->close();
    peer_connection_.reset();
  }
  initialized_ = false;
}

bool WebRTCManager::initialize(const Settings& settings)
{
  settings_ = settings;
  if (settings_.stun_servers.empty()) {
    settings_.stun_servers.emplace_back("stun:stun.l.google.com:19302");
  }
  
  try {
    gst_init(nullptr, nullptr);
    create_peer_connection();
    if (!peer_connection_) {
      std::cerr << "Failed to create WebRTC peer connection" << std::endl;
      return false;
    }
    if (!setup_gstreamer_pipeline()) {
      std::cerr << "Failed to create GStreamer pipeline" << std::endl;
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "WebRTC initialization failed: " << e.what() << std::endl;
    return false;
  }
  
  initialized_ = true;
  return true;
}

void WebRTCManager::set_signaling_callback(SignalingCallback callback)
{
  std::lock_guard<std::mutex> lock(signaling_mutex_);
  signaling_callback_ = callback;
}

void WebRTCManager::create_peer_connection()
{
  config_.iceServers = settings_.stun_servers;
  peer_connection_ = std::make_shared<rtc::PeerConnection>(config_);

  peer_connection_->onLocalDescription([this](rtc::Description description) {
    nlohmann::json data;
    data["sdp"] = std::string(description);
    data["type"] = description.typeString();
    
    std::lock_guard<std::mutex> lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_(description.typeString(), data);
    }
  });

  peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
    nlohmann::json data;
    data["candidate"] = std::string(candidate);
    data["sdpMid"] = candidate.mid();
    // Note: libdatachannel doesn't directly expose mLineIndex
    // For now, we'll omit it or set to 0 as Unity/browser can derive it from sdpMid
    data["sdpMLineIndex"] = 0;
    
    std::lock_guard<std::mutex> lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_("candidate", data);
    }
  });

  peer_connection_->onStateChange([](rtc::PeerConnection::State state) {
    std::cout << "WebRTC State: " << state << std::endl;
  });

  // Create a video track
  video_track_ = peer_connection_->addTrack(rtc::Description::Video("video"));
  
  // Create a data channel for good measure (latency testing etc)
  data_channel_ = peer_connection_->createDataChannel("control");
  data_channel_->onOpen([]() {
    std::cout << "Data channel open" << std::endl;
  });
  data_channel_->onMessage([](auto data) {
    if (std::holds_alternative<std::string>(data)) {
      std::cout << "Received data: " << std::get<std::string>(data) << std::endl;
    }
  });
}

void WebRTCManager::handle_offer(const std::string& sdp)
{
  if (!initialized_) {
    return;
  }
  std::cout << "Received WebRTC Offer" << std::endl;
  peer_connection_->setRemoteDescription(rtc::Description(sdp, "offer"));
  start_pipeline();
}

void WebRTCManager::handle_answer(const std::string& sdp)
{
  if (!initialized_) {
    return;
  }
  std::cout << "Received WebRTC Answer" << std::endl;
  peer_connection_->setRemoteDescription(rtc::Description(sdp, "answer"));
}

void WebRTCManager::handle_candidate(const std::string& candidate, const std::string& sdp_mid, int sdp_mline_index)
{
  if (!initialized_) {
    return;
  }
  std::cout << "Received WebRTC Candidate" << std::endl;
  peer_connection_->addRemoteCandidate(rtc::Candidate(candidate, sdp_mid));
}

bool WebRTCManager::setup_gstreamer_pipeline()
{
  std::string pipeline_str;
  if (!settings_.pipeline.empty()) {
    pipeline_str = settings_.pipeline;
  } else {
    std::string encoder = settings_.encoder;
    
    // Auto-detect hardware encoder if default or auto is specified
    if (encoder == "x264enc" || encoder == "auto") {
      GstElementFactory* factory = gst_element_factory_find("nvh264enc");
      if (factory) {
        encoder = "nvh264enc preset=low-latency-hp";
        gst_object_unref(factory);
        std::cout << "Selected hardware encoder: NVIDIA (nvh264enc)" << std::endl;
      } else {
        factory = gst_element_factory_find("vaapih264enc");
        if (factory) {
          encoder = "vaapih264enc";
          gst_object_unref(factory);
          std::cout << "Selected hardware encoder: VAAPI (vaapih264enc)" << std::endl;
        } else {
          encoder = "x264enc tune=zerolatency speed-preset=ultrafast";
          std::cout << "Selected software encoder: x264enc" << std::endl;
        }
      }
    }

    std::ostringstream builder;
    builder << "appsrc name=mysource format=time is-live=true do-timestamp=true ! ";
    builder << "videoconvert ! ";
    builder << encoder;
    
    builder << " bitrate=" << std::max(1, settings_.bitrate_kbps);
    builder << " key-int-max=" << std::max(1, settings_.framerate);
    builder << " ! rtph264pay config-interval=1 pt=96 ! ";
    builder << "appsink name=mysink emit-signals=true sync=false";
    pipeline_str = builder.str();
  }

  GError* error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
  
  if (error) {
    std::cerr << "Failed to create pipeline: " << error->message << std::endl;
    g_error_free(error);
    pipeline_ = nullptr;
    return false;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysource");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "mysink");
  if (!appsrc_ || !appsink_) {
    std::cerr << "Failed to acquire appsrc/appsink from pipeline" << std::endl;
    return false;
  }

  // Configure appsink callback
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);
  return true;
}

void WebRTCManager::start_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_ && !pipeline_started_) {
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    pipeline_started_ = true;
  }
}

void WebRTCManager::stop_pipeline()
{
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    pipeline_started_ = false;
  }
}

void WebRTCManager::push_frame(const std::vector<uint8_t>& data, int width, int height, const std::string& format)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  
  if (!appsrc_ || !pipeline_) {
    return;
  }

  guint size = data.size();
  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) {
    std::cerr << "Failed to allocate GStreamer buffer" << std::endl;
    return;
  }
  
  gst_buffer_fill(buffer, 0, data.data(), size);

  // Set caps only if they changed
  if (width != last_width_ || height != last_height_ || format != last_format_) {
    GstCaps* caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, format.c_str(),
      "width", G_TYPE_INT, width,
      "height", G_TYPE_INT, height,
      "framerate", GST_TYPE_FRACTION, std::max(1, settings_.framerate), 1,
      nullptr);
    
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);
    
    last_width_ = width;
    last_height_ = height;
    last_format_ = format;
  }

  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);
  
  if (ret != GST_FLOW_OK) {
    std::cerr << "Failed to push frame to GStreamer pipeline: " << ret << std::endl;
  }
}

GstFlowReturn WebRTCManager::on_new_sample(GstElement* sink, gpointer user_data)
{
  WebRTCManager* manager = static_cast<WebRTCManager*>(user_data);
  GstSample* sample;
  g_signal_emit_by_name(sink, "pull-sample", &sample);
  
  if (sample) {
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    if (buffer && manager->video_track_) {
      GstMapInfo info;
      if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
        // Send RTP packet to WebRTC track
        // Note: libdatachannel expects RTP packets for media tracks
        try {
            // We need to cast pointer to byte*
            auto* data_ptr = reinterpret_cast<const std::byte*>(info.data);
            manager->video_track_->send(data_ptr, info.size);
        } catch (const std::exception& e) {
            std::cerr << "Failed to send RTP packet: " << e.what() << std::endl;
        }
        gst_buffer_unmap(buffer, &info);
      }
    }
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  return GST_FLOW_ERROR;
}

void WebRTCManager::request_keyframe()
{
  if (!pipeline_ || !appsrc_) return;
  
  GstStructure* s = gst_structure_new_empty("GstForceKeyUnit");
  gst_structure_set(s, "all-headers", G_TYPE_BOOLEAN, TRUE, nullptr);
  
  GstEvent* event = gst_event_new_custom(GST_EVENT_CUSTOM_UPSTREAM, s);
  if (!gst_element_send_event(appsrc_, event)) {
      std::cerr << "Failed to send keyframe request event" << std::endl;
  }
}

} // namespace horus_unity_bridge
