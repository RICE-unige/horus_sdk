// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>

#include <rtc/rtc.hpp>
#include <nlohmann/json.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace horus_unity_bridge
{

class WebRTCManager
{
public:
  using SignalingCallback = std::function<void(const std::string& type, const nlohmann::json& data)>;
  struct Settings {
    std::vector<std::string> stun_servers;
    std::string encoder = "x264enc";
    int bitrate_kbps = 2000;
    int framerate = 30;
    std::string pipeline;
    bool wait_for_complete_gathering = true;
  };

  WebRTCManager();
  ~WebRTCManager();

  // Initialize GStreamer and WebRTC configuration
  bool initialize(const Settings& settings);
  
  // Set callback for sending signaling messages back to the client via TCP
  void set_signaling_callback(SignalingCallback callback);

  // Handle incoming signaling messages from the client
  void handle_offer(const std::string& sdp);
  void handle_answer(const std::string& sdp);
  void handle_candidate(const std::string& candidate, const std::string& sdp_mid, int sdp_mline_index);

  // Feed video frame to the streaming pipeline
  void push_frame(const std::vector<uint8_t>& data, int width, int height, const std::string& format);

  // Request a keyframe from the encoder (useful for packet loss recovery)
  void request_keyframe();

private:
  Settings settings_;
  bool initialized_ = false;
  uint32_t video_ssrc_ = 0;
  int video_payload_type_ = 96;
  // WebRTC components
  std::shared_ptr<rtc::PeerConnection> peer_connection_;
  std::shared_ptr<rtc::Track> video_track_;
  std::shared_ptr<rtc::DataChannel> data_channel_;
  
  // GStreamer pipeline
  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* appsink_ = nullptr;
  std::atomic<bool> pipeline_started_{false};
  std::mutex frame_mutex_;
  std::atomic<bool> peer_connected_{false};
  std::atomic<bool> video_track_open_{false};
  std::chrono::steady_clock::time_point last_track_closed_log_time_{};

  // Caps caching to avoid redundant GStreamer calls
  int last_width_ = 0;
  int last_height_ = 0;
  std::string last_format_;
  
  SignalingCallback signaling_callback_;
  std::mutex signaling_mutex_;
  std::mutex pending_description_mutex_;
  bool has_pending_local_description_ = false;
  std::string pending_local_description_type_;
  nlohmann::json pending_local_description_data_;
  std::mutex pipeline_mutex_;
  
  // Configuration
  rtc::Configuration config_;
  
  // Internal helpers
  void create_peer_connection();
  bool setup_gstreamer_pipeline();
  void start_pipeline();
  void stop_pipeline();
  static GstFlowReturn on_new_sample(GstElement* sink, gpointer user_data);
};

} // namespace horus_unity_bridge
