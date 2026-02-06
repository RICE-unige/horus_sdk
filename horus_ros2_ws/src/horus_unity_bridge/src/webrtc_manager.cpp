// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/webrtc_manager.hpp"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstddef>
#include <chrono>

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
  if (video_ssrc_ == 0) {
    const auto now_ticks = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    const auto derived_ssrc = static_cast<uint32_t>(now_ticks & 0xFFFFFFFFu);
    video_ssrc_ = (derived_ssrc == 0u) ? 1u : derived_ssrc;
  }

  config_.iceServers.clear();
  for (const auto& server : settings_.stun_servers) {
    if (server.empty()) {
      continue;
    }

    try {
      config_.iceServers.emplace_back(server);
    } catch (const std::exception& e) {
      std::cerr << "Invalid ICE server URL '" << server << "': " << e.what() << std::endl;
    }
  }

  config_.iceTransportPolicy = rtc::TransportPolicy::All;
  config_.enableIceTcp = true;
  config_.enableIceUdpMux = true;
  config_.forceMediaTransport = true;

  peer_connection_ = std::make_shared<rtc::PeerConnection>(config_);

  peer_connection_->onLocalDescription([this](rtc::Description description) {
    nlohmann::json data;
    data["sdp"] = std::string(description);
    data["type"] = description.typeString();

    const bool should_delay_answer =
      settings_.wait_for_complete_gathering &&
      description.type() == rtc::Description::Type::Answer &&
      peer_connection_ &&
      peer_connection_->gatheringState() != rtc::PeerConnection::GatheringState::Complete;

    if (should_delay_answer) {
      {
        std::lock_guard<std::mutex> lock(pending_description_mutex_);
        has_pending_local_description_ = true;
        pending_local_description_type_ = description.typeString();
        pending_local_description_data_ = data;
      }
      std::cout << "Delaying local WebRTC answer until ICE gathering completes" << std::endl;
      return;
    }

    std::lock_guard<std::mutex> signaling_lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_(description.typeString(), data);
    }
  });

  peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
    std::string candidate_sdp = std::string(candidate);
    if (candidate_sdp.rfind("a=", 0) == 0) {
      candidate_sdp = candidate_sdp.substr(2);
    }
    std::string candidate_type = "unknown";
    const std::string type_marker = " typ ";
    const auto type_pos = candidate_sdp.find(type_marker);
    if (type_pos != std::string::npos) {
      const auto start = type_pos + type_marker.size();
      auto end = candidate_sdp.find(' ', start);
      if (end == std::string::npos) {
        end = candidate_sdp.size();
      }
      if (end > start) {
        candidate_type = candidate_sdp.substr(start, end - start);
      }
    }
    std::cout << "WebRTC local candidate mid=" << candidate.mid()
              << " type=" << candidate_type
              << " sdp='" << candidate_sdp << "'" << std::endl;

    nlohmann::json data;
    data["candidate"] = candidate_sdp;
    data["sdpMid"] = candidate.mid();
    // Note: libdatachannel doesn't directly expose mLineIndex
    // For now, we'll omit it or set to 0 as Unity/browser can derive it from sdpMid
    data["sdpMLineIndex"] = 0;
    
    std::lock_guard<std::mutex> lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_("candidate", data);
    }
  });

  peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
    std::cout << "WebRTC State: " << state << std::endl;
    switch (state) {
      case rtc::PeerConnection::State::Connected:
        peer_connected_.store(true);
        start_pipeline();
        request_keyframe();
        break;
      case rtc::PeerConnection::State::Disconnected:
      case rtc::PeerConnection::State::Failed:
      case rtc::PeerConnection::State::Closed:
        peer_connected_.store(false);
        video_track_open_.store(false);
        stop_pipeline();
        break;
      default:
        break;
    }
  });
  peer_connection_->onIceStateChange([this](rtc::PeerConnection::IceState state) {
    std::cout << "WebRTC ICE State: " << state << std::endl;
    switch (state) {
      case rtc::PeerConnection::IceState::Connected:
      case rtc::PeerConnection::IceState::Completed:
        peer_connected_.store(true);
        start_pipeline();
        request_keyframe();
        break;
      case rtc::PeerConnection::IceState::Failed:
      case rtc::PeerConnection::IceState::Disconnected:
      case rtc::PeerConnection::IceState::Closed:
        peer_connected_.store(false);
        video_track_open_.store(false);
        stop_pipeline();
        break;
      default:
        break;
    }
  });
  peer_connection_->onGatheringStateChange([this](rtc::PeerConnection::GatheringState state) {
    std::cout << "WebRTC Gathering State: " << state << std::endl;
    if (state != rtc::PeerConnection::GatheringState::Complete ||
        !settings_.wait_for_complete_gathering) {
      return;
    }

    bool emit_pending = false;
    std::string pending_type;
    nlohmann::json pending_data;
    {
      std::lock_guard<std::mutex> lock(pending_description_mutex_);
      if (has_pending_local_description_) {
        emit_pending = true;
        pending_type = pending_local_description_type_;
        pending_data = pending_local_description_data_;
        has_pending_local_description_ = false;
        pending_local_description_type_.clear();
        pending_local_description_data_.clear();
      }
    }

    if (!emit_pending) {
      return;
    }

    std::lock_guard<std::mutex> signaling_lock(signaling_mutex_);
    if (signaling_callback_) {
      signaling_callback_(pending_type, pending_data);
      std::cout << "Published delayed WebRTC answer after ICE gathering completed" << std::endl;
    }
  });
  peer_connection_->onSignalingStateChange([](rtc::PeerConnection::SignalingState state) {
    std::cout << "WebRTC Signaling State: " << state << std::endl;
  });

  // Explicitly declare H264 + SSRC so SDP and outgoing RTP packets stay aligned.
  rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
  media.addH264Codec(video_payload_type_);
  media.addSSRC(video_ssrc_, "horus-video");
  video_track_ = peer_connection_->addTrack(media);
  if (video_track_) {
    video_track_->onOpen([this]() {
      video_track_open_.store(true);
      std::cout << "WebRTC video track open" << std::endl;
      request_keyframe();
    });
    video_track_->onClosed([this]() {
      video_track_open_.store(false);
      std::cout << "WebRTC video track closed" << std::endl;
    });
  }
}

void WebRTCManager::handle_offer(const std::string& sdp)
{
  if (!initialized_) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(pending_description_mutex_);
    has_pending_local_description_ = false;
    pending_local_description_type_.clear();
    pending_local_description_data_.clear();
  }
  std::cout << "Received WebRTC Offer" << std::endl;
  peer_connection_->setRemoteDescription(rtc::Description(sdp, "offer"));
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
  std::string normalized_candidate = candidate;
  if (normalized_candidate.rfind("a=", 0) == 0) {
    normalized_candidate = normalized_candidate.substr(2);
  }

  std::cout << "Received WebRTC Candidate (mid=" << sdp_mid
            << ", mline=" << sdp_mline_index << ")" << std::endl;
  std::vector<std::string> mids_to_try;
  if (!sdp_mid.empty()) {
    mids_to_try.push_back(sdp_mid);
  }
  if (video_track_) {
    try {
      const auto track_mid = video_track_->mid();
      if (!track_mid.empty()) {
        mids_to_try.push_back(track_mid);
      }
    } catch (...) {
    }
  }
  mids_to_try.push_back("0");
  mids_to_try.push_back("video");

  auto dedupe = [](std::vector<std::string>& mids) {
    std::vector<std::string> unique;
    for (const auto& mid : mids) {
      if (mid.empty()) {
        continue;
      }
      if (std::find(unique.begin(), unique.end(), mid) == unique.end()) {
        unique.push_back(mid);
      }
    }
    mids.swap(unique);
  };
  dedupe(mids_to_try);

  std::string first_error;
  for (const auto& mid : mids_to_try) {
    try {
      peer_connection_->addRemoteCandidate(rtc::Candidate(normalized_candidate, mid));
      if (mid != sdp_mid) {
        std::cout << "Accepted WebRTC Candidate using fallback mid='"
                  << mid << "'" << std::endl;
      }
      return;
    } catch (const std::exception& e) {
      if (first_error.empty()) {
        first_error = e.what();
      }
    }
  }

  if (!first_error.empty()) {
    std::cerr << "Failed to add remote candidate: " << first_error << std::endl;
  }
}

bool WebRTCManager::setup_gstreamer_pipeline()
{
  auto has_factory = [](const char * factory_name) -> bool {
      if (!factory_name || !*factory_name) {
        return false;
      }
      GstElementFactory * factory = gst_element_factory_find(factory_name);
      if (!factory) {
        return false;
      }
      gst_object_unref(factory);
      return true;
    };

  auto build_pipeline = [this](const std::string & encoder_stage) {
      std::ostringstream builder;
      builder << "appsrc name=mysource format=time is-live=true do-timestamp=true ! ";
      builder << "videoconvert ! ";
      builder << encoder_stage << " ! ";
      builder << "h264parse ! ";
      builder << "rtph264pay config-interval=1 pt=" << video_payload_type_;
      builder << " ssrc=" << video_ssrc_ << " mtu=1200 ! ";
      builder << "appsink name=mysink emit-signals=true sync=false";
      return builder.str();
    };

  std::string pipeline_str;
  if (!settings_.pipeline.empty()) {
    pipeline_str = settings_.pipeline;
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
      std::cerr << "Failed to create pipeline: "
                << (error ? error->message : "unknown error") << std::endl;
      if (error) {
        g_error_free(error);
      }
      if (pipeline_) {
        gst_object_unref(GST_OBJECT(pipeline_));
      }
      pipeline_ = nullptr;
      return false;
    }
  } else {
    const int bitrate_kbps = std::max(1, settings_.bitrate_kbps);
    const int framerate = std::max(1, settings_.framerate);
    const int bitrate_bps = bitrate_kbps * 1000;

    std::vector<std::pair<std::string, std::string>> encoder_candidates;
    auto add_candidate = [&encoder_candidates](
      const std::string & label, const std::string & stage)
      {
        for (const auto & existing : encoder_candidates) {
          if (existing.second == stage) {
            return;
          }
        }
        encoder_candidates.emplace_back(label, stage);
      };

    // If a specific encoder was requested, try it first.
    const std::string requested_encoder = settings_.encoder;
    if (!requested_encoder.empty() && requested_encoder != "auto") {
      const bool simple_factory_name =
        requested_encoder.find(' ') == std::string::npos &&
        requested_encoder.find('!') == std::string::npos;
      if (!simple_factory_name || has_factory(requested_encoder.c_str())) {
        add_candidate("requested", requested_encoder);
      } else {
        std::cout << "Requested encoder '" << requested_encoder
                  << "' is not available. Trying fallbacks." << std::endl;
      }
    }

    // Hardware + software fallback chain.
    if (has_factory("nvh264enc")) {
      add_candidate("nvh264enc", "nvh264enc preset=low-latency-hp");
    }
    if (has_factory("vaapih264enc")) {
      add_candidate("vaapih264enc", "vaapih264enc");
    }
    if (has_factory("v4l2h264enc")) {
      add_candidate("v4l2h264enc", "v4l2h264enc");
    }
    if (has_factory("openh264enc")) {
      std::ostringstream stage;
      stage << "openh264enc bitrate=" << bitrate_bps;
      add_candidate("openh264enc", stage.str());
    }
    if (has_factory("x264enc")) {
      std::ostringstream stage;
      stage << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << bitrate_kbps
            << " key-int-max=" << framerate;
      add_candidate("x264enc", stage.str());
    }
    if (has_factory("avenc_h264")) {
      add_candidate("avenc_h264", "avenc_h264");
    }

    if (encoder_candidates.empty()) {
      std::cerr << "No supported H264 encoder found (tried nvh264enc/vaapih264enc/v4l2h264enc/openh264enc/x264enc/avenc_h264)."
                << std::endl;
      return false;
    }

    for (const auto & candidate : encoder_candidates) {
      pipeline_str = build_pipeline(candidate.second);
      std::cout << "Trying WebRTC encoder: " << candidate.first
                << " [" << candidate.second << "]" << std::endl;

      GError * error = nullptr;
      GstElement * trial_pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
      if (error || !trial_pipeline) {
        std::cerr << "Encoder '" << candidate.first << "' failed: "
                  << (error ? error->message : "unknown error") << std::endl;
        if (error) {
          g_error_free(error);
        }
        if (trial_pipeline) {
          gst_object_unref(GST_OBJECT(trial_pipeline));
        }
        continue;
      }

      pipeline_ = trial_pipeline;
      std::cout << "Selected WebRTC encoder: " << candidate.first << std::endl;
      break;
    }

    if (!pipeline_) {
      std::cerr << "Failed to create GStreamer pipeline with all encoder candidates." << std::endl;
      return false;
    }
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
  
  if (!appsrc_ || !pipeline_ || !pipeline_started_) {
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
      if (!manager->peer_connected_.load() || !manager->video_track_open_.load() ||
          manager->peer_connection_->state() != rtc::PeerConnection::State::Connected ||
          !manager->video_track_->isOpen()) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
      }
      GstMapInfo info;
      if (gst_buffer_map(buffer, &info, GST_MAP_READ)) {
        // Send RTP packet to WebRTC track
        // Note: libdatachannel expects RTP packets for media tracks
        try {
            // We need to cast pointer to byte*
            auto* data_ptr = reinterpret_cast<const std::byte*>(info.data);
            manager->video_track_->send(data_ptr, info.size);
        } catch (const std::exception& e) {
            const auto now = std::chrono::steady_clock::now();
            if (now - manager->last_track_closed_log_time_ > std::chrono::seconds(2)) {
              std::cerr << "Failed to send RTP packet: " << e.what() << std::endl;
              manager->last_track_closed_log_time_ = now;
            }
            if (std::string(e.what()).find("Track is closed") != std::string::npos) {
              manager->video_track_open_.store(false);
            }
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
