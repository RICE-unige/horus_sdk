// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/protocol_handler.hpp"
#include <cstring>
#include <algorithm>

namespace horus_unity_bridge
{

ProtocolHandler::ProtocolHandler()
  : state_(ParserState::READING_DEST_LENGTH),
    dest_length_(0),
    payload_length_(0),
    bytes_needed_(0),
    length_buffer_pos_(0)
{
  stats_ = Statistics{};
}

size_t ProtocolHandler::parse_messages(
  const uint8_t* data,
  size_t size,
  std::vector<ProtocolMessage>& messages)
{
  const uint8_t* current = data;
  size_t remaining = size;
  size_t total_consumed = 0;
  
  while (remaining > 0) {
    bool message_complete = false;
    
    switch (state_) {
      case ParserState::READING_DEST_LENGTH: {
        // Read 4-byte destination length
        while (length_buffer_pos_ < 4 && remaining > 0) {
          length_buffer_[length_buffer_pos_++] = *current++;
          remaining--;
          total_consumed++;
        }
        
        if (length_buffer_pos_ == 4) {
          // Parse little-endian uint32
          dest_length_ = static_cast<uint32_t>(length_buffer_[0]) |
                        (static_cast<uint32_t>(length_buffer_[1]) << 8) |
                        (static_cast<uint32_t>(length_buffer_[2]) << 16) |
                        (static_cast<uint32_t>(length_buffer_[3]) << 24);
          
          current_dest_.clear();
          current_dest_.reserve(dest_length_);
          state_ = ParserState::READING_DESTINATION;
          length_buffer_pos_ = 0;
          bytes_needed_ = dest_length_;
        }
        break;
      }
      
      case ParserState::READING_DESTINATION: {
        // Read destination string
        size_t to_read = std::min(remaining, bytes_needed_);
        current_dest_.append(reinterpret_cast<const char*>(current), to_read);
        current += to_read;
        remaining -= to_read;
        total_consumed += to_read;
        bytes_needed_ -= to_read;
        
        if (bytes_needed_ == 0) {
          state_ = ParserState::READING_PAYLOAD_LENGTH;
        }
        break;
      }
      
      case ParserState::READING_PAYLOAD_LENGTH: {
        // Read 4-byte payload length
        while (length_buffer_pos_ < 4 && remaining > 0) {
          length_buffer_[length_buffer_pos_++] = *current++;
          remaining--;
          total_consumed++;
        }
        
        if (length_buffer_pos_ == 4) {
          // Parse little-endian uint32
          payload_length_ = static_cast<uint32_t>(length_buffer_[0]) |
                           (static_cast<uint32_t>(length_buffer_[1]) << 8) |
                           (static_cast<uint32_t>(length_buffer_[2]) << 16) |
                           (static_cast<uint32_t>(length_buffer_[3]) << 24);
          
          current_payload_.clear();
          if (payload_length_ > 0) {
            current_payload_.reserve(payload_length_);
            state_ = ParserState::READING_PAYLOAD;
            bytes_needed_ = payload_length_;
          } else {
            // Empty payload - message complete
            message_complete = true;
          }
          length_buffer_pos_ = 0;
        }
        break;
      }
      
      case ParserState::READING_PAYLOAD: {
        // Read payload data
        size_t to_read = std::min(remaining, bytes_needed_);
        current_payload_.insert(current_payload_.end(), current, current + to_read);
        current += to_read;
        remaining -= to_read;
        total_consumed += to_read;
        bytes_needed_ -= to_read;
        
        if (bytes_needed_ == 0) {
          message_complete = true;
        }
        break;
      }
    }
    
    if (message_complete) {
      // Unity often sends command/topic names with trailing null terminators.
      while (!current_dest_.empty() && current_dest_.back() == '\0') {
        current_dest_.pop_back();
      }
      // Create completed message
      ProtocolMessage msg;
      msg.destination = std::move(current_dest_);
      msg.payload = std::move(current_payload_);
      messages.push_back(std::move(msg));
      
      // Reset for next message
      state_ = ParserState::READING_DEST_LENGTH;
      dest_length_ = 0;
      payload_length_ = 0;
      bytes_needed_ = 0;
      current_dest_.clear();
      current_payload_.clear();
      
      stats_.messages_parsed++;
    }
  }
  
  stats_.bytes_received += total_consumed;
  return total_consumed;
}

std::vector<uint8_t> ProtocolHandler::serialize_message(
  const std::string& destination,
  const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> result;
  result.reserve(8 + destination.size() + payload.size());
  
  // Destination length (4 bytes, little-endian)
  uint32_t dest_len = static_cast<uint32_t>(destination.size());
  write_uint32(result, dest_len);
  
  // Destination string
  result.insert(result.end(), destination.begin(), destination.end());
  
  // Payload length (4 bytes, little-endian)
  uint32_t payload_len = static_cast<uint32_t>(payload.size());
  write_uint32(result, payload_len);
  
  // Payload data
  result.insert(result.end(), payload.begin(), payload.end());
  
  stats_.messages_serialized++;
  stats_.bytes_sent += result.size();
  
  return result;
}

std::vector<uint8_t> ProtocolHandler::serialize_system_command(
  const std::string& command,
  const std::string& params)
{
  std::vector<uint8_t> result;
  result.reserve(8 + command.size() + params.size());
  
  // Command name length and data
  uint32_t cmd_len = static_cast<uint32_t>(command.size());
  write_uint32(result, cmd_len);
  result.insert(result.end(), command.begin(), command.end());
  
  // Parameters length and data
  uint32_t params_len = static_cast<uint32_t>(params.size());
  write_uint32(result, params_len);
  result.insert(result.end(), params.begin(), params.end());
  
  stats_.messages_serialized++;
  stats_.bytes_sent += result.size();
  
  return result;
}

void ProtocolHandler::reset()
{
  state_ = ParserState::READING_DEST_LENGTH;
  dest_length_ = 0;
  payload_length_ = 0;
  bytes_needed_ = 0;
  length_buffer_pos_ = 0;
  current_dest_.clear();
  current_payload_.clear();
}

void ProtocolHandler::write_uint32(std::vector<uint8_t>& buffer, uint32_t value)
{
  buffer.push_back(static_cast<uint8_t>(value & 0xFF));
  buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

} // namespace horus_unity_bridge
