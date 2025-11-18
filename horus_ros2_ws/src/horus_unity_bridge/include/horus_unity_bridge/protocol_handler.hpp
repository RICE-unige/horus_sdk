// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <memory>
#include <array>

namespace horus_unity_bridge
{

/**
 * @brief Unity-ROS protocol message structure
 * 
 * Binary wire format:
 * - 4 bytes: destination length (uint32_t, little-endian)
 * - N bytes: destination string (UTF-8)
 * - 4 bytes: payload length (uint32_t, little-endian)
 * - M bytes: payload data (serialized ROS message)
 */
struct ProtocolMessage
{
  std::string destination;        // Topic name, service name, or system command
  std::vector<uint8_t> payload;   // Serialized message data
  
  // System command identification
  bool is_system_command() const {
    return !destination.empty() && destination[0] == '_' && destination[1] == '_';
  }
  
  bool is_keepalive() const {
    return destination.empty();
  }
};

/**
 * @brief High-performance protocol handler for Unity-ROS communication
 * 
 * Features:
 * - Zero-copy parsing where possible
 * - Efficient buffer management
 * - Chunked message handling for large payloads
 * - Validation and error checking
 */
class ProtocolHandler
{
public:
  ProtocolHandler();
  ~ProtocolHandler() = default;
  
  /**
   * @brief Parse incoming bytes into protocol messages
   * 
   * @param data Raw bytes from socket
   * @param size Number of bytes
   * @param messages Output vector of parsed messages
   * @return Number of bytes consumed (may be less than size if incomplete message)
   */
  size_t parse_messages(
    const uint8_t* data,
    size_t size,
    std::vector<ProtocolMessage>& messages
  );
  
  /**
   * @brief Serialize a message for transmission
   * 
   * @param destination Topic/service/command name
   * @param payload Message data
   * @return Serialized bytes ready for socket transmission
   */
  std::vector<uint8_t> serialize_message(
    const std::string& destination,
    const std::vector<uint8_t>& payload
  );
  
  /**
   * @brief Serialize a system command
   * 
   * @param command Command name (e.g., "__subscribe")
   * @param params JSON parameters as string
   * @return Serialized bytes
   */
  std::vector<uint8_t> serialize_system_command(
    const std::string& command,
    const std::string& params
  );
  
  /**
   * @brief Reset parser state (e.g., after connection reset)
   */
  void reset();
  
  /**
   * @brief Get statistics
   */
  struct Statistics {
    uint64_t messages_parsed = 0;
    uint64_t messages_serialized = 0;
    uint64_t bytes_received = 0;
    uint64_t bytes_sent = 0;
    uint64_t parse_errors = 0;
  };
  
  const Statistics& get_statistics() const { return stats_; }

private:
  // Internal parser state for handling partial messages
  enum class ParserState {
    READING_DEST_LENGTH,
    READING_DESTINATION,
    READING_PAYLOAD_LENGTH,
    READING_PAYLOAD
  };
  
  ParserState state_;
  uint32_t dest_length_;
  uint32_t payload_length_;
  std::string current_dest_;
  std::vector<uint8_t> current_payload_;
  size_t bytes_needed_;
  
  // Buffer for partial reads
  std::array<uint8_t, 4> length_buffer_;
  size_t length_buffer_pos_;
  
  Statistics stats_;
  
  // Helper methods
  bool read_uint32(const uint8_t*& data, size_t& remaining, uint32_t& value);
  bool read_bytes(const uint8_t*& data, size_t& remaining, 
                  std::vector<uint8_t>& buffer, size_t count);
  
  void write_uint32(std::vector<uint8_t>& buffer, uint32_t value);
};

} // namespace horus_unity_bridge
