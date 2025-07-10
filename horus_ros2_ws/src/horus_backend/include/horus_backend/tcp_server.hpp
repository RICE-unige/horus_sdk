// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace horus_backend
{

class TcpServer
{
public:
  using MessageCallback = std::function<std::string(const std::string &)>;

  explicit TcpServer(int port);
  ~TcpServer();

  void start();
  void stop();
  bool is_running() const;

  void set_message_callback(MessageCallback callback);

private:
  int port_;
  int server_socket_;
  std::atomic<bool> running_;
  std::thread server_thread_;
  MessageCallback message_callback_;

  void server_loop();
  void handle_client(int client_socket);
  std::string process_message(const std::string & message);

  // Utility methods
  bool create_socket();
  bool bind_socket();
  bool listen_socket();
  void close_socket();
};

}  // namespace horus_backend
