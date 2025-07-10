// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_backend/tcp_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace horus_backend
{

TcpServer::TcpServer(int port)
: port_(port), server_socket_(-1), running_(false) {}

TcpServer::~TcpServer() {stop();}

void TcpServer::start()
{
  if (running_) {
    return;
  }

  if (!create_socket() || !bind_socket() || !listen_socket()) {
    throw std::runtime_error("Failed to start TCP server");
  }

  running_ = true;
  server_thread_ = std::thread(&TcpServer::server_loop, this);

  std::cout << "TCP Server started on port " << port_ << std::endl;
}

void TcpServer::stop()
{
  if (!running_) {
    return;
  }

  running_ = false;

  if (server_thread_.joinable()) {
    server_thread_.join();
  }

  close_socket();
  std::cout << "TCP Server stopped" << std::endl;
}

bool TcpServer::is_running() const {return running_;}

void TcpServer::set_message_callback(MessageCallback callback)
{
  message_callback_ = callback;
}

bool TcpServer::create_socket()
{
  server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  // Set socket options
  int opt = 1;
  if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) <
    0)
  {
    std::cerr << "Failed to set socket options" << std::endl;
    return false;
  }

  return true;
}

bool TcpServer::bind_socket()
{
  struct sockaddr_in address;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port_);

  if (bind(server_socket_, (struct sockaddr *)&address, sizeof(address)) < 0) {
    std::cerr << "Failed to bind socket to port " << port_ << std::endl;
    return false;
  }

  return true;
}

bool TcpServer::listen_socket()
{
  if (listen(server_socket_, 3) < 0) {
    std::cerr << "Failed to listen on socket" << std::endl;
    return false;
  }

  return true;
}

void TcpServer::close_socket()
{
  if (server_socket_ >= 0) {
    close(server_socket_);
    server_socket_ = -1;
  }
}

void TcpServer::server_loop()
{
  while (running_) {
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    int client_socket =
      accept(server_socket_, (struct sockaddr *)&address, &addrlen);
    if (client_socket < 0) {
      if (running_) {
        std::cerr << "Failed to accept client connection" << std::endl;
      }
      continue;
    }

    std::cout << "Client connected from " << inet_ntoa(address.sin_addr)
              << std::endl;

    // Handle client in a separate thread
    std::thread client_thread(&TcpServer::handle_client, this, client_socket);
    client_thread.detach();
  }
}

void TcpServer::handle_client(int client_socket)
{
  char buffer[1024] = {0};

  while (running_) {
    ssize_t bytes_read = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
    if (bytes_read <= 0) {
      break;
    }

    buffer[bytes_read] = '\0';
    std::string message(buffer);

    // Process message
    std::string response = process_message(message);

    // Send response
    send(client_socket, response.c_str(), response.length(), 0);
  }

  close(client_socket);
  std::cout << "Client disconnected" << std::endl;
}

std::string TcpServer::process_message(const std::string & message)
{
  if (message_callback_) {
    return message_callback_(message);
  }

  // Default response
  return "{\"status\": \"ok\", \"message\": \"Message received\"}";
}

}  // namespace horus_backend
