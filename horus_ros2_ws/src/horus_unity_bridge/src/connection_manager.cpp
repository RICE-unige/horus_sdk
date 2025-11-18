// SPDX-FileCopyrightText: 2025 RICE Lab, University of Genoa
// SPDX-License-Identifier: Apache-2.0

#include "horus_unity_bridge/connection_manager.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace horus_unity_bridge
{

ConnectionManager::ConnectionManager(const Config& config)
  : config_(config),
    server_fd_(-1),
    epoll_fd_(-1),
    running_(false)
{
  stats_ = Statistics{};
}

ConnectionManager::~ConnectionManager()
{
  stop();
}

bool ConnectionManager::start()
{
  if (running_.load()) {
    return true;
  }
  
  if (!setup_server_socket()) {
    std::cerr << "Failed to setup server socket" << std::endl;
    return false;
  }
  
  if (!setup_epoll()) {
    std::cerr << "Failed to setup epoll" << std::endl;
    close(server_fd_);
    return false;
  }
  
  running_ = true;
  
  // Start accept thread
  accept_thread_ = std::thread(&ConnectionManager::accept_loop, this);
  
  // Start I/O thread
  io_thread_ = std::thread(&ConnectionManager::io_loop, this);
  
  std::cout << "Connection manager started on " << config_.bind_address 
            << ":" << config_.port << std::endl;
  
  return true;
}

void ConnectionManager::stop()
{
  if (!running_.load()) {
    return;
  }
  
  running_ = false;
  
  // Wait for threads to finish
  if (accept_thread_.joinable()) {
    accept_thread_.join();
  }
  
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  
  // Close all client connections
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    for (auto& pair : connections_) {
      close(pair.first);
    }
    connections_.clear();
  }
  
  // Close epoll and server socket
  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }
  
  if (server_fd_ >= 0) {
    close(server_fd_);
    server_fd_ = -1;
  }
  
  std::cout << "Connection manager stopped" << std::endl;
}

bool ConnectionManager::setup_server_socket()
{
  server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd_ < 0) {
    std::cerr << "Failed to create socket: " << strerror(errno) << std::endl;
    return false;
  }
  
  // Set SO_REUSEADDR
  int opt = 1;
  if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }
  
  // Bind
  struct sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(config_.port);
  
  if (config_.bind_address == "0.0.0.0") {
    address.sin_addr.s_addr = INADDR_ANY;
  } else {
    if (inet_pton(AF_INET, config_.bind_address.c_str(), &address.sin_addr) <= 0) {
      std::cerr << "Invalid bind address" << std::endl;
      close(server_fd_);
      return false;
    }
  }
  
  if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
    std::cerr << "Failed to bind: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }
  
  // Listen
  if (listen(server_fd_, config_.max_connections) < 0) {
    std::cerr << "Failed to listen: " << strerror(errno) << std::endl;
    close(server_fd_);
    return false;
  }
  
  // Set non-blocking
  int flags = fcntl(server_fd_, F_GETFL, 0);
  fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
  
  return true;
}

bool ConnectionManager::setup_epoll()
{
  epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
  if (epoll_fd_ < 0) {
    std::cerr << "Failed to create epoll: " << strerror(errno) << std::endl;
    return false;
  }
  
  // Add server socket to epoll
  return add_to_epoll(server_fd_, EPOLLIN);
}

void ConnectionManager::accept_loop()
{
  while (running_.load()) {
    struct epoll_event events[1];
    int nfds = epoll_wait(epoll_fd_, events, 1, 100);  // 100ms timeout
    
    if (nfds < 0) {
      if (errno == EINTR) continue;
      std::cerr << "epoll_wait error in accept: " << strerror(errno) << std::endl;
      break;
    }
    
    for (int i = 0; i < nfds; i++) {
      if (events[i].data.fd == server_fd_) {
        accept_connection();
      }
    }
  }
}

bool ConnectionManager::accept_connection()
{
  struct sockaddr_in client_addr;
  socklen_t addr_len = sizeof(client_addr);
  
  int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &addr_len);
  if (client_fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      std::cerr << "Accept error: " << strerror(errno) << std::endl;
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.connection_errors++;
    }
    return false;
  }
  
  // Configure socket
  configure_socket(client_fd);
  
  // Get client info
  char ip_str[INET_ADDRSTRLEN];
  inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, INET_ADDRSTRLEN);
  uint16_t port = ntohs(client_addr.sin_port);
  
  // Create connection object
  auto connection = std::make_unique<ClientConnection>(client_fd, ip_str, port);
  
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    connections_[client_fd] = std::move(connection);
    
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.total_connections++;
    stats_.active_connections++;
  }
  
  // Add to epoll for I/O
  add_to_epoll(client_fd, EPOLLIN | EPOLLOUT | EPOLLET);
  
  std::cout << "Client connected: " << ip_str << ":" << port << std::endl;
  
  // Notify callback
  if (on_connect_callback_) {
    on_connect_callback_(client_fd, ip_str, port);
  }
  
  return true;
}

void ConnectionManager::io_loop()
{
  const int MAX_EVENTS = 64;
  struct epoll_event events[MAX_EVENTS];
  std::vector<uint8_t> buffer(config_.socket_buffer_size);
  
  while (running_.load()) {
    int nfds = epoll_wait(epoll_fd_, events, MAX_EVENTS, 100);  // 100ms timeout
    
    if (nfds < 0) {
      if (errno == EINTR) continue;
      std::cerr << "epoll_wait error in I/O: " << strerror(errno) << std::endl;
      break;
    }
    
    for (int i = 0; i < nfds; i++) {
      int fd = events[i].data.fd;
      
      // Skip server socket (handled in accept loop)
      if (fd == server_fd_) {
        continue;
      }
      
      if (events[i].events & (EPOLLERR | EPOLLHUP)) {
        disconnect_client(fd);
        continue;
      }
      
      if (events[i].events & EPOLLIN) {
        handle_client_read(fd);
      }
      
      if (events[i].events & EPOLLOUT) {
        handle_client_write(fd);
      }
    }
  }
}

void ConnectionManager::handle_client_read(int client_fd)
{
  std::unique_ptr<ClientConnection>* conn_ptr = nullptr;
  
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(client_fd);
    if (it == connections_.end()) {
      return;
    }
    conn_ptr = &(it->second);
  }
  
  std::vector<uint8_t> buffer(config_.socket_buffer_size);
  
  while (true) {
    ssize_t bytes_read = recv(client_fd, buffer.data(), buffer.size(), 0);
    
    if (bytes_read < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // No more data available
        break;
      }
      // Error - disconnect
      disconnect_client(client_fd);
      return;
    }
    
    if (bytes_read == 0) {
      // Client closed connection
      disconnect_client(client_fd);
      return;
    }
    
    // Parse messages
    std::vector<ProtocolMessage> messages;
    (*conn_ptr)->protocol->parse_messages(buffer.data(), bytes_read, messages);
    
    // Process each message
    for (const auto& msg : messages) {
      if (message_callback_) {
        message_callback_(client_fd, msg);
      }
    }
    
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.messages_received += messages.size();
      stats_.bytes_received += bytes_read;
    }
  }
}

void ConnectionManager::handle_client_write(int client_fd)
{
  std::unique_ptr<ClientConnection>* conn_ptr = nullptr;
  
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(client_fd);
    if (it == connections_.end()) {
      return;
    }
    conn_ptr = &(it->second);
  }
  
  std::lock_guard<std::mutex> queue_lock((*conn_ptr)->queue_mutex);
  
  while (!(*conn_ptr)->send_queue.empty()) {
    auto& data = (*conn_ptr)->send_queue.front();
    
    ssize_t bytes_sent = send(client_fd, data.data(), data.size(), MSG_NOSIGNAL);
    
    if (bytes_sent < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Socket buffer full, try again later
        break;
      }
      // Error - disconnect
      disconnect_client(client_fd);
      return;
    }
    
    if (bytes_sent < static_cast<ssize_t>(data.size())) {
      // Partial send - keep remainder in queue
      data.erase(data.begin(), data.begin() + bytes_sent);
      break;
    }
    
    // Full send - remove from queue
    (*conn_ptr)->send_queue.pop();
    
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.messages_sent++;
      stats_.bytes_sent += bytes_sent;
    }
  }
}

void ConnectionManager::disconnect_client(int client_fd)
{
  std::unique_ptr<ClientConnection> conn;
  
  {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(client_fd);
    if (it == connections_.end()) {
      return;
    }
    conn = std::move(it->second);
    connections_.erase(it);
    
    std::lock_guard<std::mutex> stats_lock(stats_mutex_);
    stats_.active_connections--;
  }
  
  remove_from_epoll(client_fd);
  close(client_fd);
  
  std::cout << "Client disconnected: " << conn->client_ip << ":" << conn->client_port << std::endl;
  
  if (on_disconnect_callback_) {
    on_disconnect_callback_(client_fd, conn->client_ip, conn->client_port);
  }
}

void ConnectionManager::configure_socket(int socket_fd)
{
  // Set TCP_NODELAY
  if (config_.tcp_nodelay) {
    int flag = 1;
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  }
  
  // Set socket buffer sizes
  int bufsize = config_.socket_buffer_size;
  setsockopt(socket_fd, SOL_SOCKET, SO_RCVBUF, &bufsize, sizeof(bufsize));
  setsockopt(socket_fd, SOL_SOCKET, SO_SNDBUF, &bufsize, sizeof(bufsize));
  
  // Set non-blocking
  int flags = fcntl(socket_fd, F_GETFL, 0);
  fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK);
}

bool ConnectionManager::add_to_epoll(int fd, uint32_t events)
{
  struct epoll_event ev;
  memset(&ev, 0, sizeof(ev));
  ev.events = events;
  ev.data.fd = fd;
  
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &ev) < 0) {
    std::cerr << "Failed to add fd to epoll: " << strerror(errno) << std::endl;
    return false;
  }
  
  return true;
}

bool ConnectionManager::modify_epoll(int fd, uint32_t events)
{
  struct epoll_event ev;
  memset(&ev, 0, sizeof(ev));
  ev.events = events;
  ev.data.fd = fd;
  
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, fd, &ev) < 0) {
    std::cerr << "Failed to modify fd in epoll: " << strerror(errno) << std::endl;
    return false;
  }
  
  return true;
}

bool ConnectionManager::remove_from_epoll(int fd)
{
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr) < 0) {
    // Don't log error if fd is already removed
    if (errno != ENOENT) {
      std::cerr << "Failed to remove fd from epoll: " << strerror(errno) << std::endl;
    }
    return false;
  }
  
  return true;
}

bool ConnectionManager::send_to_client(int client_fd, const std::string& destination,
                                      const std::vector<uint8_t>& payload)
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  auto it = connections_.find(client_fd);
  if (it == connections_.end()) {
    return false;
  }
  
  auto serialized = it->second->protocol->serialize_message(destination, payload);
  
  std::lock_guard<std::mutex> queue_lock(it->second->queue_mutex);
  it->second->send_queue.push(std::move(serialized));
  
  // Trigger write event
  modify_epoll(client_fd, EPOLLIN | EPOLLOUT | EPOLLET);
  
  return true;
}

void ConnectionManager::broadcast_message(const std::string& destination,
                                         const std::vector<uint8_t>& payload)
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  
  for (auto& pair : connections_) {
    auto serialized = pair.second->protocol->serialize_message(destination, payload);
    
    std::lock_guard<std::mutex> queue_lock(pair.second->queue_mutex);
    pair.second->send_queue.push(std::move(serialized));
    
    modify_epoll(pair.first, EPOLLIN | EPOLLOUT | EPOLLET);
  }
}

size_t ConnectionManager::get_connection_count() const
{
  std::lock_guard<std::mutex> lock(connections_mutex_);
  return connections_.size();
}

ConnectionManager::Statistics ConnectionManager::get_statistics() const
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

} // namespace horus_unity_bridge
