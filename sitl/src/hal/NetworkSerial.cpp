#include "NetworkSerial.h"

#include <errno.h>
#include <fcntl.h>

#include <cstring>
#include <iostream>

NetworkSerial::NetworkSerial(uint16_t port, size_t rx_buffer_size,
                             size_t tx_buffer_size)
    : port_(port),
      udp_socket_(-1),
      has_client_(false),
      server_running_(false),
      client_connected_(false),
      should_stop_(false),
      rx_buffer_size_(rx_buffer_size),
      tx_buffer_size_(tx_buffer_size),
      rx_buffer_head_(0),
      rx_buffer_tail_(0),
      tx_buffer_head_(0),
      tx_buffer_tail_(0) {
  rx_buffer_ = new uint8_t[rx_buffer_size_];
  tx_buffer_ = new uint8_t[tx_buffer_size_];
}

NetworkSerial::~NetworkSerial() {
  end();
  delete[] rx_buffer_;
  delete[] tx_buffer_;
}

void NetworkSerial::begin(uint32_t baud, uint16_t format) {
  if (server_running_) return;

  std::cout << "begin called" << std::endl;
  rx_buffer_head_ = 0;
  rx_buffer_tail_ = 0;
  tx_buffer_head_ = 0;
  tx_buffer_tail_ = 0;

  setupSocket();

  server_running_ = true;
  should_stop_ = false;

  // Run this continually in a different thread
  io_thread_ = std::thread(&NetworkSerial::ioLoop, this);
}

void NetworkSerial::end() {
  should_stop_ = true;
  server_running_ = false;
  client_connected_ = false;

  if (io_thread_.joinable()) io_thread_.join();

  closeSocket();
}

void NetworkSerial::setupSocket() {
  udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_ < 0) {
    std::cerr << "UDP socket creation failed: " << strerror(errno) << std::endl;
    return;
  }

  int opt = 1;
  setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in address; // internet socket address
  address.sin_family = AF_INET;  // I think this is IPv4
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port_); // Have to convert this to network byte order

  /*if (bind(udp_socket_, (struct sockaddr *)&address, sizeof(address)) < 0) {*/
  /*  // A return value of -1 indicates a failure*/
  /*  std::cerr << "UDP bind failed: " << strerror(errno) << std::endl;*/
  /*  close(udp_socket_);*/
  /*  udp_socket_ = -1;*/
  /*  return;*/
  /*}*/

  has_client_ = false;
  memset(&client_addr_, 0, sizeof(client_addr_));

  std::cout << "NetworkSerial UDP broadcasting on port " << port_ << std::endl;
}

void NetworkSerial::closeSocket() {
  if (udp_socket_ >= 0) {
    close(udp_socket_);
    udp_socket_ = -1;
  }
  has_client_ = false;
}

void NetworkSerial::ioLoop() {
  while (!should_stop_) {
    if (udp_socket_ < 0) { // Socket not created
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    size_t head = tx_buffer_head_;
    size_t tail = tx_buffer_tail_;

    struct sockaddr_in address; // internet socket address
    address.sin_family = AF_INET;  // I think this is IPv4
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_); // Have to convert this to network byte order

    // Transmit
    if (head != tail) {
      uint8_t tx_temp_buffer[256];
      size_t bytes_to_send = 0;

      while (head != tail && bytes_to_send < sizeof(tx_temp_buffer)) {
        if (++tail >= tx_buffer_size_) tail = 0;
        tx_temp_buffer[bytes_to_send++] = tx_buffer_[tail];
      }

      if (bytes_to_send > 0) {
        std::cout << "bytes to send: " << bytes_to_send << std::endl;
        ssize_t bytes_sent =
            sendto(udp_socket_, tx_temp_buffer, bytes_to_send, 0,
                   (struct sockaddr *)&address, sizeof(address));
        if (bytes_sent > 0) {
          std::cout << "bytes sent: " << bytes_sent << std::endl;
          size_t actual_tail = tx_buffer_tail_;
          for (ssize_t i = 0; i < bytes_sent; i++) {
            if (++actual_tail >= tx_buffer_size_) actual_tail = 0;
          }
          tx_buffer_tail_ = actual_tail;
        }
      }
    }

    // Receive
    uint8_t temp_buffer[256];
    struct sockaddr_in from_addr;
    socklen_t from_len = sizeof(from_addr);

    ssize_t bytes_received =
        recvfrom(udp_socket_, temp_buffer, sizeof(temp_buffer), MSG_DONTWAIT,
                 (struct sockaddr *)&from_addr, &from_len);

    if (bytes_received > 0) {
      if (!has_client_) {
        client_addr_ = from_addr;
        has_client_ = true;
        client_connected_ = true;
        std::cout << "UDP client connected from "
                  << inet_ntoa(from_addr.sin_addr) << ":"
                  << ntohs(from_addr.sin_port) << std::endl;
      }

      for (ssize_t i = 0; i < bytes_received; i++) {
        size_t head = rx_buffer_head_;
        size_t next_head = head + 1;
        if (next_head >= rx_buffer_size_) next_head = 0;

        if (next_head != rx_buffer_tail_) {
          rx_buffer_[next_head] = temp_buffer[i];
          rx_buffer_head_ = next_head;
        }
      }
    }


    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

int NetworkSerial::available() {
  size_t head = rx_buffer_head_;
  size_t tail = rx_buffer_tail_;

  if (head >= tail) {
    return head - tail;
  } else {
    return rx_buffer_size_ + head - tail;
  }
}

int NetworkSerial::availableForWrite() {
  size_t head = tx_buffer_head_;
  size_t tail = tx_buffer_tail_;

  if (head >= tail) {
    return tx_buffer_size_ - 1 - head + tail;
  }
  return tail - head - 1;
}

int NetworkSerial::peek() {
  size_t head = rx_buffer_head_;
  size_t tail = rx_buffer_tail_;

  if (head == tail) return -1;

  size_t next_tail = tail + 1;
  if (next_tail >= rx_buffer_size_) next_tail = 0;

  return rx_buffer_[next_tail];
}

int NetworkSerial::read() {
  size_t head = rx_buffer_head_;
  size_t tail = rx_buffer_tail_;

  if (head == tail) return -1;

  if (++tail >= rx_buffer_size_) tail = 0;
  int c = rx_buffer_[tail];
  rx_buffer_tail_ = tail;

  return c;
}

void NetworkSerial::flush() {
  while (tx_buffer_head_ != tx_buffer_tail_ && has_client_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

size_t NetworkSerial::write(uint8_t c) {
  /*if (!has_client_) return 0;*/

  size_t head = tx_buffer_head_;
  size_t next_head = head + 1;
  if (next_head >= tx_buffer_size_) next_head = 0;

  while (tx_buffer_tail_ == next_head && has_client_) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  /*if (!has_client_) return 0;*/

  tx_buffer_[next_head] = c;
  tx_buffer_head_ = next_head;

  return 1;
}

size_t NetworkSerial::write(const uint8_t *buffer, size_t size) {
  size_t written = 0;
  for (size_t i = 0; i < size; i++) {
    if (write(buffer[i]) == 0) break;
    written++;
  }
  return written;
}

void NetworkSerial::clear() {
  // I think this needs to be locked because it isn't safe
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  rx_buffer_head_ = rx_buffer_tail_.load();
}
