#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>

class NetworkSerial {
 public:
  NetworkSerial(uint16_t port = 12345, size_t rx_buffer_size = 1024,
                size_t tx_buffer_size = 1024);
  ~NetworkSerial();

  void begin(uint32_t baud = 115200, uint16_t format = 0);
  void end();

  int available();
  int availableForWrite();
  int peek();
  int read();
  void flush();

  void addMemoryForRead(uint8_t *buffer, size_t length) {
    // no-op. Should just adjust buffer size in constructor for this
  }
  void addMemoryForWrite(uint8_t *buffer, size_t length) {
    // no-op. Should just adjust buffer size in constructor for this
  }

  size_t write(uint8_t c);
  size_t write(const uint8_t *buffer, size_t size);

  void clear();

  bool isConnected() const { return client_connected_; }
  uint16_t getPort() const { return port_; }

 private:
  static const size_t DEFAULT_BUFFER_SIZE = 1024;

  uint16_t port_;
  int udp_socket_;
  struct sockaddr_in client_addr_;
  bool has_client_;
  std::atomic<bool> server_running_;
  std::atomic<bool> client_connected_;
  std::atomic<bool> should_stop_;

  uint8_t *rx_buffer_;
  uint8_t *tx_buffer_;
  size_t rx_buffer_size_;
  size_t tx_buffer_size_;

  // Locations of head and tail
  // Making these atomic since they are used b/w threads
  // That said, ioloop only modifies rx head and tx tail
  // while write and read only use tx head and rx tail.
  std::atomic<size_t> rx_buffer_head_;
  std::atomic<size_t> rx_buffer_tail_;
  std::atomic<size_t> tx_buffer_head_;
  std::atomic<size_t> tx_buffer_tail_;

  std::thread io_thread_;

  // Needed to lock threads while clearing
  mutable std::mutex buffer_mutex_;

  void ioLoop();

  void setupSocket();
  void closeSocket();
};
