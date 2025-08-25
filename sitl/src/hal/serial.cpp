#ifdef SITL_BUILD

#include <cstring>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <thread>

#include "teensy_hal.h"

// Enhanced Serial implementation with buffering and formatting

class SITLSerialImpl {
 public:
  SITLSerialImpl(const std::string& name) : name_(name), baud_rate_(9600), initialized_(false) {}

  void begin(uint32_t baud) {
    baud_rate_ = baud;
    initialized_ = true;
    std::cout << "[" << name_ << "] Serial initialized at " << baud << " baud" << std::endl;
  }

  void end() {
    initialized_ = false;
    std::cout << "[" << name_ << "] Serial ended" << std::endl;
  }

  // Print functions
  void print(const char* str) {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << str << std::flush;
  }

  void print(const std::string& str) { print(str.c_str()); }

  void print(char c) {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << c << std::flush;
  }

  void print(int val, int base = 10) {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (base == 16) {
      std::cout << "0x" << std::hex << val << std::dec;
    } else if (base == 8) {
      std::cout << std::oct << val << std::dec;
    } else if (base == 2) {
      // Binary output
      std::cout << "0b";
      for (int i = 31; i >= 0; --i) {
        std::cout << ((val >> i) & 1);
      }
    } else {
      std::cout << val;
    }
    std::cout << std::flush;
  }

  void print(unsigned int val, int base = 10) { print(static_cast<int>(val), base); }

  void print(long val, int base = 10) { print(static_cast<int>(val), base); }

  void print(unsigned long val, int base = 10) { print(static_cast<int>(val), base); }

  void print(float val, int digits = 2) {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << std::fixed << std::setprecision(digits) << val << std::flush;
  }

  void print(double val, int digits = 2) { print(static_cast<float>(val), digits); }

  // Println functions
  void println() {
    if (!initialized_) return;
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << std::endl;
  }

  void println(const char* str) {
    print(str);
    println();
  }

  void println(const std::string& str) { println(str.c_str()); }

  void println(char c) {
    print(c);
    println();
  }

  void println(int val, int base = 10) {
    print(val, base);
    println();
  }

  void println(unsigned int val, int base = 10) {
    print(val, base);
    println();
  }

  void println(long val, int base = 10) {
    print(val, base);
    println();
  }

  void println(unsigned long val, int base = 10) {
    print(val, base);
    println();
  }

  void println(float val, int digits = 2) {
    print(val, digits);
    println();
  }

  void println(double val, int digits = 2) {
    print(val, digits);
    println();
  }

  /*// Printf-style formatting*/
  /*void printf(const char* format, ...) {*/
  /*  if (!initialized_) return;*/
  /**/
  /*  std::va_list args;*/
  /*  std::va_start(args, format);*/
  /**/
  /*  char buffer[512];*/
  /*  vsnprintf(buffer, sizeof(buffer), format, args);*/
  /**/
  /*  std::va_end(args);*/
  /**/
  /*  print(buffer);*/
  /*}*/

  // Write functions
  size_t write(uint8_t c) {
    print(static_cast<char>(c));
    return 1;
  }

  size_t write(const uint8_t* buffer, size_t size) {
    if (!initialized_) return 0;
    std::lock_guard<std::mutex> lock(output_mutex_);

    for (size_t i = 0; i < size; ++i) {
      std::cout << static_cast<char>(buffer[i]);
    }
    std::cout << std::flush;

    return size;
  }

  size_t write(const char* buffer, size_t size) { return write(reinterpret_cast<const uint8_t*>(buffer), size); }

  // Input functions (basic implementation)
  bool available() {
    std::lock_guard<std::mutex> lock(input_mutex_);
    return !input_buffer_.empty();
  }

  int read() {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_buffer_.empty()) {
      return -1;
    }

    char c = input_buffer_.front();
    input_buffer_.pop();
    return static_cast<int>(c);
  }

  int peek() {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_buffer_.empty()) {
      return -1;
    }
    return static_cast<int>(input_buffer_.front());
  }

  void flush() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    std::cout << std::flush;
  }

  // Buffer management
  void addInputData(const std::string& data) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    for (char c : data) {
      input_buffer_.push(c);
    }
  }

  operator bool() const { return initialized_; }

 private:
  std::string name_;
  uint32_t baud_rate_;
  bool initialized_;

  std::mutex output_mutex_;
  std::mutex input_mutex_;
  std::queue<char> input_buffer_;
};

// Global serial instances
static SITLSerialImpl serial_usb("USB");
static SITLSerialImpl serial5_hw("HW5");

// SITLSerial wrapper implementation
void SITLSerial::begin(uint32_t baud) {
  if (this == &Serial) {
    serial_usb.begin(baud);
  } else if (this == &Serial5) {
    serial5_hw.begin(baud);
  }
}

void SITLSerial::print(const char* str) {
  if (this == &Serial) {
    serial_usb.print(str);
  } else if (this == &Serial5) {
    serial5_hw.print(str);
  }
}

void SITLSerial::print(float val) {
  if (this == &Serial) {
    serial_usb.print(val);
  } else if (this == &Serial5) {
    serial5_hw.print(val);
  }
}

void SITLSerial::print(int val) {
  if (this == &Serial) {
    serial_usb.print(val);
  } else if (this == &Serial5) {
    serial5_hw.print(val);
  }
}

void SITLSerial::println(const char* str) {
  if (this == &Serial) {
    serial_usb.println(str);
  } else if (this == &Serial5) {
    serial5_hw.println(str);
  }
}

void SITLSerial::println(float val) {
  if (this == &Serial) {
    serial_usb.println(val);
  } else if (this == &Serial5) {
    serial5_hw.println(val);
  }
}

void SITLSerial::println(int val) {
  if (this == &Serial) {
    serial_usb.println(val);
  } else if (this == &Serial5) {
    serial5_hw.println(val);
  }
}

void SITLSerial::println() {
  if (this == &Serial) {
    serial_usb.println();
  } else if (this == &Serial5) {
    serial5_hw.println();
  }
}

bool SITLSerial::available() {
  if (this == &Serial) {
    return serial_usb.available();
  } else if (this == &Serial5) {
    return serial5_hw.available();
  }
  return false;
}

char SITLSerial::read() {
  if (this == &Serial) {
    return static_cast<char>(serial_usb.read());
  } else if (this == &Serial5) {
    return static_cast<char>(serial5_hw.read());
  }
  return 0;
}

// Helper functions for input simulation (useful for testing)
void simulateSerialInput(const std::string& data) { serial_usb.addInputData(data); }

void simulateSerial5Input(const std::string& data) { serial5_hw.addInputData(data); }

// Extended Serial interface (if your code uses these)
extern "C" {
void serial_print_float_precision(float val, int precision) { serial_usb.print(val, precision); }

void serial_print_hex(int val) { serial_usb.print(val, 16); }

void serial_print_binary(int val) { serial_usb.print(val, 2); }
}

#endif  // SITL_BUILD
