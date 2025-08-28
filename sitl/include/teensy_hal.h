#ifndef TEENSY_HAL_H
#define TEENSY_HAL_H

#ifdef SITL_BUILD

#include <cstdint>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>

// Teensy Arduino compatibility layer for SITL

// Basic types
typedef uint8_t byte;

// Math constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// Arduino-like functions
uint32_t micros();
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
float constrain(float value, float min_val, float max_val);

// Digital I/O
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);

// Serial communication mock
class SITLSerial {
public:
    void begin(uint32_t baud);
    void print(const char* str);
    void print(float val);
    void print(int val);
    void println(const char* str);
    void println(float val);
    void println(int val);
    void println();
    bool available();
    char read();
};

extern SITLSerial Serial;
extern SITLSerial Serial5;
extern SITLSerial Serial2;

// elapsedMicros simulation
class elapsedMicros {
private:
    std::chrono::steady_clock::time_point start_time;
    
public:
    elapsedMicros() : start_time(std::chrono::steady_clock::now()) {}
    
    operator uint32_t() const {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time);
        return static_cast<uint32_t>(duration.count());
    }
    
    elapsedMicros& operator=(uint32_t val) {
        if (val == 0) {
            start_time = std::chrono::steady_clock::now();
        } else {
            start_time = std::chrono::steady_clock::now() - std::chrono::microseconds(val);
        }
        return *this;
    }
};

// Wire library mock (for I2C)
class SITLWire {
public:
    void begin();
    void beginTransmission(uint8_t address);
    uint8_t endTransmission();
    uint8_t write(uint8_t data);
    uint8_t requestFrom(uint8_t address, uint8_t quantity);
    uint8_t read();
    bool available();
};

extern SITLWire Wire;

// SPI library mock
class SITLSPI {
public:
    void begin();
    void end();
    void setBitOrder(uint8_t bitOrder);
    void setDataMode(uint8_t dataMode);
    void setClockDivider(uint8_t clockDiv);
    uint8_t transfer(uint8_t data);
};

extern SITLSPI SPI;

// SBUS mock - will be connected to virtual RC transmitter
class SITLSBUS {
private:
    bool initialized = false;
    
public:
    void begin();
    bool read(uint16_t* channels, bool* failSafe, bool* lostFrame);
};

// CPU restart (do nothing in SITL)
#define CPU_RESTART_ADDR (uint32_t *)0x0
#define CPU_RESTART_VAL 0x0
#define CPU_RESTART {}

// Math functions
using std::abs;
using std::sin;
using std::cos;
using std::tan;
using std::asin;
using std::acos;
using std::atan;
using std::atan2;
using std::sqrt;
using std::pow;
using std::isnan;

#endif // SITL_BUILD
#endif // TEENSY_HAL_H
