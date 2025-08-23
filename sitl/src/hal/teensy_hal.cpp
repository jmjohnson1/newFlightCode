#ifdef SITL_BUILD

#include "teensy_hal.h"
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <random>

// Global time reference
static std::chrono::steady_clock::time_point sim_start_time = std::chrono::steady_clock::now();

// Arduino-like timing functions
uint32_t micros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - sim_start_time);
    return static_cast<uint32_t>(duration.count());
}

uint32_t millis() {
    return micros() / 1000;
}

void delay(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void delayMicroseconds(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

float constrain(float value, float min_val, float max_val) {
    return std::max(min_val, std::min(value, max_val));
}

// Digital I/O simulation
static bool pin_states[64] = {false}; // Assuming max 64 pins
static uint8_t pin_modes[64] = {INPUT};

void pinMode(uint8_t pin, uint8_t mode) {
    if (pin < 64) {
        pin_modes[pin] = mode;
    }
}

void digitalWrite(uint8_t pin, uint8_t value) {
    if (pin < 64 && pin_modes[pin] == OUTPUT) {
        pin_states[pin] = (value == HIGH);
        // For LED pin, you could add visual indication
        if (pin == 10 || pin == 13) { // Common LED pins
            std::cout << (pin_states[pin] ? "LED ON\n" : "LED OFF\n");
        }
    }
}

int digitalRead(uint8_t pin) {
    if (pin < 64) {
        return pin_states[pin] ? HIGH : LOW;
    }
    return LOW;
}

// Serial implementation
SITLSerial Serial;
SITLSerial Serial5;

void SITLSerial::begin(uint32_t baud) {
    std::cout << "Serial initialized at " << baud << " baud\n";
}

void SITLSerial::print(const char* str) {
    std::cout << str;
}

void SITLSerial::print(float val) {
    std::cout << std::fixed << std::setprecision(6) << val;
}

void SITLSerial::print(int val) {
    std::cout << val;
}

void SITLSerial::println(const char* str) {
    std::cout << str << std::endl;
}

void SITLSerial::println(float val) {
    std::cout << std::fixed << std::setprecision(6) << val << std::endl;
}

void SITLSerial::println(int val) {
    std::cout << val << std::endl;
}

void SITLSerial::println() {
    std::cout << std::endl;
}

bool SITLSerial::available() {
    // Simple implementation - could be enhanced with actual input buffering
    return false;
}

char SITLSerial::read() {
    return 0;
}

// Wire (I2C) simulation
SITLWire Wire;

void SITLWire::begin() {
    std::cout << "I2C initialized\n";
}

void SITLWire::beginTransmission(uint8_t address) {
    std::cout << "I2C begin transmission to 0x" << std::hex << (int)address << std::dec << std::endl;
}

uint8_t SITLWire::endTransmission() {
    return 0; // Success
}

uint8_t SITLWire::write(uint8_t data) {
    return 1; // Bytes written
}

uint8_t SITLWire::requestFrom(uint8_t address, uint8_t quantity) {
    return quantity; // Simulate successful read
}

uint8_t SITLWire::read() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 255);
    return dis(gen); // Random data for simulation
}

bool SITLWire::available() {
    return false;
}

// SPI simulation
SITLSPI SPI;

void SITLSPI::begin() {
    std::cout << "SPI initialized\n";
}

void SITLSPI::end() {
    std::cout << "SPI ended\n";
}

void SITLSPI::setBitOrder(uint8_t bitOrder) {
    // No-op for simulation
}

void SITLSPI::setDataMode(uint8_t dataMode) {
    // No-op for simulation
}

void SITLSPI::setClockDivider(uint8_t clockDiv) {
    // No-op for simulation
}

uint8_t SITLSPI::transfer(uint8_t data) {
    // Echo back data or return simulated sensor data
    return data;
}

#endif // SITL_BUILD