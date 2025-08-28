#ifndef EEPROM_SIM_H
#define EEPROM_SIM_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <filesystem>


class EEPROMSimulator {
private:
  std::string filename;
  size_t memory_size;
  std::vector<uint8_t> memory;

public:
  EEPROMSimulator(const std::string& file_path, size_t size)
    : filename(file_path), memory_size(size) {
    memory.resize(memory_size, 0xFF); // Initialize memory with all 1
    loadFromFile(); // This won't actually change anything if the file doesn't exist yet
  }

  ~EEPROMSimulator() {
    saveToFile();
  }

  // Read a single byte at an address
  uint8_t read(size_t address) {
    if (address >= memory_size) {
      throw std::out_of_range("Address out of range");
    }
    return memory[address];
  }

  // Write a single byte at an address
  void write(size_t address, uint8_t data) {
    if (address >= memory_size) {
      throw std::out_of_range("Address out of range");
    }
    memory[address] = data;
    saveToFile();
  }

  void loadFromFile() {
    if (!std::filesystem::exists(filename)) {
      // File doesn't exist, keep the default memory
      return;
    }

    std::ifstream file(filename, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Cannot open EEPROM file for reading: " + filename);
    }

    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t bytes_to_read = std::min(file_size, memory_size);
    // Using reinterpret_cast since char* and uint8_t are unrelated types. Still
    // same size and nothing risky going on, so this is safe.
    file.read(reinterpret_cast<char*>(memory.data()), bytes_to_read);
  }


  void saveToFile() {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Cannot open EEPROM file for writing: " + filename);
    }

    file.write(reinterpret_cast<const char*>(memory.data()), memory_size);
    if (!file.good()) {
      throw std::runtime_error("Error writing to file: " + filename);
    }
  }
};

static EEPROMSimulator EEPROM("eeprom_data.bin", 1024);

#endif // EEPROM_SIM_H
