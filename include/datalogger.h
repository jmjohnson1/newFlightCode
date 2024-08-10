#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <vector>
#include <memory>
#include "common.h"
#include "RingBuf.h"  // Ring buffer used to store values for SD card
#include "SdFat.h"    // Library used for SD card

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)
// Size to log 256 byte lines at 100 Hz for a while
#define LOG_FILE_SIZE 256 * 100 * 600 * 10 /*~1,500,000,000 bytes.*/
// Space to hold more than 1 second of 256-byte lines at 100 Hz in the buffer
#define RING_BUF_CAPACITY 50 * 512

class LogItemBase {};
// The idea is that this would handle items of type int or float
template <typename t>
class LogItem : public LogItemBase {
public:
  LogItem(const t* var, String varName, int digitsOrBase);

  String GetName() {return variableName_;}
  const t* GetPtr() {return variablePtr_;}
  int GetDigitsOrBase() {return digitsOrBase_;}
  t GetValue() {return *variablePtr_;}

private:
  String variableName_ = "";
  const t *variablePtr_ = nullptr;
  int digitsOrBase_ = 10;
};

class Datalogger {
public:
  int Setup();
  // I tried to do this with templates. Total failure.
  // Scalars
  void AddItem(const int* var, String varName, int digitsOrBase);
  void AddItem(const uint64_t* var, String varName, int digitsOrBase);
  void AddItem(const uint32_t* var, String varName, int digitsOrBase);
  void AddItem(const uint16_t* var, String varName, int digitsOrbase);
  void AddItem(const uint8_t* var, String varName, int digitsOrBase);
  void AddItem(const float* var, String varName, int digitsOrBase);
  // Arrays
  void AddItem(const int* var, String varName, int digitsOrBase, size_t size);
  void AddItem(const uint64_t* var, String varName, int digitsOrBase, size_t size);
  void AddItem(const uint32_t* var, String varName, int digitsOrBase, size_t size);
  void AddItem(const uint16_t* var, String varName, int digitsOrBase, size_t size);
  void AddItem(const uint8_t* var, String varName, int digitsOrBase, size_t size);
  void AddItem(const float* var, String varName, int digitsOrBase, size_t size);
  // Vectors
  void AddItem(const Eigen::Ref<const Eigen::VectorXf> &var, String varName, int digitsOrBase);
  void AddItem(const Eigen::Ref<const Eigen::VectorXi> &var, String varName, int digitsOrBase);

  int Write();
  void End();
private:
  // SD Card settings
  String filePrefix = "flight_data";
  String fileExtension = ".csv";
  String fileName;
  void PrintHeader();

  // Yes, this is messy. Look away.
  std::vector<LogItem<float>> itemsFloat_;
  std::vector<LogItem<int>> itemsInt_;
  std::vector<LogItem<uint64_t>> itemsUInt64_;
  std::vector<LogItem<uint32_t>> itemsUInt32_;
  std::vector<LogItem<uint16_t>> itemsUInt16_;
  std::vector<LogItem<uint8_t>> itemsUInt8_;

  FsFile file;
  SdFs sd;
  RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

};


#endif
