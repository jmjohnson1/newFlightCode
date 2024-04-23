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
  LogItem(t* const var, String varName, int digitsOrBase);

  String GetName() {return variableName_;}
  t* GetPtr() {return variablePtr_;}
  int GetDigitsOrBase() {return digitsOrBase_;}
  t GetValue() {return *variablePtr_;}

private:
  String variableName_ = "";
  t *variablePtr_ = nullptr;
  int digitsOrBase_ = 10;
};

class Datalogger {
public:
  int Setup();
  // I tried to do this with templates. Total failure.
  void AddItem(int* const var, String varName, int digitsOrBase);
  void AddItem(float* const var, String varName, int digitsOrBase);
  void AddItem(int* const var, String varName, int digitsOrBase, size_t size);
  void AddItem(float* const var, String varName, int digitsOrBase, size_t size);
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
  std::vector<LogItem<float>> itemsFloat_;
  std::vector<LogItem<int>> itemsInt_;
  FsFile file;
  SdFs sd;
  RingBuf<FsFile, RING_BUF_CAPACITY> buffer;

};


#endif