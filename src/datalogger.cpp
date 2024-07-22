#include "datalogger.h"


template<typename t>
LogItem<t>::LogItem(const t* var, String varName, int digitsOrBase) {
  variablePtr_ = var;
  variableName_ = varName;
  digitsOrBase_ = digitsOrBase;
}
  
// Scalars
void Datalogger::AddItem(const int* var, String varName, int digitsOrBase) {
  itemsInt_.push_back(LogItem<int>(var, varName, digitsOrBase));
}
void Datalogger::AddItem(const uint64_t* var, String varName, int digitsOrBase) {
  itemsUInt64_.push_back(LogItem<uint64_t>(var, varName, digitsOrBase));
}
void Datalogger::AddItem(const uint32_t* var, String varName, int digitsOrBase) {
  itemsUInt32_.push_back(LogItem<uint32_t>(var, varName, digitsOrBase));
}
void Datalogger::AddItem(const uint16_t* var, String varName, int digitsOrBase) {
  itemsUInt16_.push_back(LogItem<uint16_t>(var, varName, digitsOrBase));
}
void Datalogger::AddItem(const uint8_t* var, String varName, int digitsOrBase) {
  itemsUInt8_.push_back(LogItem<uint8_t>(var, varName, digitsOrBase));
}
void Datalogger::AddItem(const float* var, String varName, int digitsOrBase) {
  itemsFloat_.push_back(LogItem<float>(var, varName, digitsOrBase));
}

// Arrays
void Datalogger::AddItem(const int* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsInt_.push_back(LogItem<int>(&(var[i]), varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const uint64_t* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsUInt64_.push_back(LogItem<uint64_t>(&(var[i]), varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const uint32_t* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsUInt32_.push_back(LogItem<uint32_t>(&(var[i]), varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const uint16_t* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsUInt16_.push_back(LogItem<uint16_t>(&(var[i]), varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const uint8_t* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsUInt8_.push_back(LogItem<uint8_t>(&(var[i]), varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const float* var, String varName, int digitsOrBase, size_t size) {
  for (int i = 0; i < size; i++) {
    String varName_i = varName + String(i);
    itemsFloat_.push_back(LogItem<float>(&(var[i]), varName_i, digitsOrBase));
  }
}

// Vectors
void Datalogger::AddItem(const Eigen::Ref<const Eigen::VectorXf> &var, String varName, int digitsOrBase) {
  const float* dat = var.data();
  for (int i = 0; i < var.rows(); i++) {
    String varName_i = varName + String(i);
    itemsFloat_.push_back(LogItem<float>(&dat[i], varName_i, digitsOrBase));
  }
}
void Datalogger::AddItem(const Eigen::Ref<const Eigen::VectorXi> &var, String varName, int digitsOrBase) {
  const int* dat = var.data();
  for (int i = 0; i < var.rows(); i++) {
    String varName_i = varName + String(i);
    itemsInt_.push_back(LogItem<int>(&dat[i], varName_i, digitsOrBase));
  }
}

/**
 * @brief Prints the header line for the datalogger csv file
*/
void Datalogger::PrintHeader() {
  // Write the name of each item from the vector
  for (int i = 0; i < itemsFloat_.size(); i++) {
    if (i != 0) {
      buffer.write(",");
    }
    buffer.print(itemsFloat_[i].GetName());
  }
  for (int i = 0; i < itemsInt_.size(); i++) {
    // I'm assuming some float items exist, so printing commas
    buffer.write(",");
    buffer.print(itemsInt_[i].GetName());
  }
  for (int i = 0; i < itemsUInt64_.size(); i++) {
    buffer.write(",");
    buffer.print(itemsUInt64_[i].GetName());
  }
  for (int i = 0; i < itemsUInt32_.size(); i++) {
    buffer.write(",");
    buffer.print(itemsUInt32_[i].GetName());
  }
  for (int i = 0; i < itemsUInt16_.size(); i++) {
    buffer.write(",");
    buffer.print(itemsUInt16_[i].GetName());
  }
  for (int i = 0; i < itemsUInt8_.size(); i++) {
    buffer.write(",");
    buffer.print(itemsUInt8_[i].GetName());
  }
  buffer.println();
}

/**
 * @brief Initializes SD card logging
 * @returns 1 if there was an error communicating with the SD card or creating
 * the file. 0 if successful
*/
int Datalogger::Setup() {
  // Initialize the SD
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorPrint(&Serial); // Prints message to serial if SD can't init
    return 1;
  }
  // Determine logfile name
  int fileIncrement = 0;
  fileName = filePrefix + String(fileIncrement) + fileExtension;
  while(sd.exists(fileName)) {
    // Increment file name if it exists and try again
    fileIncrement++;
    fileName = filePrefix + String(fileIncrement) + fileExtension;
  }
  //Open or create file - truncate existing
  if (!file.open(fileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("open failed\n");
    return 1;
  }
  // Initialize ring buffer
  buffer.begin(&file);
  Serial.println("Buffer initialized");
  PrintHeader();
  return 0;
}

/**
 * @brief Writes the data to the FIFO buffer used for the sd card. Once the
 * amount of data in the FIFO buffer has reached the SD sector size (512
 * bytes), the data is written to the SD card.
 * @returns 1 if the log file is full or the data buffer failed to write to
 * the SD card. 0 if successful.
*/
int Datalogger::Write() {
  size_t amtDataInBuf = buffer.bytesUsed();
  
  if ((amtDataInBuf + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
    Serial.println("Log file full -- No longer writing");
    return 1;
  }
  if (amtDataInBuf >= 1024 && !file.isBusy()) {
    // One sector (512 bytes) can be printed before busy wait
    // Write from buffer to file
    if (1024 != buffer.writeOut(1024)) {
      Serial.println("Write to file from buffer failed -- breaking");
      return 1;
    }
  }

  // Writing
  for (int i = 0; i < itemsFloat_.size(); i++) {
    if (i != 0) {
      buffer.write(",");
    }
    buffer.print(*(itemsFloat_[i].GetPtr()), itemsFloat_[i].GetDigitsOrBase());
  }
  for (int i = 0; i < itemsInt_.size(); i++) {
    // I'm assuming some float items exist, so printing commas
    buffer.write(",");
    buffer.print(*(itemsInt_[i].GetPtr()), itemsInt_[i].GetDigitsOrBase());
  }
  for (int i = 0; i < itemsUInt64_.size(); i++) {
    buffer.write(",");
    buffer.print(*(itemsUInt64_[i].GetPtr()), itemsUInt64_[i].GetDigitsOrBase());
  }
  for (int i = 0; i < itemsUInt32_.size(); i++) {
    buffer.write(",");
    buffer.print(*(itemsUInt32_[i].GetPtr()), itemsUInt32_[i].GetDigitsOrBase());
  }
  for (int i = 0; i < itemsUInt16_.size(); i++) {
    buffer.write(",");
    buffer.print(*(itemsUInt16_[i].GetPtr()), itemsUInt16_[i].GetDigitsOrBase());
  }
  for (int i = 0; i < itemsUInt8_.size(); i++) {
    buffer.write(",");
    buffer.print(*(itemsUInt8_[i].GetPtr()), itemsUInt8_[i].GetDigitsOrBase());
  }
  buffer.println();

  if (buffer.getWriteError()) {
    Serial.println("WriteError");
    return 1;
  }
  return 0;
}

/**
 * @brief Ends the process of writing to the SD card and closes the file. This
 * must be called in order for the logfile to save.
*/

void Datalogger::End() {
  // Write any remaining buffer data to file
  buffer.sync();
  file.truncate();
  file.rewind();
  file.close();
}
