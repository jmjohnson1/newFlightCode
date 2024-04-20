#include "datalogger.h"


/**
 * @brief Prints the header line for the datalogger csv file
*/
void PrintCSVHeader() {
  buffer.print("roll");
  buffer.write(",");
  buffer.print("pitch");
  buffer.write(",");
  buffer.print("yaw");
  buffer.write(",");
  buffer.print("roll_des");
  buffer.write(",");
  buffer.print("pitch_des");
  buffer.write(",");
  buffer.print("yaw_des");
  buffer.write(",");
  buffer.print("throttle_des");
  buffer.write(",");
  buffer.print("roll_PID");
  buffer.write(",");
  buffer.print("pitch_PID");
  buffer.write(",");
  buffer.print("yaw_PID");
  buffer.write(",");

  // radio channels
  for (int i = 0; i < numChannels; i++) {
    buffer.print(radioChannels[i]->GetName());
    buffer.write(",");
  }

  buffer.print("GyroX");
  buffer.write(",");
  buffer.print("GyroY");
  buffer.write(",");
  buffer.print("GyroZ");
  buffer.write(",");
  buffer.print("AccX");
  buffer.write(",");
  buffer.print("AccY");
  buffer.write(",");
  buffer.print("AccZ");
  buffer.write(","); 
  buffer.print("m1_command");
  buffer.write(",");
  buffer.print("m2_command");
  buffer.write(",");
  buffer.print("m3_command");
  buffer.write(",");
  buffer.print("m4_command");
  buffer.write(",");
  buffer.print("kp_rp");
  buffer.write(",");
  buffer.print("ki_rp");
  buffer.write(",");
  buffer.print("kd_rp");
  buffer.write(",");
  buffer.print("kp_yaw");
  buffer.write(",");
  buffer.print("ki_yaw");
  buffer.write(",");
  buffer.print("kd_yaw");
  buffer.write(",");
  buffer.print("failsafeTriggered");
  buffer.write(",");
  buffer.print("timeSinceBoot");
  buffer.write(",");
  buffer.print("numMocapUpdates");
  buffer.write(",");
  buffer.print("MocapPositionX");
  buffer.write(",");
  buffer.print("MocapPositionY");
  buffer.write(",");
  buffer.print("MocapPositionZ");

#ifdef USE_EKF
  buffer.write(",");
  buffer.print("rollEstEKF");
  buffer.write(",");
  buffer.print("pitchEstEKF");
  buffer.write(",");
  buffer.print("yawEstEKF");
  buffer.write(",");
  buffer.print("xEstEKF");
  buffer.write(",");
  buffer.print("yEstEKF");
  buffer.write(",");
  buffer.print("zEstEKF");

  buffer.write(",");
  buffer.print("vxEstEKF");
  buffer.write(",");
  buffer.print("vyEstEKF");
  buffer.write(",");
  buffer.print("vzEstEKF");
  buffer.write(",");
  buffer.print("AccelBias1");
  buffer.write(",");
  buffer.print("AccelBias2");
  buffer.write(",");
  buffer.print("AccelBias3");
  buffer.write(",");
  buffer.print("GyroBias1");
  buffer.write(",");
  buffer.print("GyroBias2");
  buffer.write(",");
  buffer.print("GyroBias3");
  

#endif

#ifdef USE_POSITION_CONTROLLER
  buffer.write(",");
  buffer.print("setpointX");
  buffer.write(",");
  buffer.print("setpointY");
  buffer.write(",");
  buffer.print("setpointZ");

  buffer.write(",");
  buffer.print("kp_xy");
  buffer.write(",");
  buffer.print("ki_xy");
  buffer.write(",");
  buffer.print("kd_xy");
  buffer.write(",");
  buffer.print("kp_z");
  buffer.write(",");
  buffer.print("ki_z");
  buffer.write(",");
  buffer.print("kd_z");
#endif

  buffer.write(",");
  buffer.print("mocapTimestamp");
  buffer.write(",");
  buffer.print("quadTimestamp");

  buffer.println();
}

/**
 * @brief Initializes SD card logging
 * @returns 1 if there was an error communicating with the SD card or creating
 * the file. 0 if successful
*/
int Setup() {
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
  datalogger::PrintCSVHeader();
  return 0;
}

/**
 * @brief Writes the data to the FIFO buffer used for the sd card. Once the
 * amount of data in the FIFO buffer has reached the SD sector size (512
 * bytes), the data is written to the SD card.
 * @returns 1 if the log file is full or the data buffer failed to write to
 * the SD card. 0 if successful.
*/
int WriteBuffer() {
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

  buffer.print(quadData.att.eulerAngles_madgwick[0], 4);
  buffer.write(",");
  buffer.print(quadData.att.eulerAngles_madgwick[0], 4);
  buffer.write(",");
  buffer.print(quadData.att.eulerAngles_madgwick[0], 4);
  buffer.write(",");
  buffer.print(roll_des, 4);
  buffer.write(",");
  buffer.print(pitch_des, 4);
  buffer.write(",");
  buffer.print(yaw_des, 4);
  buffer.write(",");
  buffer.print(thrust_des, 4);
  buffer.write(",");
  buffer.print(controller.GetRollPID(), 4);
  buffer.write(",");
  buffer.print(controller.GetPitchPID(), 4);
  buffer.write(",");
  buffer.print(controller.GetYawPID(), 4);
  buffer.write(",");

  // radio channels
  for (int i = 0; i < numChannels; i++) {
    buffer.print(radioChannels[i]->GetRawValue());
    buffer.write(",");
  }

  buffer.print(quadIMU.GetGyroX(), 4);
  buffer.write(",");
  buffer.print(quadIMU.GetGyroY(), 4);
  buffer.write(",");
  buffer.print(quadIMU.GetGyroZ(), 4);
  buffer.write(",");
  buffer.print(quadIMU.GetAccX(), 4);
  buffer.write(",");
  buffer.print(quadIMU.GetAccY(), 4);
  buffer.write(",");
  buffer.print(quadIMU.GetAccZ(), 4);
  buffer.write(",");
  float motorCommands[4] = {0, 0, 0, 0};
  motors.GetMotorCommands(motorCommands);
  buffer.print(motorCommands[0], 4);
  buffer.write(",");
  buffer.print(motorCommands[1], 4);
  buffer.write(",");
  buffer.print(motorCommands[2], 4);
  buffer.write(",");
  buffer.print(motorCommands[3], 4);
  buffer.write(",");
  buffer.print(Kp_roll_angle*pScale_att, 4);
  buffer.write(",");
  buffer.print(Ki_roll_angle*iScale_att, 4);
  buffer.write(",");
  buffer.print(Kd_roll_angle*dScale_att, 4);
  buffer.write(",");
  buffer.print(Kp_yaw, 4);
  buffer.write(",");
  buffer.print(Ki_yaw, 4);
  buffer.write(",");
  buffer.print(Kd_yaw, 4);
  buffer.write(",");
  buffer.print(failureFlag);
  buffer.write(",");
  buffer.print(micros());
  buffer.write(",");
  buffer.print(numMocapUpdates);
  buffer.write(",");
  buffer.print(mocapPosition[0], 10);
  buffer.write(",");
  buffer.print(mocapPosition[1], 10);
  buffer.write(",");
  buffer.print(mocapPosition[2], 10);
#ifdef USE_EKF
  buffer.write(",");
  buffer.print(ins.Get_OrientEst()[0], 5);
  buffer.write(",");
  buffer.print(ins.Get_OrientEst()[1], 5);
  buffer.write(",");
  buffer.print(ins.Get_OrientEst()[2], 5);
  buffer.write(",");
  buffer.print(ins.Get_PosEst()[0], 10);
  buffer.write(",");
  buffer.print(ins.Get_PosEst()[1], 10);
  buffer.write(",");
  buffer.print(ins.Get_PosEst()[2], 10);
  buffer.write(",");
  buffer.print(ins.Get_VelEst()[0], 4);
  buffer.write(",");
  buffer.print(ins.Get_VelEst()[1], 4);
  buffer.write(",");
  buffer.print(ins.Get_VelEst()[2], 4);
  buffer.write(",");
  buffer.print(ins.Get_AccelBias()[0], 4);
  buffer.write(",");
  buffer.print(ins.Get_AccelBias()[1], 4);
  buffer.write(",");
  buffer.print(ins.Get_AccelBias()[2], 4);
  buffer.write(",");
  buffer.print(ins.Get_RotRateBias()[0], 4);
  buffer.write(",");
  buffer.print(ins.Get_RotRateBias()[1], 4);
  buffer.write(",");
  buffer.print(ins.Get_RotRateBias()[2], 4);
#endif
#ifdef USE_POSITION_CONTROLLER
  buffer.write(",");
  buffer.print(quadData.navData.positionSetpoint_NED[0]);
  buffer.write(",");
  buffer.print(quadData.navData.positionSetpoint_NED[1]);
  buffer.write(",");
  buffer.print(quadData.navData.positionSetpoint_NED[2]);
  buffer.write(",");
  buffer.print(posControl.GetKp()[0]);
  buffer.write(",");
  buffer.print(posControl.GetKi()[0]);
  buffer.write(",");
  buffer.print(posControl.GetKd()[0]);
  buffer.write(",");
  buffer.print(posControl.GetKp()[2]);
  buffer.write(",");
  buffer.print(posControl.GetKi()[2]);
  buffer.write(",");
  buffer.print(posControl.GetKd()[2]);
#endif

  buffer.write(",");
  buffer.print(mocapTimestamp);
  buffer.write(",");
  buffer.print(quadTimestamp);

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

void EndProcess() {
  // Write any remaining buffer data to file
  buffer.sync();
  file.truncate();
  file.rewind();
  file.close();
  
  Serial.println("logging ended peacefully");
}