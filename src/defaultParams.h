#ifndef TELEM_DEFAULT_PARAMS_H
#define TELEM_DEFAULT_PARAMS_H

#include <stdint.h>
#include <string.h>


// Given the size of Teensy 4.0 EEPROM, I think we're limited to 50 parameters
// if we store them with IDs and a checksum at the end and a header at the
// beginning.
constexpr uint8_t PARAM_HEADER[1] = {'P'};
constexpr std::size_t NUM_PARAMS = 18;
constexpr std::size_t PARAM_SIZE = sizeof(char[1]) + NUM_PARAMS*(sizeof(float) + sizeof(char[16])) + sizeof(uint16_t);

constexpr char PARAM_ID_KP_ROLL[16]  = "KP_ROLL";
constexpr char PARAM_ID_KI_ROLL[16]  = "KI_ROLL";
constexpr char PARAM_ID_KD_ROLL[16]  = "KD_ROLL";
constexpr char PARAM_ID_KP_PITCH[16] = "KP_PITCH";
constexpr char PARAM_ID_KI_PITCH[16] = "KI_PITCH";
constexpr char PARAM_ID_KD_PITCH[16] = "KD_PITCH";
constexpr char PARAM_ID_KP_YAW[16]   = "KP_YAW";
constexpr char PARAM_ID_KI_YAW[16]   = "KI_YAW";
constexpr char PARAM_ID_KD_YAW[16]   = "KD_YAW";
constexpr char PARAM_ID_KP_X[16]     = "KP_X";
constexpr char PARAM_ID_KI_X[16]     = "KI_X";
constexpr char PARAM_ID_KD_X[16]     = "KD_X";
constexpr char PARAM_ID_KP_Y[16]     = "KP_Y";
constexpr char PARAM_ID_KI_Y[16]     = "KI_Y";
constexpr char PARAM_ID_KD_Y[16]     = "KD_Y";
constexpr char PARAM_ID_KP_Z[16]     = "KP_Z";
constexpr char PARAM_ID_KI_Z[16]     = "KI_Z";
constexpr char PARAM_ID_KD_Z[16]     = "KD_Z";

constexpr float PARAM_DEFAULT_KP_ROLL  = 0.0105f;
constexpr float PARAM_DEFAULT_KI_ROLL  = 0.00247f;
constexpr float PARAM_DEFAULT_KD_ROLL  = 0.001958f;
constexpr float PARAM_DEFAULT_KP_PITCH = 0.0105f;
constexpr float PARAM_DEFAULT_KI_PITCH = 0.00247f;
constexpr float PARAM_DEFAULT_KD_PITCH = 0.001958f;
constexpr float PARAM_DEFAULT_KP_YAW   = 0.002f;
constexpr float PARAM_DEFAULT_KI_YAW   = 0.0f;
constexpr float PARAM_DEFAULT_KD_YAW   = 0.0f;
constexpr float PARAM_DEFAULT_KP_X     = 5.0f;
constexpr float PARAM_DEFAULT_KI_X     = 3.0f;
constexpr float PARAM_DEFAULT_KD_X     = 8.0f;
constexpr float PARAM_DEFAULT_KP_Y     = 5.0f;
constexpr float PARAM_DEFAULT_KI_Y     = 3.0f;
constexpr float PARAM_DEFAULT_KD_Y     = 8.0f;
constexpr float PARAM_DEFAULT_KP_Z     = 29.0f;
constexpr float PARAM_DEFAULT_KI_Z     = 8.0f;
constexpr float PARAM_DEFAULT_KD_Z     = 16.0f;

inline void GetDefaultTelemParams(uint8_t paramBuf[PARAM_SIZE]) {
  float paramDefaultVals[NUM_PARAMS];
  const char * paramDefaultIDs[NUM_PARAMS];
  // Set the header
  paramBuf[0] = PARAM_HEADER[0];
  // This is stupid
  paramDefaultVals[0]  = PARAM_DEFAULT_KP_ROLL ;
  paramDefaultVals[1]  = PARAM_DEFAULT_KI_ROLL ;
  paramDefaultVals[2]  = PARAM_DEFAULT_KD_ROLL ;
  paramDefaultVals[3]  = PARAM_DEFAULT_KP_PITCH;
  paramDefaultVals[4]  = PARAM_DEFAULT_KI_PITCH;
  paramDefaultVals[5]  = PARAM_DEFAULT_KD_PITCH;
  paramDefaultVals[6]  = PARAM_DEFAULT_KP_YAW  ;
  paramDefaultVals[7]  = PARAM_DEFAULT_KI_YAW  ;
  paramDefaultVals[8]  = PARAM_DEFAULT_KD_YAW  ;
  paramDefaultVals[9]  = PARAM_DEFAULT_KP_X    ;
  paramDefaultVals[10] = PARAM_DEFAULT_KI_X    ;
  paramDefaultVals[11] = PARAM_DEFAULT_KD_X    ;
  paramDefaultVals[12] = PARAM_DEFAULT_KP_Y    ;
  paramDefaultVals[13] = PARAM_DEFAULT_KI_Y    ;
  paramDefaultVals[14] = PARAM_DEFAULT_KD_Y    ;
  paramDefaultVals[15] = PARAM_DEFAULT_KP_Z    ;
  paramDefaultVals[16] = PARAM_DEFAULT_KI_Z    ;
  paramDefaultVals[17] = PARAM_DEFAULT_KD_Z    ;

  paramDefaultIDs[0]  = PARAM_ID_KP_ROLL ;
  paramDefaultIDs[1]  = PARAM_ID_KI_ROLL ;
  paramDefaultIDs[2]  = PARAM_ID_KD_ROLL ;
  paramDefaultIDs[3]  = PARAM_ID_KP_PITCH;
  paramDefaultIDs[4]  = PARAM_ID_KI_PITCH;
  paramDefaultIDs[5]  = PARAM_ID_KD_PITCH;
  paramDefaultIDs[6]  = PARAM_ID_KP_YAW  ;
  paramDefaultIDs[7]  = PARAM_ID_KI_YAW  ;
  paramDefaultIDs[8]  = PARAM_ID_KD_YAW  ;  
  paramDefaultIDs[9]  = PARAM_ID_KP_X    ;
  paramDefaultIDs[10] = PARAM_ID_KI_X    ;
  paramDefaultIDs[11] = PARAM_ID_KD_X    ;
  paramDefaultIDs[12] = PARAM_ID_KP_Y    ;
  paramDefaultIDs[13] = PARAM_ID_KI_Y    ;
  paramDefaultIDs[14] = PARAM_ID_KD_Y    ;
  paramDefaultIDs[15] = PARAM_ID_KP_Z    ;
  paramDefaultIDs[16] = PARAM_ID_KI_Z    ;
  paramDefaultIDs[17] = PARAM_ID_KD_Z    ;

  // Copy these to the buffer
  std::memcpy(&(paramBuf[1]), &(paramDefaultVals[0]), NUM_PARAMS*sizeof(float));

  // This is also stupid
  for (int i = 0; i < NUM_PARAMS; i++) {
    std::memcpy(&(paramBuf[1 + NUM_PARAMS*sizeof(float) + i*sizeof(char[16])]), paramDefaultIDs[i], sizeof(char[16]));
  }
}
#endif