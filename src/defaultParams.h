#ifndef TELEM_DEFAULT_PARAMS_H
#define TELEM_DEFAULT_PARAMS_H

#include <stdint.h>
#include <string.h>


// Given the size of Teensy 4.0 EEPROM, I think we're limited to 50 parameters
// if we store them with IDs and a checksum at the end and a header at the
// beginning.
constexpr uint8_t PARAM_HEADER[1] = {'P'};
constexpr std::size_t NUM_PARAMS = 24;
constexpr std::size_t PARAM_SIZE = sizeof(char[1]) + NUM_PARAMS*(sizeof(float) + sizeof(char[16])) + sizeof(uint16_t);

constexpr char PARAM_ID_0[16]  = "CT_KP_ROLL";
constexpr char PARAM_ID_1[16]  = "CT_KI_ROLL";
constexpr char PARAM_ID_2[16]  = "CT_KD_ROLL";
constexpr char PARAM_ID_3[16]  = "CT_KP_PITCH";
constexpr char PARAM_ID_4[16]  = "CT_KI_PITCH";
constexpr char PARAM_ID_5[16]  = "CT_KD_PITCH";
constexpr char PARAM_ID_6[16]  = "CT_KP_YAW";
constexpr char PARAM_ID_7[16]  = "CT_KI_YAW";
constexpr char PARAM_ID_8[16]  = "CT_KD_YAW";
constexpr char PARAM_ID_9[16]  = "CT_KP_XY";
constexpr char PARAM_ID_10[16] = "CT_KI_XY";
constexpr char PARAM_ID_11[16] = "CT_KD_XY";
constexpr char PARAM_ID_12[16] = "CT_KP_Z";
constexpr char PARAM_ID_13[16] = "CT_KI_Z";
constexpr char PARAM_ID_14[16] = "CT_KD_Z";
constexpr char PARAM_ID_15[16] = "EKF_A_NOISE";
constexpr char PARAM_ID_16[16] = "EKF_A_MKV";
constexpr char PARAM_ID_17[16] = "EKF_A_TAU";
constexpr char PARAM_ID_18[16] = "EKF_G_NOISE";
constexpr char PARAM_ID_19[16] = "EKF_G_MKV";
constexpr char PARAM_ID_20[16] = "EKF_G_TAU";

constexpr char PARAM_ID_21[16] = "CT2_KR";
constexpr char PARAM_ID_22[16] = "CT2_Kw";
constexpr char PARAM_ID_23[16] = "CT2_KI";


constexpr float PARAM_DEFAULT_0  = 1.66f;
constexpr float PARAM_DEFAULT_1  = 4.81f;
constexpr float PARAM_DEFAULT_2  = 0.34f;
constexpr float PARAM_DEFAULT_3  = 1.66f;
constexpr float PARAM_DEFAULT_4  = 4.81f;
constexpr float PARAM_DEFAULT_5  = 0.34;
constexpr float PARAM_DEFAULT_6  = 0.11f;
constexpr float PARAM_DEFAULT_7  = 0.1f;
constexpr float PARAM_DEFAULT_8  = 0.0f;
constexpr float PARAM_DEFAULT_9  = 5.0f;
constexpr float PARAM_DEFAULT_10 = 3.0f;
constexpr float PARAM_DEFAULT_11 = 8.0f;
constexpr float PARAM_DEFAULT_12 = 29.0f;
constexpr float PARAM_DEFAULT_13 = 8.0f;
constexpr float PARAM_DEFAULT_14 = 16.0f;
constexpr float PARAM_DEFAULT_15 = 0.096f;
constexpr float PARAM_DEFAULT_16 = 0.0003f;
constexpr float PARAM_DEFAULT_17 = 500.0f;
constexpr float PARAM_DEFAULT_18 = 0.0375f;
constexpr float PARAM_DEFAULT_19 = 0.00025f;
constexpr float PARAM_DEFAULT_20 = 250.0f;

constexpr float PARAM_DEFAULT_21 = 1.5f;
constexpr float PARAM_DEFAULT_22 = 0.11f;
constexpr float PARAM_DEFAULT_23 = 0.06f;

inline void GetDefaultTelemParams(uint8_t paramBuf[PARAM_SIZE]) {
  float paramDefaultVals[NUM_PARAMS];
  const char * paramDefaultIDs[NUM_PARAMS];
  // Set the header
  paramBuf[0] = PARAM_HEADER[0];
  // This is stupid
  paramDefaultVals[0]  = PARAM_DEFAULT_0;
  paramDefaultVals[1]  = PARAM_DEFAULT_1;
  paramDefaultVals[2]  = PARAM_DEFAULT_2;
  paramDefaultVals[3]  = PARAM_DEFAULT_3;
  paramDefaultVals[4]  = PARAM_DEFAULT_4;
  paramDefaultVals[5]  = PARAM_DEFAULT_5;
  paramDefaultVals[6]  = PARAM_DEFAULT_6;
  paramDefaultVals[7]  = PARAM_DEFAULT_7;
  paramDefaultVals[8]  = PARAM_DEFAULT_8;
  paramDefaultVals[9]  = PARAM_DEFAULT_9;
  paramDefaultVals[10] = PARAM_DEFAULT_10;
  paramDefaultVals[11] = PARAM_DEFAULT_11;
  paramDefaultVals[12] = PARAM_DEFAULT_12;
  paramDefaultVals[13] = PARAM_DEFAULT_13;
  paramDefaultVals[14] = PARAM_DEFAULT_14;
  paramDefaultVals[15] = PARAM_DEFAULT_15;
  paramDefaultVals[16] = PARAM_DEFAULT_16;
  paramDefaultVals[17] = PARAM_DEFAULT_17;
  paramDefaultVals[18] = PARAM_DEFAULT_18;
  paramDefaultVals[19] = PARAM_DEFAULT_19;
  paramDefaultVals[20] = PARAM_DEFAULT_20;
  paramDefaultVals[21] = PARAM_DEFAULT_21;
  paramDefaultVals[22] = PARAM_DEFAULT_22;
  paramDefaultVals[23] = PARAM_DEFAULT_23;


  paramDefaultIDs[0]  = PARAM_ID_0;
  paramDefaultIDs[1]  = PARAM_ID_1;
  paramDefaultIDs[2]  = PARAM_ID_2;
  paramDefaultIDs[3]  = PARAM_ID_3;
  paramDefaultIDs[4]  = PARAM_ID_4;
  paramDefaultIDs[5]  = PARAM_ID_5;
  paramDefaultIDs[6]  = PARAM_ID_6;
  paramDefaultIDs[7]  = PARAM_ID_7;
  paramDefaultIDs[8]  = PARAM_ID_8;  
  paramDefaultIDs[9]  = PARAM_ID_9;
  paramDefaultIDs[10] = PARAM_ID_10;
  paramDefaultIDs[11] = PARAM_ID_11;
  paramDefaultIDs[12] = PARAM_ID_12;
  paramDefaultIDs[13] = PARAM_ID_13;
  paramDefaultIDs[14] = PARAM_ID_14;
  paramDefaultIDs[15] = PARAM_ID_15;
  paramDefaultIDs[16] = PARAM_ID_16;
  paramDefaultIDs[17] = PARAM_ID_17;
  paramDefaultIDs[18] = PARAM_ID_18;
  paramDefaultIDs[19] = PARAM_ID_19;
  paramDefaultIDs[20] = PARAM_ID_20;
  paramDefaultIDs[21] = PARAM_ID_21;
  paramDefaultIDs[22] = PARAM_ID_22;
  paramDefaultIDs[23] = PARAM_ID_23;

  // Copy these to the buffer
  std::memcpy(&(paramBuf[1]), &(paramDefaultVals[0]), NUM_PARAMS*sizeof(float));

  // This is also stupid
  for (int i = 0; i < NUM_PARAMS; i++) {
    std::memcpy(&(paramBuf[1 + NUM_PARAMS*sizeof(float) + i*sizeof(char[16])]), paramDefaultIDs[i], sizeof(char[16]));
  }
}
#endif
