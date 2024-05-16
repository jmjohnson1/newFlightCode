/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan

Adapted from prior versions
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang

Reference Frames and Coordinates from nav-functions()
I - ECI (Earch Center Inertial): origin at Earth center
E - ECEF (Earch Center Earth Fixed): origin at Earth center
D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
G - Geocentric: origin at Earth center, Uses spheroid definition
L - Local Level: origin at specified reference, [x- North, y- East, z- Down]
B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]

All units meters and radians
"Acceleration" is actually "specific gravity", ie. gravity is removed.
*/

#pragma once

#include <stdint.h>
#include <math.h>
#include "eigen.h"
using namespace Eigen;

#include "nav-functions.h"

class uNavINS {
  public:
    uNavINS() {};
    void Configure();
    void Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m);
    bool Initialized() { return initialized_; } // returns whether the INS has been initialized
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m);
    void Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps);

    // Set Configuration
    inline void Set_AccelSigma(Vector3f val) { aNoiseSigma_mps2 = val; }
    inline void Set_AccelMarkov(Vector3f val) { aMarkovSigma_mps2 = val; }
    inline void Set_AccelTau(Vector3f val) { aMarkovTau_s = val; }
    inline void Set_RotRateSigma(Vector3f val) { wNoiseSigma_rps = val; }
    inline void Set_RotRateMarkov(Vector3f val) { wMarkovSigma_rps = val; }
    inline void Set_RotRateTau(Vector3f val) { wMarkovTau_s = val; }
    inline void Set_PosSigmaNE(float val) { pNoiseSigma_NE_m = val; }
    inline void Set_PosSigmaD(float val) { pNoiseSigma_D_m = val; }

    // Set Initial Covariance
    inline void Set_InitPosSigma(float val) { pErrSigma_Init_m = val; }
    inline void Set_InitVelSigma(float val) { vErrSigma_Init_mps = val; }
    inline void Set_InitOrientSigma(float val) { attErrSigma_Init_rad = val; }
    inline void Set_InitHeadingSigma(float val) { hdgErrSigma_Init_rad = val; }
    inline void Set_InitAccelBiasSigma(float val) { aBiasSigma_Init_mps2 = val; }
    inline void Set_InitRotRateBiasSigma(float val) { wBiasSigma_Init_rps = val; }

    // Get Navigation Estimates
    inline Vector3f Get_AccelEst() { return aEst_B_mps2_; }
    inline Vector3f Get_AccelBias() { return aBias_mps2_; }
    inline Vector3f Get_RotRateEst() { return wEst_B_rps_; }
    inline Vector3f Get_RotRateBias() { return wBias_rps_; }
    inline Vector3f Get_OrientEst() { return euler_BL_rad_; }
    inline Vector3d Get_PosEst() { return pEst_NED_m_; }
    inline Vector3f Get_VelEst() { return vEst_NED_mps_; }
    inline float Get_Track() { return atan2f(vEst_NED_mps_(1), vEst_NED_mps_(0)); }

    // Get Covariance Estimates
    inline Vector3f Get_CovPos() { return P_.block(0,0,3,3).diagonal(); }
    inline Vector3f Get_CovVel() { return P_.block(3,3,3,3).diagonal(); }
    inline Vector3f Get_CovOrient() { return P_.block(6,6,3,3).diagonal(); }
    inline Vector3f Get_CovAccelBias() { return P_.block(9,9,3,3).diagonal(); }
    inline Vector3f Get_CovRotRateBias() { return P_.block(12,12,3,3).diagonal(); }

    // Get Innovation
    inline Vector3f Get_InnovationPos() { return S_.block(0,0,3,3).diagonal(); }
    inline Vector3f Get_InnovationVel() { return S_.block(3,3,3,3).diagonal(); }

    // Get State
    inline Vector<float, 15> Get_State() { return state_; }

  private:
    // Model Constants
    const float G = 9.807f; // Acceleration due to gravity
    const double EARTH_RADIUS = 6378137.0; // earth semi-major axis radius (m)

    // Initialize flag
    bool initialized_ = false;

    // Timing
    uint64_t tPrev_us_;
    float dt_s_;
    unsigned long timeWeekPrev_;

    // Sensor variances (as standard deviation) and models (tau)
    Vector3f aNoiseSigma_mps2 = {0.096f, 0.096f, 0.096f}; // Std dev of accelerometer wide band noise (m/s^2)
    Vector3f aMarkovSigma_mps2 = {0.0003, 0.0003, 0.0003}; // Std dev of accelerometer Markov bias
    Vector3f aMarkovTau_s = {500, 500, 500}; // Correlation time or time constant

    Vector3f wNoiseSigma_rps {0.0375f, 0.0375f, 0.0375f}; // Std dev of rotation rate output noise (rad/s)
    Vector3f wMarkovSigma_rps = {0.00025f, 0.00025f, 0.00025f}; // Std dev of correlated rotation rate bias
    Vector3f wMarkovTau_s = {250, 250, 250}; // Correlation time or time constant

    float pNoiseSigma_NE_m = 0.02375; // GPS measurement noise std dev (m)
    float pNoiseSigma_D_m = 0.02375; // GPS measurement noise std dev (m)
	
    float vNoiseSigma_NE_mps = 1.0f; // GPS measurement noise std dev (m/s)  PLACEHOLDER!
    float vNoiseSigma_D_mps = 1.0f; // GPS measurement noise std dev (m/s)

    // Initial set of covariance
    float pErrSigma_Init_m = 1.0f; // Std dev of initial position error (m)
    float vErrSigma_Init_mps = 1.0f; // Std dev of initial velocity error (m/s)
    float attErrSigma_Init_rad = 0.34906f/3; // Std dev of initial attitude (phi and theta) error (rad)
    float hdgErrSigma_Init_rad = 3.14159f; // Std dev of initial Heading (psi) error (rad)
    float aBiasSigma_Init_mps2 = 0.981f/10; // Std dev of initial acceleration bias (m/s^2)
    float wBiasSigma_Init_rps = 0.01745f; // Std dev of initial rotation rate bias (rad/s)



    // Identity matrices
    const Matrix<float,2,2> I2 = Matrix<float,2,2>::Identity();
    const Matrix<float,3,3> I3 = Matrix<float,3,3>::Identity();
    const Matrix<float,5,5> I5 = Matrix<float,5,5>::Identity();
    const Matrix<float,15,15> I15 = Matrix<float,15,15>::Identity();

    // Kalman Matrices
    Matrix<float,3,15> H_; // Observation matrix
    Matrix<float,3,3> R_;// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<float,12,12> Rw_; // Covariance of the Sensor Noise (associated with TimeUpdate())
    Matrix<float,3,3> S_; // Innovation covariance
    Matrix<float,15,15> P_; // Covariance estimate

    Matrix<float,6,15> H2_; // Observation matrix for meas update with velocity
    Matrix<float,6,6> R2_;// Covariance of the Observation Noise (associated with MeasUpdate())
    Matrix<float,6,6> S2_; // Innovation covariance
	
    // Global variables
    Vector3f aBias_mps2_; // acceleration bias
    Vector3f wBias_rps_; // rotation rate bias
    Vector3f euler_BL_rad_; // Euler angles - B wrt L (3-2-1) [phi, theta, psi]
    Quaternionf quat_BL_; // Quaternion of B wrt L
    Vector3f aEst_B_mps2_; // Estimated acceleration in Body
    Vector3f wEst_B_rps_; // Estimated rotation rate in Body
    Vector3f vEst_NED_mps_; // Estimated velocity in NED
    Vector3d pEst_NED_m_; // Estimated position in NED

    // State Vector
    Vector<float, 15> state_;

    // Methods
    void TimeUpdate();
    void MeasUpdate(Vector3d pMeas_D_rrm);
		void MeasUpdate(Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps);
    void UpdateStateVector();
};
