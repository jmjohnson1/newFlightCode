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

#include "EKF.h"


void EKF::Configure() {
  // Observation matrix (H)
  H_.setZero();
  H_.block(0,0,3,3) = I3;

  // Covariance of the Process Noise (associated with TimeUpdate())
  Rw_.setZero();
  Rw_.block(0,0,3,3) = (aNoiseSigma_mps2.cwiseProduct(aNoiseSigma_mps2)).asDiagonal();
  Rw_.block(3,3,3,3) = (wNoiseSigma_rps.cwiseProduct(wNoiseSigma_rps)).asDiagonal();
  Rw_.block(6,6,3,3) = 2.0f * (aMarkovSigma_mps2.cwiseProduct(aMarkovSigma_mps2)).cwiseQuotient(aMarkovTau_s).asDiagonal();
  Rw_.block(9,9,3,3) = 2.0f * (wMarkovSigma_rps.cwiseProduct(wMarkovSigma_rps)).cwiseQuotient(wMarkovTau_s).asDiagonal();

  // Covariance of the Observation Noise (associated with MeasUpdate())
  R_.setZero();
  R_.block(0,0,2,2) = (pNoiseSigma_NE_m * pNoiseSigma_NE_m) * I2;
  R_(2,2) = (pNoiseSigma_D_m * pNoiseSigma_D_m);

  // Initial Innovation Covariance Estimate (S)
  S_.setZero();

  // Initial Covariance Estimate (P)
  P_.setZero();
  P_.block(0,0,3,3) = (pErrSigma_Init_m * pErrSigma_Init_m) * I3;
  P_.block(3,3,3,3) = (vErrSigma_Init_mps * vErrSigma_Init_mps) * I3;
  P_.block(6,6,2,2) = (attErrSigma_Init_rad * attErrSigma_Init_rad) * I2;
  P_(8,8) = (hdgErrSigma_Init_rad * hdgErrSigma_Init_rad);
  P_.block(9,9,3,3) = (aBiasSigma_Init_mps2 * aBiasSigma_Init_mps2) * I3;
  P_.block(12,12,3,3) = (wBiasSigma_Init_rps * wBiasSigma_Init_rps) * I3;



  H2_.setZero();
  H2_.block(0,0,5,5) = I5;
  R2_.setZero();
  R2_.block(0,0,2,2) = (pNoiseSigma_NE_m * pNoiseSigma_NE_m) * I2;
  R2_(2,2) = (pNoiseSigma_D_m * pNoiseSigma_D_m);
  R2_.block(3,3,2,2) = (vNoiseSigma_NE_mps * vNoiseSigma_NE_mps) * I2;
  R2_(5,5) = (vNoiseSigma_D_mps * vNoiseSigma_D_mps);
  S2_.setZero();
}

void EKF::Initialize(Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m) {
  // Initialize Position and Velocity
  pEst_NED_m_ = pMeas_NED_m; // Position in NED
  vEst_NED_mps_.setZero(); // Velocity in NED

  // Initialize sensor biases
  //wBias_rps_ = wMeas_B_rps;
  wBias_rps_.setZero();
  aBias_mps2_.setZero();

  // New Specific forces and Rotation Rate
  aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
  wEst_B_rps_ = wMeas_B_rps - wBias_rps_;

  // Initial attitude, roll and pitch
  Vector3f aEst_B_nd = aEst_B_mps2_ / aEst_B_mps2_.norm(); // Normalize to remove the 1g sensitivity
  euler_BL_rad_(1) = asinf(aEst_B_nd(0));
  euler_BL_rad_(0) = -asinf(aEst_B_nd(1) / cosf(euler_BL_rad_(1)));

  // Estimate initial heading
  euler_BL_rad_(2) = M_PI;

  // Euler to quaternion
  quat_BL_ = Euler2Quat(euler_BL_rad_);

  // State vector
  UpdateStateVector();

  // set initialized flag
  initialized_ = true;
}

void EKF::UpdateStateVector() {
  state_(seq(0,2)) = pEst_NED_m_.cast <float> ();
  state_(seq(3, 5)) = vEst_NED_mps_;
  state_(seq(6, 8)) = euler_BL_rad_;
  state_(seq(9, 11)) = aBias_mps2_;
  state_(seq(12,14)) = wBias_rps_;
}


void EKF::Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m) {
  // change in time
  dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6;
  tPrev_us_ = t_us;

  // Catch large dt
  if (dt_s_ > 0.1) {dt_s_ = 0.1;}

  // A-priori accel and rotation rate estimate
  aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
  wEst_B_rps_ = wMeas_B_rps - wBias_rps_;

  // Kalman Time Update (Prediction)
  TimeUpdate();

  // Gps measurement update, if TOW increased
  if ((timeWeek - timeWeekPrev_) > 0) {
    timeWeekPrev_ = timeWeek;

    // Kalman Measurement Update
    MeasUpdate(pMeas_NED_m);

    // Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
    aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
    wEst_B_rps_ = wMeas_B_rps - wBias_rps_;
  }

  // Euler angles from quaternion
  euler_BL_rad_ = Quat2Euler(quat_BL_);

  UpdateStateVector();
}

void EKF::Update(uint64_t t_us, unsigned long timeWeek, Vector3f wMeas_B_rps, Vector3f aMeas_B_mps2, Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps) {
  // change in time
  dt_s_ = ((float)(t_us - tPrev_us_)) / 1e6;
  tPrev_us_ = t_us;

  // Catch large dt
  if (dt_s_ > 0.1) {dt_s_ = 0.1;}

  // A-priori accel and rotation rate estimate
  aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
  wEst_B_rps_ = wMeas_B_rps - wBias_rps_;

  // Kalman Time Update (Prediction)
  TimeUpdate();

  // Gps measurement update, if TOW increased
  if ((timeWeek - timeWeekPrev_) > 0) {
    timeWeekPrev_ = timeWeek;

    // Kalman Measurement Update
    MeasUpdate(pMeas_NED_m, vMeas_NED_mps);

    // Post-priori accel and rotation rate estimate, biases updated in MeasUpdate()
    aEst_B_mps2_ = aMeas_B_mps2 - aBias_mps2_;
    wEst_B_rps_ = wMeas_B_rps - wBias_rps_;
  }

  // Euler angles from quaternion
  euler_BL_rad_ = Quat2Euler(quat_BL_);

  UpdateStateVector();
}

void EKF::TimeUpdate() {
  // Attitude Update
  Quaternionf dQuat_BL = Quaternionf(1.0, 0.5f*wEst_B_rps_(0)*dt_s_, 0.5f*wEst_B_rps_(1)*dt_s_, 0.5f*wEst_B_rps_(2)*dt_s_);
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();

  // Avoid quaternion flips sign
  if (quat_BL_.w() < 0) {
    quat_BL_ = Quaternionf(-quat_BL_.w(), -quat_BL_.x(), -quat_BL_.y(), -quat_BL_.z());
  }

  // Compute DCM (Body to/from NED) Transformations from Quaternion
  Matrix3f T_NED2B = Quat2DCM(quat_BL_);
  Matrix3f T_B2NED = T_NED2B.transpose();

  // Velocity Update
  Vector3f aGrav_mps2 = Vector3f(0.0, 0.0, G);
	pEst_NED_m_ += (dt_s_*vEst_NED_mps_ + 0.5f*dt_s_*dt_s_*(T_B2NED*aEst_B_mps2_ + aGrav_mps2)).cast <double> ();
  vEst_NED_mps_ += dt_s_ * (T_B2NED * aEst_B_mps2_ + aGrav_mps2);

  // Position Update
  //Vector3f pDot_D = NED2D_Rate(vEst_NED_mps_, pEst_D_rrm_);
  //pEst_D_rrm_ += (dt_s_ * pDot_D).cast <double> ();

  // Assemble the Jacobian (state update matrix)
  Matrix<float,15,15> Fs; Fs.setZero();
  Fs.block(0,3,3,3) = I3;
  Fs(5,2) = -2.0f * G / EARTH_RADIUS;
  Fs.block(3,6,3,3) = -2.0f * T_B2NED * Skew(aEst_B_mps2_);
  Fs.block(3,9,3,3) = -T_B2NED;
  Fs.block(6,6,3,3) = -Skew(wEst_B_rps_);
  Fs.block(6,12,3,3) = -0.5f * I3;
  Fs.block(9,9,3,3) = -1*aMarkovTau_s.cwiseInverse().asDiagonal(); // ... Accel Markov Bias
  Fs.block(12,12,3,3) = -1*wMarkovTau_s.cwiseInverse().asDiagonal(); // ... Rotation Rate Markov Bias

  // State Transition Matrix
  Matrix<float,15,15> PHI = I15 + Fs * dt_s_;

  // Process Noise Covariance (Discrete approximation)
  Matrix<float,15,12> Gs; Gs.setZero();
  Gs.block(3,0,3,3) = -T_B2NED;
  Gs.block(6,3,3,3) = -0.5f * I3;
  Gs.block(9,6,3,3) = I3;
  Gs.block(12,9,3,3) = I3;

  // Discrete Process Noise
  Matrix<float,15,15> Q; Q.setZero();
  Q = PHI * dt_s_ * Gs * Rw_ * Gs.transpose();
  Q = 0.5f * (Q + Q.transpose());

  // Covariance Time Update
  P_ = PHI * P_ * PHI.transpose() + Q;
  P_ = 0.5f * (P_ + P_.transpose());
}

// Measurement Update
void EKF::MeasUpdate(Vector3d pMeas_NED_m) {
  // Position Error, converted to NED
  //Matrix3f T_E2NED = TransE2NED(pEst_NED_m_).cast <float> (); // Compute ECEF to NED with double precision, cast to float
  //Vector3f pErr_NED_m = T_E2NED * (D2E(pMeas_D_rrm) - D2E(pEst_D_rrm_)).cast <float> ();// Compute position error double precision, cast to float, apply transformation
  Vector3f pErr_NED_m = (pMeas_NED_m - pEst_NED_m_).cast<float>();

  // Create measurement Y, as Error between Measures and Outputs
  Matrix<float,3,1> y; y.setZero();
  y.segment(0,3) = pErr_NED_m;

  // Innovation covariance
  S_ = H_ * P_ * H_.transpose() + R_;

  // Kalman gain
  Matrix<float,15,3> K; K.setZero();
  K = P_ * H_.transpose() * S_.inverse();

  // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
  Matrix<float,15,15> I_KH = I15 - K * H_; // temp
  P_ = I_KH * P_ * I_KH.transpose() + K * R_ * K.transpose();

  // State update, x = K * y
  Matrix<float,15,1> x = K * y;

  // Pull apart x terms to update the Position, velocity, orientation, and sensor biases
  Vector3f pDeltaEst_L = x.segment(0,3); // Position Deltas in NED
  Vector3f vDeltaEst_L = x.segment(3,3); // Velocity Deltas in NED
  Vector3f quatDelta = x.segment(6,3); // Quaternion Delta
  Vector3f aBiasDelta = x.segment(9,3); // Accel Bias Deltas
  Vector3f wBiasDelta = x.segment(12,3); // Rotation Rate Bias Deltas

  // Position update
  pEst_NED_m_ += pDeltaEst_L.cast<double>();

  // Velocity update
  vEst_NED_mps_ += vDeltaEst_L;

  // Attitude correction
  Quaternionf dQuat_BL = Quaternionf(1.0, quatDelta(0), quatDelta(1), quatDelta(2));
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();

  // Update biases from states
  aBias_mps2_ += aBiasDelta;
  wBias_rps_ += wBiasDelta;
}

void EKF::MeasUpdate(Vector3d pMeas_NED_m, Vector3f vMeas_NED_mps) {
  Vector3f pErr_NED_m = (pMeas_NED_m - pEst_NED_m_).cast<float>();
	Vector3f vErr_NED_mps = vMeas_NED_mps - vEst_NED_mps_;

  // Create measurement Y, as Error between Measures and Outputs
  Matrix<float,6,1> y; y.setZero();
  y.segment(0,3) = pErr_NED_m;
	y.segment(3,3) = vErr_NED_mps;

  // Innovation covariance
  S2_ = H2_ * P_ * H2_.transpose() + R2_;

  // Kalman gain
  Matrix<float,15,6> K; K.setZero();
  K = P_ * H2_.transpose() * S2_.inverse();

  // Covariance update, P = (I + K * H) * P * (I + K * H)' + K * R * K'
  Matrix<float,15,15> I_KH = I15 - K * H2_; // temp
  P_ = I_KH * P_ * I_KH.transpose() + K * R2_ * K.transpose();

  // State update, x = K * y
  Matrix<float,15,1> x = K * y;

  // Pull apart x terms to update the Position, velocity, orientation, and sensor biases
  Vector3f pDeltaEst_D = x.segment(0,3); // Position Deltas in NED
  Vector3f vDeltaEst_L = x.segment(3,3); // Velocity Deltas in NED
  Vector3f quatDelta = x.segment(6,3); // Quaternion Delta
  Vector3f aBiasDelta = x.segment(9,3); // Accel Bias Deltas
  Vector3f wBiasDelta = x.segment(12,3); // Rotation Rate Bias Deltas

  // Position update
  pEst_NED_m_ += x.segment(0,3).cast <double> ();

  // Velocity update
  vEst_NED_mps_ += vDeltaEst_L;

  // Attitude correction
  Quaternionf dQuat_BL = Quaternionf(1.0, quatDelta(0), quatDelta(1), quatDelta(2));
  quat_BL_ = (quat_BL_ * dQuat_BL).normalized();

  // Update biases from states
  aBias_mps2_ += aBiasDelta;
  wBias_rps_ += wBiasDelta;
}