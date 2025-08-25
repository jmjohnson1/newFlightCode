#include "quadcopter.h"
#include "dynamicsMath.h"
#include "teensy_hal.h"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>


QuadcopterModel::QuadcopterModel(QuadState state, QuadConfig config) {
  state_ = state;
  config_ = config;
}

int QuadcopterModel::GetStateDerivative(double t, const double y[], double dydt[], void *params) {
  /* \brief Computes the quadcopter's state derivative
   * \param t Current simulation time (unused)
   * \param y Current state
   * \param dydt State derivative (output)
   * \param params Pointer to parameters (unused)
   */

  ParamsWrapper *wrapper = static_cast<ParamsWrapper *>(params);
  QuadConfig config = wrapper->config;
  Vector4d motor_setpoint = wrapper->motor_setpoint;

  // This seems really stupid
  Vector3d r_bn_n = {y[0], y[1], y[2]};
  Vector3d v_bn_n = {y[3], y[4], y[5]};
  Matrix3d C_bn; // Populated below
  Vector3d w_bn_b = {y[15], y[16], y[17]};
  Vector4d w_mb_b = {y[18], y[19], y[20], y[21]};

  Vector3d r_bn_n_dot, v_bn_n_dot, w_bn_b_dot;
  Matrix3d C_bn_dot;
  Vector4d w_mb_b_dot;

  C_bn << y[6], y[7], y[8],
          y[9], y[10], y[11],
          y[12], y[13], y[14];


  // Motor dynamics
  w_mb_b_dot = (motor_setpoint - w_mb_b) / config.tau_motor;
  for (int i = 0; i < 4; ++i) {
    if (w_mb_b[i] <= config.min_motor_rate && w_mb_b_dot[i] < 0.0) {
      w_mb_b_dot[i] = 0.0;
    } else if (w_mb_b[i] >= config.max_motor_rate && w_mb_b_dot[i] > 0.0) {
      w_mb_b_dot[i] = 0.0;
    }
  }

  Vector3d motor_gyroscopic = {0, 0,
                               (w_mb_b(0) - w_mb_b(1) + w_mb_b(2) - w_mb_b(3))};
  // Vector arranged as {thrust, tau_b1, tau_b2, tau_b3}
  Vector4d thrust_and_torque =
      config.mixer_coeffs * w_mb_b.cwiseAbs2();

  // Cross product matrix of body rotation rate
  Matrix3d w_bn_x = dynamics_math::Skew(w_bn_b);

  // Position derivative
  r_bn_n_dot = v_bn_n;

  // Velocity derivative
  Vector3d thrust_b = {0, 0, thrust_and_torque[0]};
  v_bn_n_dot = -C_bn.transpose() * thrust_b / config.mass +
               Vector3d(0, 0, dynamics_math::GRAV);

  // DCM derivative
  C_bn_dot = -w_bn_x * C_bn;

  // Body rotation rate derivative
  // TODO: Check the sign on the motor gyroscopic effect
  w_bn_b_dot =
      config.body_moi_inv *
      (thrust_and_torque.tail(3) + config.prop_moi_z * w_bn_x * motor_gyroscopic -
       w_bn_x * config.body_moi * w_bn_b);

  // Put everything into the output dydt
  dydt[0] = r_bn_n_dot(0);
  dydt[1] = r_bn_n_dot(1);
  dydt[2] = r_bn_n_dot(2);

  dydt[3] = v_bn_n_dot(0);
  dydt[4] = v_bn_n_dot(1);
  dydt[5] = v_bn_n_dot(2);

  dydt[6]  = C_bn_dot(0, 0);
  dydt[7]  = C_bn_dot(0, 1);
  dydt[8]  = C_bn_dot(0, 2);
  dydt[9]  = C_bn_dot(1, 0);
  dydt[10] = C_bn_dot(1, 1);
  dydt[11] = C_bn_dot(1, 2);
  dydt[12] = C_bn_dot(2, 0);
  dydt[13] = C_bn_dot(2, 1);
  dydt[14] = C_bn_dot(2, 2);

  dydt[15] = w_bn_b_dot(0);
  dydt[16] = w_bn_b_dot(1);
  dydt[17] = w_bn_b_dot(2);

  dydt[18] = w_mb_b_dot(0);
  dydt[19] = w_mb_b_dot(1);
  dydt[20] = w_mb_b_dot(2);
  dydt[21] = w_mb_b_dot(3);
  
  return GSL_SUCCESS;
}

void QuadcopterModel::Integrate(double t, double dt) {
  // Need to allocate space and provide the function for the state derivative   
  double s[22];
  double dsdt[22];

  s[0]  = state_.position_ned(0);
  s[1]  = state_.position_ned(1);
  s[2]  = state_.position_ned(2);

  s[3]  = state_.velocity_ned(0);
  s[4]  = state_.velocity_ned(1);
  s[5]  = state_.velocity_ned(2);

  s[6]  = state_.dcm_body_ned(0, 0);
  s[7]  = state_.dcm_body_ned(0, 1);
  s[8]  = state_.dcm_body_ned(0, 2);
  s[9]  = state_.dcm_body_ned(1, 0);
  s[10] = state_.dcm_body_ned(1, 1);
  s[11] = state_.dcm_body_ned(1, 2);
  s[12] = state_.dcm_body_ned(2, 0);
  s[13] = state_.dcm_body_ned(2, 1);
  s[14] = state_.dcm_body_ned(2, 2);

  s[15] = state_.rot_rate_body_ned(0);
  s[16] = state_.rot_rate_body_ned(1);
  s[17] = state_.rot_rate_body_ned(2);

  s[18] = state_.motor_rot_rate(0);
  s[19] = state_.motor_rot_rate(1);
  s[20] = state_.motor_rot_rate(2);
  s[21] = state_.motor_rot_rate(3);

  ParamsWrapper params = {motor_setpoint_, config_};

  gsl_odeiv2_system sys = {GetStateDerivative, nullptr, 22, &params};
  gsl_odeiv2_driver *d = gsl_odeiv2_driver_alloc_y_new(&sys, gsl_odeiv2_step_rkf45, 1e-6, 1e-6, 0);
  int status = gsl_odeiv2_driver_apply(d, &t, t + dt, s);
  
  state_.position_ned(0)      = s[0];
  state_.position_ned(1)      = s[1];
  state_.position_ned(2)      = s[2];
                                    
  state_.velocity_ned(0)      = s[3];
  state_.velocity_ned(1)      = s[4];
  state_.velocity_ned(2)      = s[5];
                                    
  state_.dcm_body_ned(0, 0)   = s[6];
  state_.dcm_body_ned(0, 1)   = s[7];
  state_.dcm_body_ned(0, 2)   = s[8];
  state_.dcm_body_ned(1, 0)   = s[9];
  state_.dcm_body_ned(1, 1)   = s[10];
  state_.dcm_body_ned(1, 2)   = s[11];
  state_.dcm_body_ned(2, 0)   = s[12];
  state_.dcm_body_ned(2, 1)   = s[13];
  state_.dcm_body_ned(2, 2)   = s[14];
                                     
  state_.rot_rate_body_ned(0) = s[15];
  state_.rot_rate_body_ned(1) = s[16];
  state_.rot_rate_body_ned(2) = s[17];
                                     
  state_.motor_rot_rate(0)    = s[18];
  state_.motor_rot_rate(1)    = s[19];
  state_.motor_rot_rate(2)    = s[20];
  state_.motor_rot_rate(3)    = s[21];

  // Get body frame rotation and specific acceleration for IMU sim
  GetStateDerivative(t, s, dsdt, &params);
  accel_ = state_.dcm_body_ned * (Vector3d(dsdt[3], dsdt[4], dsdt[5]) - Vector3d(0, 0, dynamics_math::GRAV));
  gyro_ = state_.rot_rate_body_ned;
}
