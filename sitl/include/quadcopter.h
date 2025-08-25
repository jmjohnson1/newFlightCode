#include <Eigen/Dense>

using namespace Eigen;

typedef struct QuadState {
  Vector3d position_lla = Vector3d::Zero();
  Vector3d position_ned = Vector3d::Zero();
  Vector3d velocity_ned = Vector3d::Zero();
  Matrix3d dcm_body_ned = Matrix3d::Identity();
  Vector3d rot_rate_body_ned = Vector3d::Zero();
  Vector4d motor_rot_rate = Vector4d::Zero();
} QuadState;

typedef struct QuadConfig {
  double tau_motor;
  double min_motor_rate, max_motor_rate;
  double mass;
  double prop_moi_z;
  Matrix3d body_moi, body_moi_inv;
  Matrix4d mixer_coeffs;
} QuadConfig;

typedef struct ParamsWrapper {
  Vector4f motor_setpoint;
  QuadConfig config;
} ParamsWrapper;

class QuadcopterModel {
 public:
  QuadcopterModel(QuadState state, QuadConfig config);
  
  void Integrate(double t, double dt);

  // Setters and getters
  Vector3d GetPositionLLA() { return state_.position_lla; }
  Vector3d GetPositionNED() { return state_.position_ned; }
  Vector3d GetVelocityNED() { return state_.velocity_ned; }
  Matrix3d GetDCM_b_n() { return state_.dcm_body_ned; }
  Vector3d GetOmega_b_n() { return state_.rot_rate_body_ned; }
  Vector4d GetMotorRates() { return state_.motor_rot_rate; }
  QuadState GetState() { return state_; }

  Vector3f GetTrueAccel() { return accel_; }
  Vector3f GetTrueGyro() { return gyro_; }

  void SetPositionLLA(const Vector3d &pos) { state_.position_lla = pos; }
  void SetPositionNED(const Vector3d &pos) { state_.position_ned = pos; }
  void SetVelocityNED(const Vector3d &vel) { state_.velocity_ned = vel; }
  void SetDCM_b_n(const Matrix3d &dcm) { state_.dcm_body_ned = dcm; }
  void SetOmega_b_n(const Vector3d &omega) { state_.rot_rate_body_ned = omega; }
  void SetMotorRates(const Vector4d &omega) { state_.motor_rot_rate = omega; }
  void SetState(const QuadState &state) { state_ = state; }

  void SetMotorInput(const Vector4d &motor_inputs) {motor_setpoint_ = motor_inputs;}

 private:
  // making this static so that I can pass it into the ODE framework
  static int GetStateDerivative(double t, const double y[], double dydt[], void *params);

  Vector4f motor_setpoint_ = Eigen::Vector4d::Zero();
  QuadState state_;
  QuadConfig config_;
  Vector3f gyro_ = Vector3f::Zero();
  Vector3f accel_ = Vector3f::Zero();
  
};
