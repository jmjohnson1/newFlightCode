#include "trajectory.h"

trajectory::trajectory() {
	positionSetpoint_ = Eigen::Vector3f::Zero();
}

void trajectory::GoTo(const Eigen::Vector3f &position) {
	positionSetpoint_ = position;
}
