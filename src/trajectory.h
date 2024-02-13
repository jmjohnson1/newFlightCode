#ifndef TRAJECTORY_H
#define TRAJECTORY_H
// Note:
// This is all sort of in flux

#include "eigen.h"

class trajectory {
public:
	trajectory();
	Eigen::Vector3d GetSetpoint() { return positionSetpoint_; }
	inline void GoTo(const Eigen::Vector3d &position) {
		positionSetpoint_ = position;
	}
private:
	Eigen::Vector3d positionSetpoint_;
};

#endif
