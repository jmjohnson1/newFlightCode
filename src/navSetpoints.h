#ifndef NAV_SETPOINTS_H
#define NAV_SETPOINTS_H

#include "common.h"

void SetpointHandler(NavData_t *navData, Quadcopter_t *quadData);
void TakeoffSetpoints(NavData_t *navData, Quadcopter_t *quadData);
void LandingSetpoints(NavData_t *navData, Quadcopter_t *quadData);
void MissionSetpoints(NavData_t *navData, Quadcopter_t *quadData);

#endif