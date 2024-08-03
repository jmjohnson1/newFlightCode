#ifndef TEST_ROUTINES_H
#define TEST_ROUTINES_H

#include "radio.h"

namespace testStand {

float SineSweep(bool restart);
float Step(RadioChannel &angleCh);
float ThrustSweep(bool restart);

}

namespace gainTuning {

float ScaleFactor(RadioChannel &GainCh);

}


#endif
