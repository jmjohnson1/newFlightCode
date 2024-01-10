#ifndef TEST_ROUTINES_H
#define TEST_ROUTINES_H

#include "radio.h"

namespace testStand {

float SineSweep(unsigned long dt);
float Step(RadioChannel &angleCh);

}

namespace gainTuning {

float ScaleFactor(RadioChannel &GainCh);


}


#endif
