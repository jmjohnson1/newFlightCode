/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2023 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#ifndef UNITS_SRC_CONVVEL_H_  // NOLINT
#define UNITS_SRC_CONVVEL_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#endif

namespace bfs {
/* Units for measuring linear velocity */
enum class LinVelUnit : int8_t {
  FPS,  // feet per second, ft/s
  MPS,  // meters per second, m/s
  KPS,  // kilometers per second, km/s
  IPS,  // inches per second. in/s
  KPH,  // kilometers per hour, km/h
  MPH,  // miles per hour, mi/h
  KTS,  // knots
  FPM   // feet per minute, ft/min
};
/* 
* Utility to convert between linear velocity units:
* Input the value to convert, the unit the value is currently in, and the unit
* you are converting to, i.e. 'convvel(1, LinVelUnit::FPS, LinVelUnit::MPS)'
* converts 1 ft/s to m/s.
*/
float convvel(const float val, const LinVelUnit input,
              const LinVelUnit output);

}  // namespace bfs

#endif  // UNITS_SRC_CONVVEL_H_ NOLINT
