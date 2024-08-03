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

#ifndef UNITS_SRC_CONVACC_H_  // NOLINT
#define UNITS_SRC_CONVACC_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#endif

namespace bfs {
/* Units for measuring linear acceleration */
enum class LinAccUnit : int8_t {
  FPS2,   // feet per second per second, ft/s/s
  MPS2,   // meters per second per second, m/s/s
  KPS2,   // kilometers per second per second, km/s/s
  IPS2,   // inches per second per second, in/s/s
  KPHPS,  // kilometers per hour per second, km/h/s
  MPHPS,  // miles per hour per second, mi/h/s
  G,      // G force acceleration, G
};
/* 
* Utility to convert between linear acceleration units:
* Input the value to convert, the unit the value is currently in, and the unit
* you are converting to, i.e. 'convacc(1, LinAccUnit::G, LinAccUnit::MPS2)'
* converts 1 G to m/s/s.
*/
float convacc(const float val, const LinAccUnit input,
              const LinAccUnit output);

}  // namespace bfs

#endif  // UNITS_SRC_CONVACC_H_ NOLINT
