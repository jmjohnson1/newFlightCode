#ifndef TELEM_H_
#define TELEM_H_

#include "common.h"

constexpr std::size_t NUM_PARAMS = 2;
constexpr std::size_t NUM_UTM = 0; 

// The best way I could think of dealing with a linking issue was to put this in
// a class. For some reason, declaring the MavLink object as a global variable
// in the cpp file throws an exception. I don't know why. I spent too long
// trying to figure it out. This is what we get. This solution may be
// problematic. I don't care.
class telem {
public:
  bool Begin(MissionData_t &missionData);
  void Run();
private:
  bfs::MavLink<NUM_PARAMS, NUM_UTM> *mavlink;
};

#endif