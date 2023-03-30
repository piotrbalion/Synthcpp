#ifndef audiofx_hpp
#define audiofx_hpp
#include "node.hpp"

Node tremolo(Input input);

Node delay(const Input input, const int time_ms, const int feedback, const float mix);


#endif