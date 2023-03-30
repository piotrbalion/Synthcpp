#ifndef filters_hpp
#define filters_hpp
#include "node.hpp"

Node biquad_lowpass(const Input input, const float f0, const float Q);

Node biquad_highpass(const Input input, const float f0, const float Q);

Node biquad_bandpass(const Input input, const float f0, const float BW);

Node biquad_notch(const Input input, const float f0, const float BW);

Node phaser(const Input input);

Node wah(const Input input);


#endif