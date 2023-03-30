#ifndef oscillators_hpp
#define oscillators_hpp
#include "node.hpp"
#include "envelope.hpp"

Node noise(Input input, envelope env);

Node square(Input input, float freq, std::size_t midi_idx, envelope env);

Node sine(Input input, float freq, std::size_t midi_idx, envelope env);

Node triangle(Input input, float freq, std::size_t midi_idx, envelope env);

Node sawtooth(Input input, float freq, std::size_t midi_idx, envelope env);

#endif