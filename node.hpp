#ifndef node_hpp
#define node_hpp
#include <iostream>
#include <functional>
#include <span>
#include <vector>

enum InputType {
    NODE,
    MIDI,
    LFO_IN,
    NOTE_ON_IN,
};

// Input
struct Input {
    std::size_t idx;
    std::size_t midi_idx;
    InputType type;
};

struct Node {
    std::function<float(std::span<float>)> func;
    std::vector<Input> inputs;
    float output;
    

    Node(std::function<float(std::span<float>)> func, std::vector<Input> inputs);

    void compute(std::span<float> inputs);
};

#endif