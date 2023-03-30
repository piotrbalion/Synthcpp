#ifndef synth_hpp
#define synth_hpp
#include <vector>
#include <iostream>
#include <cmath>
#include <array>
#include <libremidi/libremidi.hpp>
#include <optional>
#include <span>
#include "node.hpp"

class Lfo {
    int step;
public:
    Lfo();
    float freq;
    float amp;
    void sine() {
        step++;
        float length = 44100 / freq;
        double cycles = step / length;
        amp = sin(2 * M_PI * cycles);
    }
};

struct NoteOn {
    float vel;
    std::size_t index;
};

struct NoteOff {
    std::size_t index;
};

enum MidiMsgType {
    NOTE_OFF,
    NOTE_ON,
};

// MIDI message structure

struct MidiMsg {
    union {
        NoteOn note_on;
        NoteOff note_off;
    };

    MidiMsgType type;
};

// Creating MIDI message for MIDI input

std::optional<MidiMsg> get_midi_input(libremidi::midi_in& midi) {
    MidiMsg msg;

    auto next_message = midi.get_message();
    if(!next_message.bytes.empty()) {
        if(next_message.bytes[0] == 144) {
            msg.type = NOTE_ON;
            msg.note_on.index = next_message.bytes[1];
            msg.note_on.vel = (float)next_message.bytes[2] / 127;
            //std::cout << "NOTE_ON " << "index: " << msg.note_on.index << " velocity: " <<  msg.note_on.vel << std::endl;
            
        } else if(next_message.bytes[0] == 128) {
            msg.type = NOTE_OFF;
            msg.note_on.index = next_message.bytes[1];
            //std::cout << "NOTE_OFF " << "index:" << msg.note_on.index << std::endl;
        }
        return msg;
    } else {
        return {};
    }

}

// single key state

struct MidiState {      
    bool note_on = false;
    float velocity = 0;
};

/// Synth
class Synth {
    std::vector<Node> nodes;
public:
    Lfo lfo;
    libremidi::midi_in midi;
    std::array<MidiState, 88> midi_state;
    MidiMsg msg;
    int any_note_on = 0;
    float any_vel = 0;
    std::vector<float> outputs;
    std::vector<float> inputs;
    Synth();
    Input addNode(Node& node);
    void step();    // PROCESSING ALL NODES
};

#endif