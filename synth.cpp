#include "synth.hpp"
#include <optional>


/// Synth

Synth::Synth(){
    any_note_on = 0;
    any_vel = 0;
};

// ADDING NODES TO VECTOR

Input Synth::addNode(Node& node) {
    Input in;
    std::size_t index = this -> nodes.size();
    
    in.idx = index;
    in.type = NODE;

    this -> nodes.push_back(node);
    return in;
}

// PROCESSING ALL NODES
void Synth::step() {

    this->outputs.clear();
    auto maybe_msg = get_midi_input(midi);
    lfo.sine();
    float lfo_amp = lfo.amp;
    if(maybe_msg.has_value()) {
        msg = *maybe_msg;
        if(msg.type == NOTE_ON){        //writing in state
            midi_state[msg.note_on.index].note_on = 1;
            midi_state[msg.note_on.index].velocity = msg.note_on.vel;
            any_note_on ++;
            any_vel = msg.note_on.vel;
            std::cout << midi_state[msg.note_on.index].velocity << std::endl;
        } else if(msg.type == NOTE_OFF) {
            midi_state[msg.note_on.index].note_on = 0;
            any_note_on --;
            if(any_note_on == 0) {
                any_vel = 0;
            }
        }
    }

    for(const Node& node: this->nodes) {
        outputs.push_back(node.output);
    }

    for(Node& node: this->nodes) {
        this->inputs.clear();

        for(Input& in : node.inputs) {
            float value;
            if(in.type == NODE) {
                value = outputs[in.idx];
                inputs.push_back(value);
            } else if(in.type == MIDI) {
                inputs.push_back(static_cast<float>(midi_state[in.midi_idx].note_on)); 
                //std::cout << "\n note_on = " << midi_state[in.midi_idx].note_on;  // it be like okay
                inputs.push_back(midi_state[in.midi_idx].velocity);
                //std::cout << "\n velocity = " << midi_state[in.midi_idx].velocity << std::endl;    // it be like also okay  
            } else if(in.type == LFO_IN) {
                inputs.push_back(lfo_amp);
            } else if(in.type == NOTE_ON_IN) {
                if(any_note_on == 0) {
                    inputs.push_back(0.0);
                } else {
                    inputs.push_back(1.0);
                    inputs.push_back(any_vel);
                }
            }
            
        }
        auto output_new = node.func(inputs);
        node.output = output_new;
    }
}
