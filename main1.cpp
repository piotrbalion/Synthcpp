#include <functional>
#include <vector>
#include <iostream>
#include <cmath>
#include <array>
#include <unordered_map>
#include <SDL2/SDL.h>
#include <libremidi/libremidi.hpp>
#include <optional>


enum InputType {
    NODE,
    MIDI,
};

// Input
struct Input {
    std::size_t idx;
    InputType type;
};

// Envelope generator class

class envelope {
    float attack_time;
    float decay_time;
    float release_time;
    int sustain_amp;
    int attack_amp;

public:
    bool note_on;
    bool got_start_time;
    bool got_end_time;
    int start_time;
    int end_time;
    int process_time;
    int fade_out_time;

    envelope() {
        attack_time = 800;
        decay_time = 1200;
        release_time = 20000;
        attack_amp = 2500;
        sustain_amp = 2000;
        start_time = 0;
        end_time = 0;
        got_start_time = false;
        got_end_time = false;
    }

    // getting time of Note On message

    void get_start_time(int step, bool Note_On) {
        if(got_start_time == false && Note_On == true)
        {
            start_time = step;
            got_start_time = true;
        }
    }

    // getting time of Note Off message

    void get_end_time(int step, bool Note_On) {
        if(got_end_time == false && Note_On == false)
        {
            end_time = step;
            got_end_time = true;
        }
    }

    // Amplitude generation with envelope

    float process(int t, bool Note_On) {
        float amp = 0.0;
        
        if(Note_On == true) {
            process_time = t - start_time;

            if(process_time <= attack_time) {   // ATTACK
                amp = (process_time / attack_time) * attack_amp;
            }

            if(process_time > attack_time && process_time <= (attack_time + decay_time)) {  // DECAY
                amp = ((process_time - attack_time) / decay_time) * (sustain_amp - attack_amp) + attack_amp;
            }

            if(process_time > attack_time + decay_time) {   // SUSTAIN
                amp = sustain_amp;
            }
            got_end_time = false;
            fade_out_time = 0;
            end_time = 0;
        }
        else if(Note_On == false) {     // RELEASE
            fade_out_time = t - end_time;
            amp = (fade_out_time / release_time) * (0 - sustain_amp) + sustain_amp;
            got_start_time = false;
            start_time = 0;
            process_time = 0;
        }
        if(amp < 0) {
            amp = 0;
        }
        return amp;
    }
};

// Node
struct Node {
    std::function<float(std::vector<float>)> func;
    std::vector<Input> inputs;
    float output;

    Node(std::function<float(std::vector<float>)> func, std::vector<Input> inputs) {
        this->func = func;
        this->inputs = inputs;
    }

    void compute(std::vector<float> inputs) {
        this->output = this->func(inputs);
    }
};

Node start_node() {
    auto func = [=] (std::vector<float> sig) mutable -> float { return 0; };
    std::vector<Input> inputs;
    Node node{func, inputs};
    return node;
}

// OSCILATOR NODES

Node square(Input input) {
    int step = 0;
    Input midi_in;
    midi_in.type = MIDI;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100.0 / sig[2];
        if(fmod(step, length) > length / 2) {
            return (1.0 + sig[0]) * sig[1] * 2000.0 * sig[3];
        } else {
            return (-1.0 + sig[0]) * sig[1] * 2000.0 * sig[3];
        }
    };
    std::vector<Input> inputs = {input, midi_in};
    
    Node node{func, inputs};
    return node;
}

Node sine(Input input) {
    int step = 0;
    envelope env;

    Input midi_in;
    midi_in.type = MIDI;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1; 
        env.get_start_time(step, sig[1]);
        env.get_end_time(step, sig[1]);
        float env_amp = env.process(step, (bool)sig[1]);
        

        const float length = 44100 / sig[2];
        float cycles = step / length;
        return (sin(2 * M_PI * cycles)+ sig[0]) * sig[3] * env_amp;
    };
    std::vector<Input> inputs{input, midi_in};

    Node node{func, inputs};
    return node;
}

Node triangle(Input input, float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100 / freq;
        float cycles = step / length;
        return asin(sin(2 * M_PI * cycles)) * 2 / M_PI+ sig[0];
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;
}

Node sawtooth(Input input, float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100.0 / freq;
        int cycle = fmod(step, length);
        return (cycle/length) * 2.0 - 1.0 + sig[0];
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;
}

// AMPLITUDE CONTROL NODES

Node gain(Input input, float gain) {
    auto func = [=] (std::vector<float> sig) mutable -> float {
        return sig[0] * gain;
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;    
}

// Audio output management node

Node audio_out(Input input, int id) {
    std::array<short, 128> samples;
    int i = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        samples[i] = sig[0];
        i += 1;

        if(SDL_GetQueuedAudioSize(id)/sizeof(short) > 4410) {
            SDL_Delay(1);
        }

        if(i == 128) {
            SDL_QueueAudio(id, samples.data(), samples.size() * sizeof(short));
            i = 0;
        }
        return 0;
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;    
}

// Makes an array of frequencies based on TET system

std::vector<float> calculate_frequencies(float tuning_frequency) {
    std::vector<float> notes;
    float note;
    
    for(int i = 0; i < 88; i++) {
        note = tuning_frequency * pow(pow(2.0, 1.0/12.0), (-57.0 + i));
        notes.push_back(note);
    }

    return notes;
}

struct NoteOn {
    float vel;
    float freq;
};

struct NoteOff {
    float freq;
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
    std::vector<float> note_freq = calculate_frequencies(440.0);

    auto next_message = midi.get_message();
    if(!next_message.bytes.empty()) {
        if(next_message.bytes[0] == 144) {
            msg.type = NOTE_ON;
            msg.note_on.freq = note_freq[next_message.bytes[1]];
            msg.note_on.vel = (float)next_message.bytes[2] / 127;
            std::cout << "NOTE_ON " << "frequency: " << msg.note_on.freq << " velocity: " <<  msg.note_on.vel << std::endl;
            
        } else if(next_message.bytes[0] == 128) {
            msg.type = NOTE_OFF;
            msg.note_on.freq = note_freq[next_message.bytes[1]];
            std::cout << "NOTE_OFF " << "frequency:" << msg.note_on.freq << std::endl;
        }
        return msg;       
    } else {
        return {};
    }

}

/// Synth
class Synth {
    std::vector<Node> nodes;
public:
    libremidi::midi_in midi;
    MidiMsg msg;
    Synth() = default;


    // ADDING NODES TO VECTOR

    Input addNode(Node& node) {
        Input in;
        std::size_t index = this -> nodes.size();
        
        in.idx = index;
        in.type = NODE;

        this -> nodes.push_back(node);
        return in;
    }

    // PROCESSING ALL NODES

    void step() {
        std::vector<float> outputs;
        auto maybe_msg = get_midi_input(midi);
        if(maybe_msg.has_value()) {
            msg = *maybe_msg;
        }

        for(const Node& node: this->nodes) {
            outputs.push_back(node.output);
        }

        for(Node& node: this->nodes) {
            std::vector<float> inputs;

            for(Input& in : node.inputs) {
                float value;
                if(in.type == NODE) {
                    value = outputs[in.idx];
                    inputs.push_back(value);
                } else if(in.type == MIDI) {      
                    if(msg.type == NOTE_OFF) {
                        inputs.push_back(msg.type);
                        inputs.push_back(msg.note_off.freq);
                    } else if(msg.type == NOTE_ON) {
                        inputs.push_back(msg.type); 
                        inputs.push_back(msg.note_on.freq);
                        inputs.push_back(msg.note_on.vel);
                    } else {
                        inputs.push_back(0); 
                        inputs.push_back(0);
                        inputs.push_back(0);
                    }
                }
            }
            auto output_new = node.func(inputs);
            node.output = output_new;
        }
    }

};


int main() {

    SDL_Init(SDL_INIT_AUDIO);
	SDL_AudioSpec spec, aspec;
	SDL_zero(spec);
	spec.freq = 44100; 
	spec.format = AUDIO_S16SYS;
	spec.channels = 1;
	spec.samples = 512;
	spec.callback = nullptr;
	spec.userdata = nullptr;

    int id;
	if ((id = SDL_OpenAudioDevice(nullptr, 0, &spec, &aspec, SDL_AUDIO_ALLOW_ANY_CHANGE)) <= 0 )
	{
	  fprintf(stderr, "Couldn't open audio: %s\n", SDL_GetError());
	  exit(-1);
	}

    SDL_PauseAudioDevice(id, 0);

    // ADDING NODES
    Input in;

    Synth synth;

    synth.midi.open_port(0);
    synth.midi.ignore_types(false, false, false);

    Node zero = start_node();
    Input zero_in = synth.addNode(zero);

    Node osc2 = square(zero_in);
    Input osc2_in = synth.addNode(osc2);

    Node output = audio_out(osc2_in, id);
    Input out = synth.addNode(output);

    

    while(true) {
        synth.step();
        SDL_Event event;
        
        //std::cout << midi_rec.note  << std::endl;
        while (SDL_PollEvent(&event)) {
            switch(event.type) {
                case SDL_QUIT:
                    exit(1);
                    break;
                default:
                    break;
            }
        }
    }
}