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
    Input midi_in;
    midi_in.type = MIDI;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100 / sig[2];
        float cycles = step / length;
        return (1 * sin(2 * M_PI * cycles)+ sig[0]) * sig[1] * 2000.0 * sig[3];
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

Node envelope(Input input) {
    float attack_time = 8000;
    float decay_time = 12000;
    float sustain_time = 40000;
    float release_time = 12000;
    int sustain_amp = 2000;
    int attack_amp = 2500;
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        int gain;

        if(step <= attack_time) {
            gain = (step / attack_time) * attack_amp;
        }

        if(step > attack_time && step <= (attack_time + decay_time)) {
            gain = ((step - attack_time) / decay_time) * (sustain_amp - attack_amp) + attack_amp;
        }

        if(step > (attack_time + decay_time) && step <= (attack_time + decay_time + sustain_time)) {
            gain = sustain_amp;
        }

        if(step > (attack_time + decay_time + sustain_time) && step <= (attack_time + decay_time + sustain_time + release_time)) {
            gain = ((step - (attack_time + decay_time + sustain_time)) / release_time) * (0 - sustain_amp) + sustain_amp;
        } 

        if (step > (attack_time + decay_time + sustain_time + release_time)) {
            gain = 0;
        }

        step += 1;
        return sig[0] * gain;
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;    
}

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

struct MidiMsg {
    union {
        NoteOn note_on;
        NoteOff note_off;
    };

    MidiMsgType type;
};

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
                        std::cout << "nothing" << std::endl;
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

    Node osc2 = sine(zero_in);
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