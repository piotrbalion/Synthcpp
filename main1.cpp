#include <functional>
#include <vector>
#include <iostream>
#include <cmath>
#include <array>
#include <unordered_map>
#include <SDL2/SDL.h>
#include <libremidi/libremidi.hpp>
#include <optional>
#include <random>
#include <span>


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
    int mode;
    InputType type;
};

class Lfo {
    float freq;
    int step;
public:
    float amp;

    Lfo() {
        freq = 5;
        amp = 0;
        step = 0;
    }

    void sine() {
        step++;
        float length = 44100 / freq;
        double cycles = step / length;
        amp = sin(2 * M_PI * cycles);
    }
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
    bool envelope_unactive;

    envelope() {
        attack_time = 1000;
        decay_time = 1200;
        release_time = 20000;
        attack_amp = 2500;
        sustain_amp = 2000;
        start_time = 0;
        end_time = 0;
        got_start_time = false;
        got_end_time = false;
        envelope_unactive = true;
    }

    // getting time of Note On message

    void get_start_time(int step, bool Note_On) {
        if(got_start_time == false && Note_On == true)
        {
            start_time = step;
            got_start_time = true;
            envelope_unactive = false;
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
            envelope_unactive = false;
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
        if(Note_On == false && amp < 0) {
            amp = 0;
            envelope_unactive = true;
        }
        return amp;
    }
};

// Node
struct Node {
    std::function<float(std::span<float>)> func;
    std::vector<Input> inputs;
    float output;

    Node(std::function<float(std::span<float>)> func, std::vector<Input> inputs) {
        this->func = func;
        this->inputs = inputs;
    }

    void compute(std::span<float> inputs) {
        this->output = this->func(inputs);
    }
};

Node start_node() {
    auto func = [=] (std::span<float> sig) mutable -> float { return 0; };
    std::vector<Input> inputs;
    Node node{func, inputs};
    return node;
}

// OSCILATOR NODES

Node Noise(Input input) {
    Input note_on_in;
    note_on_in.type = NOTE_ON_IN;

    envelope env;
    int step = 0;
    float noise;

    auto func = [=] (std::span<float> sig) mutable -> float { 
        step += 1;
        float note_on = sig[1];
        float velocity = sig[2];
        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);

        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            noise = (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0) * velocity * env_amp * 0.5;

            return noise + sig[0]; 
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs{input, note_on_in};
    Node node{func, inputs};
    return node;
}

Node square(Input input, float freq, std::size_t midi_idx) {
    int step = 0;
    Input midi_in;
    Input LFO_in;
    midi_in.type = MIDI;
    midi_in.midi_idx = midi_idx;
    LFO_in.type = LFO_IN;
    envelope env;

    auto func = [=] (std::span<float> sig) mutable -> float {
        step += 1;
        float note_on = sig[1];
        float velocity = sig[2];
        float lfo_amp = sig[3] / 8;

        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);

        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            const float length = 44100.0 / freq;
            float cycles = step / length;

            if((sin(2 * M_PI * cycles + 2 * M_PI)) > 0) {
                return 1.0 * velocity * env_amp * velocity + sig[0];
            } else {
                return -1.0 * velocity * env_amp * velocity + sig[0];
            }
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs = {input, midi_in, LFO_in};
    
    Node node{func, inputs};
    return node;
}

Node sine(Input input, float freq, std::size_t midi_idx) {
    int step = 0;
    envelope env;

    Input midi_in;
    midi_in.type = MIDI;
    midi_in.midi_idx = midi_idx;
    Input lfo_in;
    lfo_in.type = LFO_IN;

    auto func = [=] (std::span<float> sig) mutable -> float {
        step += 1; 
        float note_on = sig[1];
        float velocity = sig[2];
        float lfo_amp = sig[3] / 4;

        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);
        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
        
            const float length = 44100 / freq;
            float cycles = step / length;
            return (sin(2 * M_PI * cycles + 2 * M_PI * lfo_amp)) * velocity * env_amp + sig[0];
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs{input, midi_in, lfo_in};

    Node node{func, inputs};
    return node;
}

Node triangle(Input input, float freq, std::size_t midi_idx) {
    int step = 0;
    envelope env;
    Input midi_in;
    midi_in.type = MIDI;
    midi_in.midi_idx = midi_idx;
    Input lfo_in;
    lfo_in.type = LFO_IN;

    auto func = [=] (std::span<float> sig) mutable -> float {
        step += 1; 
        float note_on = sig[1];
        float velocity = sig[2];
        float lfo_amp = sig[3] / 4;

        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);

        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            const float length = 44100.0 / freq;
            float cycles = step / length;
            return (asin(sin(2 * M_PI * cycles + 2 * M_PI * lfo_amp)) * 2 / M_PI) * velocity * env_amp + sig[0];
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs{input, midi_in, lfo_in};

    Node node{func, inputs};
    return node;
}

Node sawtooth(Input input, float freq, std::size_t midi_idx) {
    int step = 0;
    envelope env;
    Input midi_in;
    midi_in.type = MIDI;
    midi_in.midi_idx = midi_idx;

    auto func = [=] (std::span<float> sig) mutable -> float {
        step += 1; 
        float note_on = sig[1];
        float velocity = sig[2];

        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);

        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            const float length = 44100.0 / freq;
            int cycle = fmod(step, length);
            return ((cycle/length) * 2.0 - 1.0) * velocity * env_amp + sig[0];
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs{input, midi_in};

    Node node{func, inputs};
    return node;
}

// FILTERS

Node low_pass(Input input, float cutoff, bool lfo_modulation) {
    Input lfo_in;
    lfo_in.type = LFO_IN;

    float tg = tan(M_PI * cutoff / 44100.0);
    float dn_1 = 0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        if(lfo_modulation == true) {
            float new_cutoff = cutoff + sig[1]* 1000;
            float tg = tan(M_PI * new_cutoff / 44100.0);
        }
        
        float a1_coef = (tg - 1.0) / (tg + 1.0);
        float filter_out = a1_coef * sig[0] + dn_1;;
        dn_1 = sig[0] - a1_coef * filter_out;

        
        return filter_out + sig[0];
    };
    std::vector<Input> inputs{input, lfo_in};

    Node node{func, inputs};
    return node;
}

Node high_pass(Input input, float cutoff) {
    float dn_1 = 0;
    float tg = tan(M_PI * cutoff / 44100.0);
    float a1_coef = (tg - 1.0) / (tg + 1.0);

    auto func = [=] (std::span<float> sig) mutable -> float {
        float filter_out = a1_coef * sig[0] + dn_1;;
        dn_1 = sig[0] - a1_coef * filter_out;

        
        return filter_out + sig[0];
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;
}

// AMPLITUDE CONTROL NODES

Node gain(Input input, float gain) {
    auto func = [=] (std::span<float> sig) mutable -> float {
        return sig[0] * gain;
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;    
}

Node tremolo(Input input) {
    Input lfo_in;
    lfo_in.type = LFO_IN;
    auto func = [=] (std::span<float> sig) mutable -> float {
        return sig[0] * sig[1];
    };
    std::vector<Input> inputs{input, lfo_in};

    Node node{func, inputs};
    return node;    
}

// Audio output management node

Node audio_out(Input input, int id) {
    std::array<short, 512> audio_buff;
    int i = 0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        //std::cout << samples[i] << std::endl;
        audio_buff[i] = sig[0];
        i += 1;

        if(SDL_GetQueuedAudioSize(id)/sizeof(short) > 4410) {
            SDL_Delay(2);
        }

        if(i == 512) {
            SDL_QueueAudio(id, audio_buff.data(), audio_buff.size() * sizeof(short));
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
            //std::cout << any_note_on << std::endl;
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
                    //std::cout << "\n note_on = " << midi_state[in.midi_idx].note_on;  // pokazuje ze dobrze
                    inputs.push_back(midi_state[in.midi_idx].velocity);
                    //std::cout << "\n velocity = " << midi_state[in.midi_idx].velocity << std::endl;    // pokazuje ze dobrze   
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
            auto output_new = node.func(inputs);    // w nodzie square dziala zle 
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

    std::vector<float> note_freq = calculate_frequencies(440.0);
    std::vector<Input> inputs_definition;
    Synth synth;

    synth.midi.open_port(0);
    synth.midi.ignore_types(false, false, false);

    Node zero = start_node();
    Input zero_in = synth.addNode(zero);

    //Node noise_generator = Noise(zero_in);
    //Input noise_in = synth.addNode(noise_generator);


    
    for(std::size_t i = 48; i <= 72; i++) {
        if(i == 48) {
            Node osc = square(zero_in, note_freq[i], i);
            Input osc_in = synth.addNode(osc);
            inputs_definition.push_back(osc_in);
        } else {
            Node osc = square(inputs_definition[i-49], note_freq[i], i);
            Input osc_in = synth.addNode(osc);
            inputs_definition.push_back(osc_in);
        }
    }

    Node filter = low_pass(inputs_definition.back(), 1000, true);
    Input filter_in = synth.addNode(filter);
    
    //Node osc = square(zero_in, 440.0, 57);
    //Input osc_in = synth.addNode(osc);
    
    //Node osc2 = square(osc_in, 220.0, 59);
    //Input osc2_in = synth.addNode(osc2);

    // Node vib = vibrato(oscilator_in.back());
    // Input vib_in = synth.addNode(vib);

    Node output = audio_out(filter_in, id);
    Input out = synth.addNode(output);



    while(true) {
        synth.step();
        SDL_Event event;
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