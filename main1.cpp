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
#include <deque>
#include <fstream>

#define SAMPLING_FREQUENCY  44100

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
    float sustain_amp;
    float attack_amp;

public:
    bool note_on;
    bool got_start_time;
    bool got_end_time;
    int start_time;
    int end_time;
    int process_time;
    int fade_out_time;
    bool envelope_unactive;

    envelope(float at, float dt, float rt) {
        attack_time = at * SAMPLING_FREQUENCY / 1000;
        decay_time = dt * SAMPLING_FREQUENCY / 1000;
        release_time = rt * SAMPLING_FREQUENCY / 1000;
        sustain_amp = 0.9;
        attack_amp = 1;
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

Node noise(Input input, envelope env) {
    Input note_on_in;
    note_on_in.type = NOTE_ON_IN;

    int step = 0;
    float noise;

    auto func = [=] (std::span<float> sig) mutable -> float { 
        step += 1;
        float note_on = sig[1];
        float velocity = sig[2];
        env.get_start_time(step, note_on);
        env.get_end_time(step, note_on);

        //if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            noise = (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0);  //* env_amp * 0.1;

            return noise + sig[0]; 
        //} else {
        //    return sig[0];
        //}
    };
    std::vector<Input> inputs{input, note_on_in};
    Node node{func, inputs};
    return node;
}

Node square(Input input, float freq, std::size_t midi_idx, envelope env) {
    int step = 0;
    Input midi_in;
    Input LFO_in;
    midi_in.type = MIDI;
    midi_in.midi_idx = midi_idx;
    LFO_in.type = LFO_IN;

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

            if((sin(2 * M_PI * cycles + 2 * M_PI * lfo_amp)) > 0) {
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

Node sine(Input input, float freq, std::size_t midi_idx, envelope env) {
    int step = 0;

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

Node triangle(Input input, float freq, std::size_t midi_idx, envelope env) {
    int step = 0;
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

Node sawtooth(Input input, float freq, std::size_t midi_idx, envelope env) {
    int step = 0;
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

Node biquad_lowpass(const Input input, const float f0, const float Q) {  // template and low pass test
    float x1 = 0.0; // sample buffers
    float x2 =0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float omega = 2 * M_PI * f0 / SAMPLING_FREQUENCY;   // used constants
        float alpha = sin(omega) / 2 * Q;
        
        float b0 = (1 - cos(omega)) / 2;
        float b1 = (1 - cos(omega)); 
        float b2 = ((1 - cos(omega)) / 2);
        float a0 = (1 + alpha);
        float a1 = (-2 * cos(omega));
        float a2 = (1 - alpha) ;
        
        float output = sig[0] * b0 / a0 + x1 * b1 / a0 + x2 * b2 / a0 - y1 * a1 / a0 - y2 * a2 / a0;

        x2 = x1;
        x1 = sig[0];
        y2 = y1;
        y1 = output;
        return output;
    };

    std::vector<Input> inputs{input};
    Node node(func, inputs);

    return node;
}

Node biquad_highpass(const Input input, const float f0, const float Q) {  // template and low pass test
    float x1 = 0.0; // sample buffers
    float x2 =0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float omega = 2 * M_PI * f0 / SAMPLING_FREQUENCY;   // used constants
        float alpha = sin(omega) / 2 * Q;
        
        float b0 = (1 + cos(omega)) / 2;
        float b1 = -(1 + cos(omega)); 
        float b2 = ((1 + cos(omega)) / 2);
        float a0 = (1 + alpha);
        float a1 = (-2 * cos(omega));
        float a2 = (1 - alpha) ;
        
        float output = sig[0] * b0 / a0 + x1 * b1 / a0 + x2 * b2 / a0 - y1 * a1 / a0 - y2 * a2 / a0;

        x2 = x1;
        x1 = sig[0];
        y2 = y1;
        y1 = output;
        return output;
    };

    std::vector<Input> inputs{input};
    Node node(func, inputs);

    return node;
}

Node biquad_bandpass(const Input input, const float f0, const float BW) {  // BW in octaves
    float x1 = 0.0; // sample buffers
    float x2 =0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float omega = 2 * M_PI * f0 / SAMPLING_FREQUENCY;   // used constants
        float alpha = sin(omega) * sinh(M_LN2 /2 * BW * omega /sin(omega));
        
        float b0 = alpha;
        float b1 = 0; 
        float b2 = -alpha;
        float a0 = 1 + alpha;
        float a1 = -2 * cos(omega);
        float a2 = 1 - alpha;
        
        float output = sig[0] * b0 / a0 + x1 * b1 / a0 + x2 * b2 / a0 - y1 * a1 / a0 - y2 * a2 / a0;

        x2 = x1;
        x1 = sig[0];
        y2 = y1;
        y1 = output;
        return output;
    };

    std::vector<Input> inputs{input};
    Node node(func, inputs);

    return node;
}

Node biquad_notch(const Input input, const float f0, const float BW) {  // BW in octaves
    float x1 = 0.0; // sample buffers
    float x2 = 0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float omega = 2 * M_PI * f0 / SAMPLING_FREQUENCY;   // used constants
        float alpha = sin(omega) * sinh(M_LN2 /2 * BW * omega /sin(omega));
        
        float b0 = 1;
        float b1 = -2 * cos(omega);
        float b2 = 1;
        float a0 = 1 + alpha;
        float a1 = -2 * cos(omega);
        float a2 = 1 - alpha;
        
        float output = sig[0] * b0 / a0 + x1 * b1 / a0 + x2 * b2 / a0 - y1 * a1 / a0 - y2 * a2 / a0;

        x2 = x1;
        x1 = sig[0];
        y2 = y1;
        y1 = output;
        return output;
    };

    std::vector<Input> inputs{input};
    Node node(func, inputs);

    return node;
}

Node low_pass(Input input, float cutoff, bool lfo_modulation) {
    Input lfo_in;
    lfo_in.type = LFO_IN;

    float tg = tan(M_PI * cutoff / 44100.0);
    float dn_1 = 0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        
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
        float filter_out = -1 * (a1_coef * sig[0] + dn_1);
        dn_1 = sig[0] - a1_coef * filter_out;

        
        return filter_out + sig[0];
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;
}

// AUDIO FX

Node delay(const Input input, const int time_ms, const int feedback, const float mix) {
    std::deque<float> delay_buffer;
    delay_buffer.resize(time_ms * 45 * feedback, 0);

    auto func = [=] (const std::span<float> sig) mutable -> float {
        delay_buffer.pop_back();
        delay_buffer.push_front(sig[0]);

        float delayed_sig = 0;
        for(int i = 0; i <= feedback; i++) {
            delayed_sig += powf(mix, i) * delay_buffer[time_ms * 44.1 * i];
        }

        return delayed_sig;
    };

    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;
}

// AMPLITUDE CONTROL NODES

Node master_gain(const Input input, const float gain) {
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
        return sig[0] * ((sig[1] / 2.0) + 1);
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
        
        audio_buff[i] = sig[0];
        i++;

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

Node save_signal(Input input, float msec) {
    std::vector<float> signal;
    int total_i = 0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        
        signal.push_back(sig[0]);
        total_i++;

        if (total_i == msec * SAMPLING_FREQUENCY / 1000) {
            std::ofstream file("signal.csv");
            float time = 0;
            for(auto& sample: signal) {
                file << sample << "; " << time/44100 << "\n";
                time += 1.0;
            }
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

    std::vector<float> note_freq = calculate_frequencies(220.0);
    std::vector<Input> inputs_definition;
    Synth synth;
    envelope env(10, 30, 100);

    synth.midi.open_port(0);
    synth.midi.ignore_types(false, false, false);

    Node zero = start_node();
    Input zero_in = synth.addNode(zero);

    Node noise_generator = noise(zero_in, env);
    Input noise_in = synth.addNode(noise_generator);


     
    /*for(std::size_t i = 48; i <= 72; i++) {
        if(i == 48) {
            Node osc = triangle(zero_in, note_freq[i], i, env);
            Input osc_in = synth.addNode(osc);
            inputs_definition.push_back(osc_in);
        } else {
            Node osc = triangle(inputs_definition[i-49], note_freq[i], i, env);
            Input osc_in = synth.addNode(osc);
            inputs_definition.push_back(osc_in);
        }
    }*/
    
    

    Node filter = biquad_lowpass(noise_in, 1000, 0.707);
    Input filter_in = synth.addNode(filter);

    //Node delaynode = delay(filter_in, 200, 5, 0.5);
    //Input delay_in = synth.addNode(delaynode);
    
    //Node osc = square(zero_in, 440.0, 57);
    //Input osc_in = synth.addNode(osc);

    //Node vib = vibrato(delay_in);
    //Input vib_in = synth.addNode(vib);

    Node master = master_gain(filter_in, 2000);
    Input master_in = synth.addNode(master);

    Node output = audio_out(master_in, id);
    Input out = synth.addNode(output);

    int iter;

    while(true) {
        iter++;

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