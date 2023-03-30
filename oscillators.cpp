
#include <cmath>
#include "oscillators.hpp"

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

        if(env.envelope_unactive == false) {
            float env_amp = env.process(step, note_on);
            noise = (2.0 * ((double)rand() / (double)RAND_MAX) - 1.0)* env_amp * 0.1;

            return noise + sig[0]; 
        } else {
            return sig[0];
        }
    };
    std::vector<Input> inputs{input, note_on_in};
    Node node{func, inputs};
    std::cout << "Added noise generator" << std::endl;
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

    std::cout << "Added square" << std::endl;
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
        float lfo_amp = sig[3];

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
    std::cout << "Added sine" << std::endl;
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
    std::cout << "Added triangle" << std::endl;
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
    std::cout << "Added sawtooth" << std::endl;
    return node;
}

