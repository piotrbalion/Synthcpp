#include "envelope.hpp"

#define SAMPLING_FREQUENCY  44100

envelope::envelope(float at, float dt, float rt) {
    attack_time = at *SAMPLING_FREQUENCY / 1000;
    decay_time = dt * SAMPLING_FREQUENCY / 1000;
    release_time = rt * SAMPLING_FREQUENCY / 1000;
    sustain_amp = 0.9;
    attack_amp = sustain_amp * 1.1;
    start_time = 0;
    end_time = 0;
    got_start_time = false;
    got_end_time = false;
    envelope_unactive = true;
}

void envelope::get_start_time(int step, bool Note_On) {
    if(got_start_time == false && Note_On == true)
    {
        start_time = step;
        got_start_time = true;
        envelope_unactive = false;
    }
}

// getting time of Note Off message

void envelope::get_end_time(int step, bool Note_On) {
    if(got_end_time == false && Note_On == false)
    {
        end_time = step;
        got_end_time = true;
    }
}

// Amplitude generation with envelope

float envelope::process(int t, bool Note_On) {
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