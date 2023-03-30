#ifndef envelope_hpp
#define envelope_hpp

class envelope {
    float sustain_amp;
    float attack_amp;

public:
    float attack_time;
    float decay_time;
    float release_time;
    bool note_on;
    bool got_start_time;
    bool got_end_time;
    int start_time;
    int end_time;
    int process_time;
    int fade_out_time;
    bool envelope_unactive;


    envelope(float at, float dt, float rt);
    // getting time of Note On message

    void get_start_time(int step, bool Note_On);

    // getting time of Note Off message

    void get_end_time(int step, bool Note_On);

    // Amplitude generation with envelope

    float process(int t, bool Note_On);
};

#endif