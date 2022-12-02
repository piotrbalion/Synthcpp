#include <SDL2/SDL.h>
#include <SDL2/SDL_audio.h>
#include <iostream>
#include <array>
#include <math.h>

class envelope {
public:
    float attack_time;
    float decay_time;
    float sustain_time;
    float release_time;
    int sustain_amp;
    int attack_amp;

    envelope() {
        attack_time = 8000;
        decay_time = 12000;
        sustain_time = 40000;
        release_time = 20000;
        sustain_amp = 1500;
        attack_amp = 2500;
    }

    int process(float t) {
        int amp;

        if(t <= attack_time) {
            amp = (t / attack_time) * attack_amp;
        }

        if(t > attack_time && t <= (attack_time + decay_time)) {
            amp = ((t - attack_time) / decay_time) * (sustain_amp - attack_amp) + attack_amp;
        }

        if(t > (attack_time + decay_time) && t <= (attack_time + decay_time + sustain_time)) {
            amp = sustain_amp;
        }

        if(t > (attack_time + decay_time + sustain_time) && t <= (attack_time + decay_time + sustain_time + release_time)) {
            amp = ((t - (attack_time + decay_time + sustain_time)) / release_time) * (0 - sustain_amp) + sustain_amp;
        } 
        return amp;
    }
};

int square(int i, int freq, int amp, int LFO) {
    float length = 44100 / freq;
    int cycles = fmod(i , length);
    if(cycles > length / 2) {
        return -amp;
    } else {
        return amp;
    }
}

int triangle(int i, int freq, int amp) {
    float length = 44100 / freq;
    double cycles = i / length;
    return amp * asin(sin(2 * M_PI * cycles)) * 2 / M_PI;
}

int sine(int i, int freq, int amp, int LFO) {
    float length = 44100 / freq;
    double cycles = i / length;
    return amp * sin(2 * M_PI * cycles);
}

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
	
    float t = 0;
    float freq = 440;

    while(true)
    {
        if(SDL_GetQueuedAudioSize(id)/sizeof(short) > 4410) {
            SDL_Delay(1);
            continue;
        }


        std::array<short, 512> samples;
        for(short &s: samples) {
            s = sine(t, freq, sine(t, 5, 1000, 0), 0);
            t += 1;
        }

        SDL_QueueAudio(id, samples.data(), samples.size() * sizeof(short));

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