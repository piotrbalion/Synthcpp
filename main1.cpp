#include <functional>
#include <vector>
#include <iostream>
#include <cmath>
#include <array>
#include <unordered_map>
#include <SDL2/SDL.h>

/* no audio feedback yet due to problems with audio libraries */

/* the node "out" will be modified to handle audio output */

// Input
struct Input {
    std::size_t idx;
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

// OSCILATOR NODES

Node square(float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100.0 / freq;
        if(fmod(step, length) > length / 2) {
            return 1.0;
        } else {
            return -1.0;
        }
    };
    std::vector<Input> inputs;

    Node node{func, inputs};
    return node;
}

Node sine(float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100 / freq;
        float cycles = step / length;
        return 1 * sin(2 * M_PI * cycles);
    };
    std::vector<Input> inputs;

    Node node{func, inputs};
    return node;
}

Node triangle(float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100 / freq;
        float cycles = step / length;
        return asin(sin(2 * M_PI * cycles)) * 2 / M_PI;
    };
    std::vector<Input> inputs;

    Node node{func, inputs};
    return node;
}

Node sawtooth(float freq) {
    int step = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        step += 1;
        const float length = 44100.0 / freq;
        int cycle = fmod(step, length);
        return (cycle/length) * 2.0 - 1.0;
    };
    std::vector<Input> inputs;

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
    std::array<short, 512> samples;
    int i = 0;

    auto func = [=] (std::vector<float> sig) mutable -> float {
        samples[i] = sig[0];
        i += 1;

        if(SDL_GetQueuedAudioSize(id)/sizeof(short) > 4410) {
            SDL_Delay(1);
        }

        if(i == 512) {
            SDL_QueueAudio(id, samples.data(), samples.size() * sizeof(short));
            i = 0;
        }
        return 0;
    };
    std::vector<Input> inputs{input};

    Node node{func, inputs};
    return node;    
}

/// Synth
class Synth {
    std::vector<Node> nodes;
public:
    Synth() = default;
    
    // ADDING NODES TO VECTOR

    Input addNode(Node& node) {
        Input in;
        std::size_t index = this -> nodes.size();
        
        in.idx = index; 
        this -> nodes.push_back(node);
        return in;
    }

    // PROCESSING ALL NODES

    void step() {
        std::vector<float> outputs;
        for(const Node& node: this->nodes) {
            outputs.push_back(node.output);
        }

        for(Node& node: this->nodes) {
            std::vector<float> inputs;

            for(Input& in : node.inputs) {
                auto value = outputs[in.idx];
                inputs.push_back(value);
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

    Synth synth;
    Node osc = sawtooth(220.0);
    Input osc_in = synth.addNode(osc);

    Node env = envelope(osc_in);
    Input env_in = synth.addNode(env);

    Node output = audio_out(env_in, id);
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