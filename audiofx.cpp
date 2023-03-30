#include "audiofx.hpp"
#include <deque>
#include <cmath>

Node tremolo(Input input) {
    Input lfo_in;
    lfo_in.type = LFO_IN;
    auto func = [=] (std::span<float> sig) mutable -> float {
        return sig[0] * ((sig[1] / 2.0) + 1);
    };
    std::vector<Input> inputs{input, lfo_in};

    Node node{func, inputs};
    std::cout << "Added tremolo" << std::endl;
    return node;    
}


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
    std::cout << "Added delay" << std::endl;
    return node;
}