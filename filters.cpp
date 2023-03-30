#include <cmath>
#include "filters.hpp"

#define SAMPLING_FREQUENCY  44100


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
    std::cout << "Added lowpass" << std::endl;
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
    std::cout << "Added highpass" << std::endl;
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
    std::cout << "Added bandpass" << std::endl;
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
    std::cout << "Added notch" << std::endl;
    return node;
}

Node phaser(const Input input) {  // BW in octaves
    Input lfo_in;
    lfo_in.type = LFO_IN;
    float f0 = 10000;
    float BW = 1;
    float x1 = 0.0; // sample buffers
    float x2 = 0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float f0_modulated = f0 + sig[1] * 5000;
        float omega = 2 * M_PI * f0_modulated / SAMPLING_FREQUENCY;   // used constants
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
    std::cout << "Added phaser" << std::endl;
    return node;
}

Node wah(const Input input) {  // template and low pass test
    Input lfo_in;
    lfo_in.type = LFO_IN;
    const float f0 = 10000;
    const float Q = 0.2;
    float x1 = 0.0; // sample buffers
    float x2 = 0.0; 
    float y1 = 0.0;
    float y2 = 0.0;

    auto func = [=] (std::span<float> sig) mutable -> float {
        float f0_modulated = f0 + sig[1] * 5000;
        float omega = 2 * M_PI * f0_modulated / SAMPLING_FREQUENCY;   // used constants
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
    std::cout << "Added wah" << std::endl;
    return node;
}
