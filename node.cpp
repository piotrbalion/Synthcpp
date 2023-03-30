#include "node.hpp"

Node::Node(std::function<float(std::span<float>)> func, std::vector<Input> inputs) {
    this->func = func;
    this->inputs = inputs;
}

void Node::compute(std::span<float> inputs) {
    this->output = this->func(inputs);
}