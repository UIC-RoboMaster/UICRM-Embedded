#include "model.h"

#include <math.h>

#include "weight.h"

CCMRAM float w1[DENSE1_SIZE];
CCMRAM float w3(0.0f);
CCMRAM float w2(0.0f);

inline float relu(float x) {
    return fmaxf(0.0f, x);
}

float predict(float x1, float x2) {
    // The activations of the first layer are small enough to store
    // on the stack (16 floats = 64 bytes).

    // First dense layer. Since there are two input neurons, we need
    // to perform a dot product with the first row of the W1 matrix.
    for (int i = 0; i < DENSE1_SIZE; ++i) {
        w1[i] = relu(x1 * W1(i, 0) + x2 * W1(i, 1) + b1(i));
    }

    // Second dense layer.
    for (int relu_x2 = 0; relu_x2 < DENSE2_SIZE; ++relu_x2) {
        // Perform a dot product of the incoming activation vector with each
        // row of the W2 matrix.
        for (int relu_y2 = 0; relu_y2 < DENSE1_SIZE; ++relu_y2) {
            w2 += w1[relu_y2] * W2(relu_x2, relu_y2);
        }
        w2 = relu(w2 + b2(relu_x2));

        // We don't actually need to store the activations of the second layer.
        // Since the last layer only has one neuron, we can immediately compute
        // how much each activation contributes to the final layer.
        w3 += w2 * W3(relu_x2);
    }

    // Final dense layer.
    return w3 + b3();
}