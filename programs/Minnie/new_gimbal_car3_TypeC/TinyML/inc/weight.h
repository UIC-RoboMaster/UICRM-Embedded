// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

#ifndef WEIGHT_H
#define WEIGHT_H

#define CCMRAM __attribute__((section(".ccmram")))

CCMRAM extern const float W_relu[];
CCMRAM extern const float b_relu[];
CCMRAM extern const float W_relu2[];
CCMRAM extern const float b_relu2[];
CCMRAM extern const float W_output[];
CCMRAM extern const float b_output[];

const int DENSE1_SIZE = 16;
const int DENSE2_SIZE = 16;

inline float W1(int relu_x, int relu_y) {
    return *(W_relu + relu_y * DENSE1_SIZE + relu_x);
}

inline float b1(int i) {
    return *(b_relu + i);
}

inline float W2(int relu_x, int relu_y) {
    return *(W_relu2 + relu_y * DENSE1_SIZE + relu_x);
}

inline float b2(int relu_x) {
    return *(b_relu2 + relu_x);
}

inline float W3(int relu_x) {
    return *(W_output + relu_x);
}

inline float b3() {
    return *b_output;
}

#endif  // WEIGHT_H
