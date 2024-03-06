/*###########################################################
 # Copyright (c) 2023-2024. BNU-HKBU UIC RoboMaster         #
 #                                                          #
 # This program is free software: you can redistribute it   #
 # and/or modify it under the terms of the GNU General      #
 # Public License as published by the Free Software         #
 # Foundation, either version 3 of the License, or (at      #
 # your option) any later version.                          #
 #                                                          #
 # This program is distributed in the hope that it will be  #
 # useful, but WITHOUT ANY WARRANTY; without even           #
 # the implied warranty of MERCHANTABILITY or FITNESS       #
 # FOR A PARTICULAR PURPOSE.  See the GNU General           #
 # Public License for more details.                         #
 #                                                          #
 # You should have received a copy of the GNU General       #
 # Public License along with this program.  If not, see     #
 # <https://www.gnu.org/licenses/>.                         #
 ###########################################################*/

#include "utils.h"

template <typename T>
EdgeDetector<T>::EdgeDetector(T initial) {
    prev_ = initial;
}

template <typename T>
void EdgeDetector<T>::input(T signal) {
    posEdge_ = false;
    negEdge_ = false;
    if (prev_ < signal)
        posEdge_ = true;
    else if (prev_ > signal)
        negEdge_ = true;
    prev_ = signal;
}

template <typename T>
bool EdgeDetector<T>::edge() {
    return posEdge_ || negEdge_;
}

template <typename T>
bool EdgeDetector<T>::posEdge() {
    return posEdge_;
}

template <typename T>
bool EdgeDetector<T>::negEdge() {
    return negEdge_;
}

BoolEdgeDetector::BoolEdgeDetector(bool initial) {
    prev_ = initial;
}

void BoolEdgeDetector::input(bool signal) {
    posEdge_ = !prev_ && signal;
    negEdge_ = prev_ && !signal;
    prev_ = signal;
}

bool BoolEdgeDetector::edge() {
    return posEdge_ || negEdge_;
}

bool BoolEdgeDetector::posEdge() {
    return posEdge_;
}

bool BoolEdgeDetector::negEdge() {
    return negEdge_;
}

bool BoolEdgeDetector::get() {
    return prev_;
}

FloatEdgeDetector::FloatEdgeDetector(float initial, float threshold) {
    prev_ = initial;
    threshold_ = threshold;
}

void FloatEdgeDetector::input(float signal) {
    float diff = signal - prev_;
    posEdge_ = diff > threshold_;
    negEdge_ = diff < -threshold_;
    prev_ = signal;
}

bool FloatEdgeDetector::edge() {
    return posEdge_ || negEdge_;
}

bool FloatEdgeDetector::posEdge() {
    return posEdge_;
}

bool FloatEdgeDetector::negEdge() {
    return negEdge_;
}

RampSource::RampSource(float initial, float min, float max, float step) {
    input_ = initial;
    output_ = initial;
    min_ = min;
    max_ = max;
    step_ = step;
}

float RampSource::Calc(float input) {
    input_ = input;
    float sub_output = output_ - step_ * input_;
    float add_output = output_ + step_ * input_;
    if ((output_ > max_ && sub_output < output_) || (output_ < min_ && add_output > output_)) {
        output_ = sub_output;
        return output_;
    }
    output_ = add_output;
    output_ = min(output_, max_);
    output_ = max(output_, min_);
    return output_;
}

float RampSource::Get() {
    return output_;
}

float RampSource::GetMax() {
    return max_;
}

float RampSource::GetMin() {
    return min_;
}

void RampSource::SetMax(float max) {
    max_ = max;
}

void RampSource::SetMin(float min) {
    min_ = min;
}

void RampSource::SetCurrent(float current) {
    output_ = current;
}