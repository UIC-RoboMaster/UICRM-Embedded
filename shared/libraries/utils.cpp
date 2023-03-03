#include "utils.h"

BoolEdgeDetector::BoolEdgeDetector(bool initial) { prev_ = initial; }

void BoolEdgeDetector::input(bool signal) {
  posEdge_ = false;
  negEdge_ = false;
  if (!prev_ && signal)
    posEdge_ = true;
  else if (prev_ && !signal)
    negEdge_ = true;
  prev_ = signal;
}

bool BoolEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool BoolEdgeDetector::posEdge() { return posEdge_; }

bool BoolEdgeDetector::negEdge() { return negEdge_; }

FloatEdgeDetector::FloatEdgeDetector(float initial, float threshold) {
  prev_ = initial;
  threshold_ = threshold;
}

void FloatEdgeDetector::input(float signal) {
  posEdge_ = false;
  negEdge_ = false;
  float diff = signal - prev_;
  if (diff > threshold_)
    posEdge_ = true;
  else if (diff < -threshold_)
    negEdge_ = true;
  prev_ = signal;
}

bool FloatEdgeDetector::edge() { return posEdge_ || negEdge_; }

bool FloatEdgeDetector::posEdge() { return posEdge_; }

bool FloatEdgeDetector::negEdge() { return negEdge_; }

RampSource::RampSource(float initial, float min, float max, float step) {
  input_ = initial;
  output_ = initial;
  min_ = min;
  max_ = max;
  step_ = step;
}

float RampSource::Calc(float input) {
  input_ = input;
  output_ += step_ * input_;
  output_ = min(output_, max_);
  output_ = max(output_, min_);
  return output_;
}

float RampSource::Get() { return output_; }

float RampSource::GetMax() { return max_; }

float RampSource::GetMin() { return min_; }


