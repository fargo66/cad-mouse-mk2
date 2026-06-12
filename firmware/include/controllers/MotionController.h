#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;

 private:
  static float clampf(float v, float lo, float hi);
  //static float hardZero(float v, float thr);
  //static float lowpass(float prev, float x, float dt, float tau);
  static float axisBaseDead(int i);
  //float filt_[6] = {};
 static float sensitivityCurve(float value, float dead, float limit);

  // Per-axis Kalman filter state
  float kalmanX_[6] = {};  // Estimated state
  float kalmanP_[6] = {};  // Estimate uncertainty (covariance)
  float kalmanStep(int axis, float measurement);
  bool motionActive_ = false;
};
