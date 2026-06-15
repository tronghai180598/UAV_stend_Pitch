// pid.h — каскадный PID (внешний угол → внутренний rate).
// D-член фильтруется ФНЧ при LPF_En ≥ 0.5 (lpfAlpha из imu.ino).

#pragma once
#include "lpf.h"
extern float useLPF;

class PID {
public:
	float p, i, d;
	float windup;
	float dtMax;

	float derivative = 0;
	float integral = 0;

	LowPassFilter<float> lpf; // ФНЧ для D-составляющей

	PID(float p, float i, float d, float windup = 0, float dAlpha = 1, float dtMax = 0.1) :
		p(p), i(i), d(d), windup(windup), lpf(dAlpha), dtMax(dtMax) {}

	float update(float error) {
		float dt = t - prevTime;

		if (dt > 0 && dt < dtMax) {
			integral += error * dt;
			float rawDeriv = (error - prevError) / dt;
			derivative = (useLPF >= 0.5f) ? lpf.update(rawDeriv) : rawDeriv;
		} else {
			integral = 0;
			derivative = 0;
		}

		prevError = error;
		prevTime = t;

		return p * error + constrain(i * integral, -windup, windup) + d * derivative; // выход PID
	}

	void reset() {
		prevError = NAN;
		prevTime = NAN;
		integral = 0;
		derivative = 0;
		lpf.reset();
	}

private:
	float prevError = NAN;
	float prevTime = NAN;
};
