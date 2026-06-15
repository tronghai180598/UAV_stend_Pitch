// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Attitude estimation from gyro and accelerometer

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "Kalman.h"

#define WEIGHT_ACC 0.003

extern float pitch_acc_rad;
extern float useQuaternion;
extern float useLPF;

float pitch_comp_rad = 0.0f;  // pitch from Complementary filter (rad)
float pitch_kalman_rad = 0.0f; // pitch from Kalman filter (rad)
float compAlpha = 0.99f;      // gyro weight, Flash key "CFAlpha" (0..1)
float kalQangle = 0.001f;     // Flash "KQang"
float kalQbias  = 0.003f;     // Flash "KQbias"
float kalRmeas  = 0.03f;      // Flash "KRmeas"
static bool compInit = false;
static bool kalInit = false;
static Kalman pitchKalman;

static void syncKalmanTuning() {
	pitchKalman.setQangle(kalQangle);
	pitchKalman.setQbias(kalQbias);
	pitchKalman.setRmeasure(kalRmeas);
}

static void resetPitchFiltersIfModeChanged() {
	static float prevQuatEn = -1.0f;
	static float prevLPFmode = -1.0f;
	if (useQuaternion == prevQuatEn && useLPF == prevLPFmode) return;
	prevQuatEn = useQuaternion;
	prevLPFmode = useLPF;
	compInit = false;
	kalInit = false;
}

void estimate() {
	if (!isfinite(dt) || dt <= 0.0f || dt > 0.5f) return;
	resetPitchFiltersIfModeChanged();
	applyGyro();
	applyAcc();
	applyComplementary();
	applyKalman();
}

void applyGyro() {
	// rates (rad/s) is already LPF-filtered by imu.ino — use directly
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	float accNorm = acc.norm();
	// acc is in raw LSB (ACC_LSB_PER_G ≈ 8192 for ±4g). Skip only if sensor invalid.
	// Bench 1-DOF: apply correction even when motors active (linear accel is small on bar).
	landed = !motorsActive() && abs(accNorm - ACC_LSB_PER_G) < ACC_LSB_PER_G * 0.1f;
	if (accNorm < ACC_LSB_PER_G * 0.3f) return;  // < 0.3g in LSB → invalid acc

	// calculate accelerometer gravity correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * WEIGHT_ACC;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

void applyComplementary() {
	if (!compInit) {
		pitch_comp_rad = pitch_acc_rad;
		compInit = true;
		return;
	}
	// No-Quaternion path: pitch_comp = alpha*(pitch_comp + rates.y*dt) + (1-alpha)*pitch_acc_rad
	pitch_comp_rad = compAlpha * (pitch_comp_rad + rates.y * dt)
	               + (1.0f - compAlpha) * pitch_acc_rad;
}

void applyKalman() {
	syncKalmanTuning();
	if (!kalInit) {
		pitchKalman.setAngle(pitch_acc_rad);
		pitch_kalman_rad = pitch_acc_rad;
		kalInit = true;
		return;
	}
	pitch_kalman_rad = pitchKalman.getAngle(pitch_acc_rad, rates.y, dt);
}
