// estimate.ino — оценка ориентации: кватернион Mahony + comp + Kalman pitch.
// pitch_comp_rad / pitch_kalman_rad используются при QuatEn=0 и LPF_En=2/3.

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
	// rates (рад/с) уже отфильтрованы в imu.ino
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	float accNorm = acc.norm();
	// acc в LSB MPU6050; на стенде коррекция даже при работающих моторах.
	landed = !motorsActive() && abs(accNorm - ACC_LSB_PER_G) < ACC_LSB_PER_G * 0.1f;
	if (accNorm < ACC_LSB_PER_G * 0.3f) return;  // < 0.3g — невалидный acc

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
	// Без кватерниона: комплементарный фильтр pitch
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
