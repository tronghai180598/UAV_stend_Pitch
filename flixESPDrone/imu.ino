// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Work with the IMU sensor

#include "lpf.h"
#include "util.h"
#include <math.h>

#include <MPU6050.h>
MPU6050 imu(Wire);

Vector accBias(0.0, 0.0, 0.0);
Vector accScale(1.0, 1.0, 1.0);
constexpr float GYRO_LSB_PER_DPS = 16.4f; // MPU6050 at +-2000 dps
constexpr float GYRO_DPS_TO_RAD = (float)M_PI / 180.0f;
// Flash "LPFAlp" — shared by acc/rates LPF (imu) and PID D-term LPF (rate PID)
float lpfAlpha = 0.2f;
LowPassFilter<Vector> accFilter(0.2f);
LowPassFilter<Vector> ratesFilter(0.2f);
Vector gyroBias;
Vector gyroRawBias;
Vector gyroBc;   // bias-corrected gyro (rad/s), before ratesFilter — for noisy logging
float useLPF = 1.0f;

float roll_H, pitch_H;

static inline float approx_atan2_quadrant(float y, float z) {
	const float kEPS = 1e-6f;
	float ay = fabsf(y);
	float az = fabsf(z);
	if (ay < kEPS && az < kEPS) { return 0.0f; }

	float beta;
	if (ay <= az)  beta = M_PI_4 * (ay / az);
	else  beta = M_PI_4 * (2.0f - az / ay);

	float th;
	if (z >= 0.0f) th = (y >= 0.0f) ? beta : -beta;
	else th = (y >= 0.0f) ? (M_PI - beta) : (-(M_PI) + beta);
	return th;
}

void update_attitude_from_acc(Vector &acc) {
	const float rol = approx_atan2_quadrant(acc.y, acc.z);
	const float pit = approx_atan2_quadrant(-acc.x, acc.z);
	roll_acc_rad = rol;
	pitch_acc_rad = pit;
	roll_H = rol * 1000.0f;
	pitch_H = pit * 1000.0f;
}

void setupIMU() {
	print("Setup IMU\n");
	Wire.begin();
	imu.begin();
	configureIMU();
	calibrateGyroOnce();
}

void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_50HZ_APPROX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
}

float kGr = 1.0f;
#define to_mRad (1.065264436f)

bool readIMU() {
	if (!imu.getIntDataReadyStatus()) return false;
	int16_t ax, ay, az, gx, gy, gz;
	imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	Vector lacc = Vector(ax, ay, az);
	lacc = (lacc - accBias) / accScale;
	rotateIMU(lacc);
	accFilter.alpha = lpfAlpha;
	ratesFilter.alpha = lpfAlpha;
	if (useLPF >= 0.5f) {
		acc = accFilter.update(lacc);
	} else {
		acc = lacc;
	}
	Vector rawGyro = Vector(gx, gy, gz);
	rotateIMU(rawGyro);
	gyro_pid = rawGyro - gyroRawBias;  // raw bias-corrected LSB for noise logging
	Vector lgyro = rawGyro / GYRO_LSB_PER_DPS * GYRO_DPS_TO_RAD;
	gyroBc = lgyro - gyroBias;          // global: bias-corrected, before LPF — noisy
	if (useLPF >= 0.5f) {
		rates = ratesFilter.update(gyroBc);
	} else {
		rates = gyroBc;
	}
	gyro = rates * (to_mRad * kGr);

	update_attitude_from_acc(acc);
	return true;
}

void rotateIMU(Vector& data) {
	data = Vector(data.x, data.y, data.z);
}

void calibrateAccel() {}
void calibrateGyroOnce() {
	Vector rawGyro;
	Vector gyroRad;
	int i = 100;
	while (!imu.getIntDataReadyStatus()) {
		i--;
		if (i == 0) return;
		delay(1);
	}

	int16_t ax, ay, az, gx, gy, gz;
	imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	rawGyro = Vector(gx, gy, gz);
	rotateIMU(rawGyro);
	gyroRawBias = rawGyro;
	gyroRad = rawGyro / GYRO_LSB_PER_DPS * GYRO_DPS_TO_RAD;
	gyroBias = gyroRad;
	for (i = 0; i < 100; i++) {
		if (imu.getIntDataReadyStatus()) {
			imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			rawGyro = Vector(gx, gy, gz);
			rotateIMU(rawGyro);
			gyroRawBias += (rawGyro - gyroRawBias) / 50.0f;
			gyroRad = rawGyro / GYRO_LSB_PER_DPS * GYRO_DPS_TO_RAD;
			gyroBias += (gyroRad - gyroBias) / 50.0f;
		}
		delay(1);
	}
	for (int j = 0; j < 1000; j++) {
		if (imu.getIntDataReadyStatus()) {
			imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			rawGyro = Vector(gx, gy, gz);
			rotateIMU(rawGyro);
			gyroRawBias += (rawGyro - gyroRawBias) / 500.0f;
			gyroRad = rawGyro / GYRO_LSB_PER_DPS * GYRO_DPS_TO_RAD;
			gyroBias += (gyroRad - gyroBias) / 500.0f;
		}
		delay(1);
	}
}

void printIMUCalibration() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");
	print("model: %s\n", imu.getModel());
	print("who am I: 0x%02X\n", imu.whoAmI());
	print("rate: %.0f\n", loopRate);
	print("gyro: %f %f %f\n", gyro.x, gyro.y, gyro.z);
	print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
	print("roll pitch: %f %f\n", roll_H, pitch_H);
	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}