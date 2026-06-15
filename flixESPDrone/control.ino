// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
extern float useLPF;
extern float lpfAlpha;
#include "util.h"
#include "KrenCtrl.hpp"
#include <math.h>

#define YAWRATE_P 0.4f
#define YAWRATE_I 0.0f
#define YAWRATE_D 0.0f
#define YAWRATE_MAX (radians(10) * 1000.0)

enum NoFlg {
	NoRoll = 1, NoPitch, NoYaw,
};
const int MANUAL = 0, ACRO = 1, STAB = 2, AUTO = 3;
int mode = STAB;
bool armed = false;
#define PITCHRATE_MAX radians(180)
#define ROLLRATE_MAX radians(180)
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);

// Angle PID + rate PID (cascade) — SetPt/SetRl: millirad; pitch_acc_rad: rad; rates.y: rad/s.
PID rollPID(3.0f, 0.0f, 0.0f);
PID pitchPID(3.0f, 0.0f, 0.0f);
PID rollRatePID(0.2f, 0.02f, 0.01f, 0.4f, 0.1f);
// Bench pitch defaults; windup>0 required when Ki≠0 (see `PID::update` constrain on I term).
PID pitchRatePID(0.2f, 0.02f, 0.01f, 0.4f, 0.1f);

// 0 = PD-PI (KrenCtrl), 1 = cascade PID (acc angle rad + rates rad/s → torque).
float ctrlAlg = 0.0f;
// Pitch angle source (Flash "QuatEn"): 0 = pitch_acc_rad (accelerometer),
// 1 = attitude.getPitch() (quaternion, default).
float useQuaternion = 1.0f;
float manualFrontOff = 0.0f;
float manualRearOff  = 0.0f;
float smcLambda = 3.0f;   // λ — v_sp = λ·e (Flash SMCLm)
float smcK      = 0.1f;   // K in tanh(K·s/Φ) (Flash SMCK)
float smcPhi    = 3.0f;   // boundary layer Φ (Flash SMCPhi)
float smcKrate  = 1.0f;   // K_rate in s = v_sp − K_rate·ω (Flash SMCKrt)
float smcKi     = 0.0f;   // angle integral gain (Flash SMCKi)
float smcIMax   = 5.0f;   // integral state limit (Flash SMCImax)
float smcVMax   = 30.0f;  // |v_sp| limit, same units as λ·e (Flash SMCVmax)
float smcLpfAlpha = 0.1f; // output LPF α when SMCLpft=0 (Flash SMCLpfa)
float smcLpfTau   = 0.0f; // output LPF τ: α=dt/τ when τ>dt (Flash SMCLpft)

static constexpr float smcUMax = 1.0f;
static constexpr float smcUMin = -1.0f;

static float smcIntegral = 0.0f;
static LowPassFilter<float> smcOutLpf(0.1f);

void resetSmc() {
	smcIntegral = 0.0f;
	smcOutLpf.reset();
}

static float smcOutputLpfAlpha(float dt) {
	if (smcLpfTau > dt)
		return dt / smcLpfTau;
	return smcLpfAlpha;
}

/** SMC pitch torque (±1). Matches bench sim: u=Umax·tanh(K·s/Φ)+Ki∫e, LPF on u. */
static float computeSmcPitchTorque(float pitch_cmd, float pitch, float rate_y, float dt) {
	const float e = pitch_cmd - pitch;
	const float v_sp = constrain(smcLambda * e, -smcVMax, smcVMax);
	const float s = v_sp - smcKrate * rate_y;
	const float phi = (smcPhi > 1e-6f) ? smcPhi : 0.5f;

	float u = smcUMax * tanhf(smcK * s / phi);

	if (dt > 0.0f && dt < 0.1f) {
		smcIntegral += e * dt;
		smcIntegral = constrain(smcIntegral, -smcIMax, smcIMax);
	}

	u += smcKi * smcIntegral;
	u = constrain(u, smcUMin, smcUMax);

	if ((u >= 0.99f * smcUMax && e > 0.0f) || (u <= 0.99f * smcUMin && e < 0.0f))
		smcIntegral -= e * dt;

	smcOutLpf.alpha = smcOutputLpfAlpha(dt);
	return smcOutLpf.update(u);
}
Quaternion attitudeTarget;
Vector torqueTarget;
float thrustTarget;

KrenCtrl pdpiRoll;
KrenCtrl pdpiPitch;

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode, roll_H, pitch_H, dt;
extern float roll_acc_rad, pitch_acc_rad, pitch_comp_rad, pitch_kalman_rad;
extern Vector gyro_pid;

/** Pitch angle (rad) from Flash "QuatEn" + `LPF_En`:
 *  QuatEn=1 → quaternion; QuatEn=0 + LPF_En=3 → Kalman; LPF_En=2 → Complementary; else acc. */
float getPitchAngle() {
	if (useQuaternion >= 0.5f) return attitude.getPitch();
	if (useLPF >= 2.5f) return pitch_kalman_rad;
	if (useLPF >= 1.5f) return pitch_comp_rad;
	return pitch_acc_rad;
}

void control() {
	mode = STAB;
	controlAttitude();
	controlTorque();
}

void controlAttitude() {
	rollRatePID.lpf.alpha = lpfAlpha;
	pitchRatePID.lpf.alpha = lpfAlpha;

	static float prevCtrlAlg = -1.0f;
	if (fabsf(ctrlAlg - prevCtrlAlg) >= 0.25f) {
		prevCtrlAlg = ctrlAlg;
		pdpiRoll.reset();
		pdpiPitch.reset();
		rollPID.reset();
		pitchPID.reset();
		rollRatePID.reset();
		pitchRatePID.reset();
		resetSmc();
	}

	const bool useCascadePid = (ctrlAlg >= 0.5f && ctrlAlg < 1.5f);

	if (useCascadePid) {
		// SetRl/SetPt stored as millirad; convert to rad to match roll_acc_rad/pitch_acc_rad.
		const float rollSpRad = controlRoll * 0.001f;
		const float pitchSpRad = controlPitch * 0.001f;
		const float eRoll = rollSpRad - roll_acc_rad;
		const float pitchAngle = getPitchAngle();
		const float ePitch = pitchSpRad - pitchAngle;

		const float rateSpRoll = rollPID.update(eRoll);
		const float rateSpPitch = pitchPID.update(ePitch);
		float Rollrate_sp = constrain(rateSpRoll, -ROLLRATE_MAX, ROLLRATE_MAX);
		float Pitchrate_sp = constrain(rateSpPitch, -PITCHRATE_MAX, PITCHRATE_MAX);

		torqueTarget.x = rollRatePID.update(Rollrate_sp - rates.x);
		torqueTarget.y = pitchRatePID.update(Pitchrate_sp - rates.y);
	} else if (ctrlAlg >= 1.5f) {
		const float pitch = getPitchAngle();
		const float pitch_cmd = controlPitch * 0.001f;
		torqueTarget.y = computeSmcPitchTorque(pitch_cmd, pitch, rates.y, dt);
		torqueTarget.x = 0.0f;
	} else {
		torqueTarget.x = pdpiRoll.updateCtrl(dt, controlRoll, roll_H, rates.x) / 1000.0f;
		torqueTarget.y = pdpiPitch.updateCtrl(dt, controlPitch, pitch_H, rates.y) / 1000.0f;
	}

	float yawRateSp = controlYaw * YAWRATE_MAX;
	torqueTarget.z = yawRatePID.update(yawRateSp - rates.z);
}
// Default torque mixing: pitch attitude only (single-axis bench setup).
// bn*=1 disables that axis torque; bn*=0 applies controller torque on that axis.
uint8_t bnRll = 1, bnPtch = 0, bnYaw = 1, bManualMtr = 0;

void DisableCnl(int cnl, int val) {
	if (cnl == NoRoll) {
		bnRll = val;
		if (!val) {
			pdpiRoll.reset();
			pdpiPitch.reset();
			rollPID.reset();
			pitchPID.reset();
			rollRatePID.reset();
			pitchRatePID.reset();
			resetSmc();
		}
	} else if (cnl == NoPitch) {
		bnPtch = val;
		if (!val) {
			pdpiRoll.reset();
			pdpiPitch.reset();
			rollPID.reset();
			pitchPID.reset();
			rollRatePID.reset();
			pitchRatePID.reset();
			resetSmc();
		}
	} else if (cnl == NoYaw) {
		bnYaw = val;
	} else {
		bnRll = bnPtch = bnYaw = val;
		if (!val) {
			pdpiRoll.reset();
			pdpiPitch.reset();
			rollPID.reset();
			pitchPID.reset();
			rollRatePID.reset();
			pitchRatePID.reset();
			resetSmc();
		}
	}
	print("Channel %d set to %d - Controllers reset\n", cnl, val);
}

void controlTorque() {
	if (!torqueTarget.valid()) return;

	if (!armed) {
		torqueTarget = Vector(0.0f, 0.0f, 0.0f);
		memset(motors, 0, sizeof(motors));
		return;
	}

	if (bManualMtr) return;

	motors[MOTOR_FRONT_LEFT] = thrustTarget - (bnRll ? 0 : torqueTarget.x) - (bnPtch ? 0 : torqueTarget.y) + (bnYaw ? 0 : torqueTarget.z);
	motors[MOTOR_FRONT_RIGHT] = thrustTarget + (bnRll ? 0 : torqueTarget.x) - (bnPtch ? 0 : torqueTarget.y) - (bnYaw ? 0 : torqueTarget.z);
	motors[MOTOR_REAR_LEFT] = thrustTarget - (bnRll ? 0 : torqueTarget.x) + (bnPtch ? 0 : torqueTarget.y) - (bnYaw ? 0 : torqueTarget.z);
	motors[MOTOR_REAR_RIGHT] = thrustTarget + (bnRll ? 0 : torqueTarget.x) + (bnPtch ? 0 : torqueTarget.y) + (bnYaw ? 0 : torqueTarget.z);

	motors[MOTOR_FRONT_LEFT]  += manualFrontOff;
	motors[MOTOR_FRONT_RIGHT] += manualFrontOff;
	motors[MOTOR_REAR_LEFT]   += manualRearOff;
	motors[MOTOR_REAR_RIGHT]  += manualRearOff;

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}