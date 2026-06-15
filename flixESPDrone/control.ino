// control.ino — контур управления стендом (pitch / roll / yaw).
//
// CtrlAlg: 0 = PD-PI (KrenCtrl), 1 = каскадный PID, 2 = SMC по pitch.
// getPitchAngle() — единый источник угла pitch для PID/SMC/логов (QuatEn + LPF_En).
// bnRll/bnPtch/bnYaw: 1 = момент по оси отключён (dscnl), 0 = от регулятора.

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

// Каскадный PID: внешний контур угол → внутренний ω → момент (±1).
// SetPt/SetRl в millirad; feedback pitch — getPitchAngle(); roll — roll_acc_rad.
PID rollPID(3.0f, 0.0f, 0.0f);
PID pitchPID(3.0f, 0.0f, 0.0f);
PID rollRatePID(0.2f, 0.02f, 0.01f, 0.4f, 0.1f);
// Для pitch на стенде; windup>0 нужен при Ki≠0 (см. PID::update).
PID pitchRatePID(0.2f, 0.02f, 0.01f, 0.4f, 0.1f);

// 0 = PD-PI, 1 = каскадный PID, 2 = SMC (Flash «CtrlAlg»).
float ctrlAlg = 0.0f;
// Источник pitch: 1 = кватернион Mahony, 0 = acc/comp/kalman (Flash «QuatEn»).
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

/** Момент SMC по pitch (±1): u = Umax·tanh(K·s/Φ) + Ki∫e, затем ФНЧ. */
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

/** Угол pitch (рад) для регулятора и телеметрии по QuatEn и LPF_En. */
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
		// SetRl/SetPt в millirad → рад для сравнения с углом.
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
// Смешивание моментов на 4 мотора. По умолчанию стенда: только pitch (bnPtch=0).
// bn*=1 — момент по оси не применяется; bn*=0 — от регулятора.
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