// parameters.ino — хранение параметров во Flash (namespace «flix», команда save).
// Таблица parameters[] связывает имя CLI (p <name>) с переменной в RAM.

#include <Preferences.h>
#include "util.h"
#include "pid.h"
#include "KrenCtrl.hpp"

extern float mavDisable, controlPitch, controlRoll;
extern KrenCtrl pdpiRoll;
extern KrenCtrl pdpiPitch;
extern PID rollPID;
extern PID pitchPID;
extern PID rollRatePID;
extern PID pitchRatePID;
extern float ctrlAlg;
extern float kGr;
extern float useQuaternion;
extern float useLPF;
extern float lpfAlpha;
extern float compAlpha;
extern float kalQangle, kalQbias, kalRmeas;
extern float smcLambda, smcK, smcPhi, smcKrate, smcKi, smcIMax;
extern float smcVMax, smcLpfAlpha, smcLpfTau;
Preferences storage;

struct Parameter {
	const char *name; // макс. 16 символов (ограничение Preferences)
	float *variable;
	float value; // кэш при чтении из Flash
};

Parameter parameters[] = {
	// control
	{"RTV", &pdpiRoll.mTv},
	{"RTM", &pdpiRoll.mTm},
	{"RTE", &pdpiRoll.mTe},
	{"RKPF", &pdpiRoll.Kpf},
	{"RKPV", &pdpiRoll.Kpv},
	{"RKDV", &pdpiRoll.Kdv},
	{"SetRl", &controlRoll},
  {"rKlmf", &pdpiRoll.klmf},
  {"rKlmv", &pdpiRoll.klmv},
	{"RTI", &pdpiRoll.mTi},
	{"RTmu", &pdpiRoll.mTmu},


	{"PTV", &pdpiPitch.mTv},
	{"PTM", &pdpiPitch.mTm},
	{"PTE", &pdpiPitch.mTe},
	{"PKPF", &pdpiPitch.Kpf},
	{"PKPV", &pdpiPitch.Kpv},
	{"PKDV", &pdpiPitch.Kdv},
	{"SetPt", &controlPitch},
  {"pKlmf", &pdpiPitch.klmf},
  {"pKlmv", &pdpiPitch.klmv},
	{"PTI", &pdpiPitch.mTi},
	{"PTmu", &pdpiPitch.mTmu},

	{"ROLL_P", &rollPID.p},
	{"ROLL_I", &rollPID.i},
	{"ROLL_D", &rollPID.d},
	{"PITCH_P", &pitchPID.p},
	{"PITCH_I", &pitchPID.i},
	{"PITCH_D", &pitchPID.d},

	{"CtrlAlg", &ctrlAlg},
	{"RRateP", &rollRatePID.p},
	{"RRateI", &rollRatePID.i},
	{"RRateD", &rollRatePID.d},
	{"PRateP", &pitchRatePID.p},
	{"PRateI", &pitchRatePID.i},
	{"PRateD", &pitchRatePID.d},

  {"MAV_Disable", &mavDisable},
  {"kGyro", &kGr},
  {"QuatEn", &useQuaternion},
  {"LPF_En", &useLPF},
  {"LPFAlp", &lpfAlpha},
  {"CFAlpha", &compAlpha},
  {"KQang", &kalQangle},
  {"KQbias", &kalQbias},
  {"KRmeas", &kalRmeas},
  {"SMCLm",  &smcLambda},
  {"SMCK",   &smcK},
  {"SMCPhi", &smcPhi},
  {"SMCKrt", &smcKrate},
  {"SMCKi",  &smcKi},
  {"SMCImax",&smcIMax},
  {"SMCVmax",&smcVMax},
  {"SMCLpfa",&smcLpfAlpha},
  {"SMCLpft",&smcLpfTau},
};

void setupParameters() {
	storage.begin("flix", false);
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, *parameter.variable);
		}
		*parameter.variable = storage.getFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return *parameters[index].variable;
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			return *parameter.variable;
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcmp(parameter.name, name) == 0) {
			*parameter.variable = value;
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.value == *parameter.variable) continue;
		if (isnan(parameter.value) && isnan(*parameter.variable)) continue; // handle NAN != NAN
		storage.putFloat(parameter.name, *parameter.variable);
		parameter.value = *parameter.variable;
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, *parameter.variable);
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}
