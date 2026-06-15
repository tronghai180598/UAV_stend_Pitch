// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Implementation of command line interface

#include "vector.h"
#include "util.h"
#include "pid.h"
#include "KrenCtrl.hpp"


extern KrenCtrl pdpiRoll;
extern KrenCtrl pdpiPitch;
extern PID rollPID;
extern PID pitchPID;
extern PID rollRatePID;
extern PID pitchRatePID;

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern const int ACRO, STAB, AUTO;
extern float t, dt, loopRate;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern int mode, glog;
extern uint8_t pitchTelMask;
extern bool armed;
extern uint8_t bnRll, bnPtch, bnYaw;
extern float pitch_acc_rad;
extern float pitch_comp_rad;
extern float pitch_kalman_rad;
extern float useQuaternion;
extern float useLPF;
extern float getPitchAngle();
extern float manualFrontOff, manualRearOff;
extern Vector torqueTarget;

void startTakeLog();
void cancelTakeLogDump();

const char* motd =
"\nWelcome to\n"
" _______  __       __  ___   ___\n"
"|   ____||  |     |  | \\  \\ /  /\n"
"|  |__   |  |     |  |  \\  V  /\n"
"|   __|  |  |     |  |   >   <\n"
"|  |     |  `----.|  |  /  .  \\\n"
"|__|     |_______||__| /__/ \\__\\\n\n"
"Commands:\n\n"
"help - show help\n"
"p - show all parameters\n"
"p <name> - show parameter\n"
"p <name> <value> - set parameter\n"
"preset - reset parameters\n"
"time - show time info\n"
"ps - show pitch/roll/yaw\n"
"psq - show attitude quaternion\n"
"imu - show IMU data\n"
"arm - arm the drone\n"
"disarm - disarm the drone\n"
"stab/acro/auto - set mode\n"
"mot - show motor output\n"
"log - dump in-RAM log\n"
"cr - calibrate RC\n"
"ca - calibrate accel\n"
"mfr, mfl, mrr, mrl - test motor (remove props)\n"
"sys - show system info\n"
"reset - reset drone's state\n"
"reboot - reboot the drone\n";

void print(const char* format, ...) {
	char buf[1000];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	Serial.print(buf);
#if WIFI_ENABLED
	mavlinkPrint(buf);
#endif
}

void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		step();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
		delay(50);
	}
}

void doCommand(String str, bool echo = false) {
	// parse command
	String command, arg0, arg1;
	splitString(str, command, arg0, arg1);
	if (command.isEmpty()) return;

	// echo command
	if (echo) {
		print("> %s\n", str.c_str());
	}

	command.toLowerCase();

	// execute command
	if (command == "help" || command == "motd") {
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {
		print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {
			print("%s = %g\n", arg0.c_str(), arg1.toFloat());
		} else {
			print("Parameter not found: %s\n", arg0.c_str());
		}
	} else if (command == "preset") {
		resetParameters();
	} else if (command == "time") {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (command == "ps") {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (command == "imu") {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (command == "pit") {
		print("pitch_acc: %f  pitch_quat: %f  pitch_comp: %f  pitch_kal: %f  LPF_En=%g QuatEn=%g  active: %f\n",
			pitch_acc_rad, attitude.getPitch(), pitch_comp_rad, pitch_kalman_rad, useLPF, useQuaternion, getPitchAngle());

	}else if (command == "arm") {
		armed = true;
		pdpiRoll.reset();    // reset Roll controller on arm
		pdpiPitch.reset();   // reset Pitch controller on arm
		rollPID.reset();
		pitchPID.reset();
		rollRatePID.reset();
		pitchRatePID.reset();
		resetSmc();
		print("Drone armed - Controllers reset\n");
	} else if (command == "disarm") {
		armed = false;
		torqueTarget = Vector(0.0f, 0.0f, 0.0f);
		pdpiRoll.reset();    // reset on disarm
		pdpiPitch.reset();
		rollPID.reset();
		pitchPID.reset();
		rollRatePID.reset();
		pitchRatePID.reset();
		resetSmc();
		print("Drone disarmed - Controllers reset\n");
	} else if (command == "stab") {
		mode = STAB;
	} else if (command == "acro") {
		mode = ACRO;
	} else if (command == "auto") {
		mode = AUTO;
	} else if (command == "mot") {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "logtel" && arg0 != "") {
		int v = arg0.toInt();
		if (v >= 0 && v <= 7) {
			pitchTelMask = (uint8_t)v;
			if (pitchTelMask == 0)
				pitchTelMask = 7;
			print("pitchTelMask=%u\n", (unsigned)pitchTelMask);
		}
	} else if (command == "takelog") {
		startTakeLog();
	} else if (command == "log") {
		if (arg0 != "") {
			int v = arg0.toInt();
			if ((v < 0 ) || (v > 2)) v = 0;
			if (v != 0)
				cancelTakeLogDump();
			glog = v;
		}
		else dumpLog();
	} else if (command == "ca") {
		calibrateAccel();
	} else if (command == "spdpi") {
		pdpiRoll.setCtrlParam();
		pdpiPitch.setCtrlParam();
	} else if (command == "mfr") {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {
		testMotor(MOTOR_REAR_LEFT);
	} 
	else if (command == "mtr" && arg0 != "" && arg1 != "") {
		setMotor(arg0.toInt(), arg1.toInt());
	} 
	else if (command == "dscnl" && arg0 != "" && arg1 != "") {
		DisableCnl(arg0.toInt(), arg1.toInt());
	} else if (command == "sys") {
#ifdef ESP32
		print("Chip: %s\n", ESP.getChipModel());
		print("Temperature: %.1f °C\n", temperatureRead());
		print("Free heap: %d\n", ESP.getFreeHeap());
		// Print tasks table
		print("Num  Task                Stack  Prio  Core  CPU%%\n");
		int taskCount = uxTaskGetNumberOfTasks();
		TaskStatus_t *systemState = new TaskStatus_t[taskCount];
		uint32_t totalRunTime;
		uxTaskGetSystemState(systemState, taskCount, &totalRunTime);
		for (int i = 0; i < taskCount; i++) {
			String core = systemState[i].xCoreID == tskNO_AFFINITY ? "*" : String(systemState[i].xCoreID);
			int cpuPercentage = systemState[i].ulRunTimeCounter / (totalRunTime / 100);
			print("%-5d%-20s%-7d%-6d%-6s%d\n",systemState[i].xTaskNumber, systemState[i].pcTaskName,
				systemState[i].usStackHighWaterMark, systemState[i].uxCurrentPriority, core, cpuPercentage);
		}
		delete[] systemState;
#endif
	} else if (command == "save") {
		syncParameters();
	} else if (command == "reset") {
		attitude = Quaternion();
		ESP.restart();
	} else if (command == "mfront" && arg0 != "") {
		manualFrontOff = constrain(arg0.toInt(), -500, 500) / 1000.0f;
	} else if (command == "mrear" && arg0 != "") {
		manualRearOff = constrain(arg0.toInt(), -500, 500) / 1000.0f;
	} else {
		print("Invalid command: %s\n", command.c_str());
	}
}

void handleInput() {
	static bool showMotd = true;
	static String input;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			doCommand(input);
			input.clear();
		} else {
			input += c;
		}
	}
}
