// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 1

float t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle;
// Roll/pitch: milliradians (SetRl/SetPt, MAVLink x/y, RC mapped); yaw/throttle: normalized as before
float controlMode = NAN;
Vector gyro; // bias-corrected gyro after rotate — angular rate for PD-PI/KrenCtrl (rad/s if LSB→rad/s scale is correct)
Vector gyro_pid; // bias-corrected, raw MPU6050 LSB after rotate (debug / legacy)
float roll_acc_rad, pitch_acc_rad; // acc angle (rad), atan2 on filtered acc
Vector acc; // accelerometer data (sensor units / filtered pipeline)
Vector rates; // filtered angular rates (rad/s), updated in controlAttitude
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary
float motors[4]; // normalized motors thrust in range [0..1]
void setup() {
	Serial.begin(SERIAL_BAUDRATE);
	print("Initializing flix\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);

#if WIFI_ENABLED
	setupWiFi();
#endif
	setupIMU();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	const bool imuUpdated = readIMU();
	step();
	if (imuUpdated) {
		estimate();
		control();
		sendMotors();
		recordTakeLog();  // one high-rate sample per fresh IMU sample (~IMU data-ready rate)
	}
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	continueDumpTakeLog();  // non-blocking: send a few CSV lines per loop, control not blocked
}
