// flixESPDrone.ino — главный цикл прошивки ESP32.
//
// loop: readIMU → step → estimate → control → sendMotors → recordTakeLog
//       → handleInput → logData → continueDumpTakeLog

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#define SERIAL_BAUDRATE 115200
#define WIFI_ENABLED 1

float t = NAN; // время текущего шага, с
float dt;      // шаг dt с прошлого цикла, с
float controlRoll, controlPitch, controlYaw, controlThrottle;
// Roll/pitch: millirad (SetRl/SetPt); yaw/throttle — нормированные
float controlMode = NAN;
Vector gyro;       // угловая скорость для PD-PI (после rotate), рад/с
Vector gyro_pid;   // сырой gyro MPU6050 в LSB (отладка)
float roll_acc_rad, pitch_acc_rad; // угол по acc (рад)
Vector acc;        // акселерометр (после калибровки / ФНЧ)
Vector rates;      // угловые скорости (рад/с), ФНЧ в readIMU
Quaternion attitude; // оценка ориентации (Mahony + acc)
bool landed;       // на земле и неподвижен
float motors[4];   // тяга моторов [0..1]
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
		recordTakeLog();  // один отсчёт TakeLog на каждый свежий IMU
	}
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	continueDumpTakeLog();  // неблокирующая отправка CSV takelog
}
