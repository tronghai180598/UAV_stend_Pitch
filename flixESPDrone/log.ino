// log.ino — телеметрия по UART и кольцевой буфер TakeLog (~6000 отсчётов).
//
// log 1/2: до 50 Гц, маска pitchTelMask (θ milli-rad, ω decideg/s, момент×1000).
// takelog: неблокирующий дамп CSV → TAKELOG_START … TAKELOG_END.

#include "vector.h"
#include "util.h"
#include <math.h>

#define LOG_RATE 100
#define LOG_DURATION 5
#define LOG_SIZE LOG_DURATION * LOG_RATE

// ---- Кольцевой буфер Take Log ----
// int16×3×6000 ≈ 36 КБ; ~6 с при частоте IMU ~1 кГц
// pitch: ×10000 (рад), gyro: decideg/s, torque: ×10000 (норм. ±1)
#define TAKELOG_SIZE 6000

struct TakeLogEntry {
	int16_t pitch_e4;    // attitude.getPitch() × 10000       (rad, res 0.0001)
	int16_t gyro_dDegS;  // gyroBc.y × (180/π) × 10    (decideg/s, res 0.1°/s, lossless ±2000dps)
	int16_t torque_e4;   // torqueTarget.y × 10000      (normalized ±1)
};

static TakeLogEntry takeLogBuf[TAKELOG_SIZE];
static int          takeLogPtr  = 0;
static bool         takeLogFull = false;

/** Частота строк телеметрии на UART (Гц). 115200: ~50 Гц для коротких строк. */
#define SERIAL_TELEMETRY_HZ 50.0f

extern Vector gyroBc;
extern Vector rates;
extern float t;
extern float useLPF;
extern Vector torqueTarget;
extern float getPitchAngle();

struct LogEntry {
	const char *name;
	float *value;
};

LogEntry logEntries[] = {
	{"t", &t},
	{"rates.x", &rates.x},
	{"rates.y", &rates.y},
};

const int logColumns = sizeof(logEntries) / sizeof(logEntries[0]);
float logBuffer[LOG_SIZE][logColumns];

int glog = 0;

/** Битовая маска каналов pitch в log 1: bit0=θ, bit1=ω, bit2=момент. */
uint8_t pitchTelMask = 7;

static constexpr float kLivePitchScale = 1000.0f;            // rad → milli-rad
static constexpr float kLiveGyroScale  = float(180.0/M_PI) * 10.0f;  // rad/s → deci-deg/s

static Rate serialTelemetryRate(SERIAL_TELEMETRY_HZ);

static void printPitchTelemetryLine() {
	if (pitchTelMask == 0)
		return;
	bool first = true;
	auto sep = [&]() {
		if (!first)
			print(" ");
		first = false;
	};
	if (pitchTelMask & 1u) {
		sep();
		float pitchLog = getPitchAngle();
		print("%d", int(lrintf(pitchLog * kLivePitchScale)));
	}
	if (pitchTelMask & 2u) {
		sep();
		float gyroLog = (useLPF >= 0.5f) ? rates.y : gyroBc.y;
		print("%d", int(lrintf(gyroLog * kLiveGyroScale)));
	}
	if (pitchTelMask & 4u) {
		sep();
		print("%d", int(lrintf(torqueTarget.y * 1000.0f)));
	}
	print("\n");
}

static inline int16_t clamp16(float v) {
	int iv = (int)lrintf(v);
	if (iv >  32767) iv =  32767;
	if (iv < -32768) iv = -32768;
	return (int16_t)iv;
}

static constexpr float kGyroDegS = float(180.0 / M_PI) * 10.0f;  // rad/s → decideg/s

// --- Non-blocking TakeLog dump state ---
static bool s_dumping   = false;
static int  s_dumpIdx   = 0;
static int  s_dumpCount = 0;
static int  s_dumpStart = 0;
static int  s_savedGlog = 0;

void recordTakeLog() {
	if (s_dumping) return;  // freeze buffer while dumping
	float pitchTkLog = getPitchAngle();
	takeLogBuf[takeLogPtr].pitch_e4   = clamp16(pitchTkLog * 10000.0f);
	takeLogBuf[takeLogPtr].gyro_dDegS = clamp16(gyroBc.y * kGyroDegS);    // before LPF, noisy
	takeLogBuf[takeLogPtr].torque_e4  = clamp16(torqueTarget.y * 10000.0f);
	takeLogPtr = (takeLogPtr + 1) % TAKELOG_SIZE;
	if (takeLogPtr == 0) takeLogFull = true;
}

/** Called from CLI on "takelog" — init only, non-blocking. */
void startTakeLog() {
	if (s_dumping) return;
	s_savedGlog = glog;
	glog        = 0;   // disable telemetry during dump to avoid mixed data
	s_dumpCount = takeLogFull ? TAKELOG_SIZE : takeLogPtr;
	s_dumpStart = takeLogFull ? takeLogPtr   : 0;
	s_dumpIdx   = 0;
	s_dumping   = true;
	if (s_dumpCount == 0)
		print("TakeLog: buffer empty — run ~6 s (IMU logging) before takelog\n");
	print("TAKELOG_START %d\n", s_dumpCount);
}

/** Called each loop() — send a few lines if UART has space; does not block control. */
void continueDumpTakeLog() {
	if (!s_dumping) return;
	if (s_dumpIdx >= s_dumpCount) {
		print("TAKELOG_END\n");
		glog      = s_savedGlog;
		s_dumping = false;
		return;
	}
	const int MAX_PER_CALL = 16;
	for (int n = 0; n < MAX_PER_CALL && s_dumpIdx < s_dumpCount; n++) {
		if (Serial.availableForWrite() < 24) break;  // avoid blocking UART
		const int idx = (s_dumpStart + s_dumpIdx) % TAKELOG_SIZE;
		const TakeLogEntry &e = takeLogBuf[idx];
		print("%d,%d,%d\n", e.pitch_e4, e.gyro_dDegS, e.torque_e4);
		s_dumpIdx++;
	}
}

/** Abort an in-progress TakeLog dump (restores glog). */
void cancelTakeLogDump() {
	if (!s_dumping)
		return;
	s_dumping = false;
	glog = s_savedGlog;
}

void logData() {
	if (glog == 0) {
        return;
    }
	/* Log 1 & 2: same content; rate-limited (not every loop → avoid UART congestion). */
	if (glog == 1 || glog == 2) {
		if (!serialTelemetryRate)
			return;
		printPitchTelemetryLine();
		return;
	}
}

void dumpLog() {
	// Print header
	for (int i = 0; i < logColumns; i++) {
		print("%s%s", logEntries[i].name, i < logColumns - 1 ? "," : "\n");
	}
	// Print data
	for (int i = 0; i < LOG_SIZE; i++) {
		if (logBuffer[i][0] == 0) continue; // skip empty records
		for (int j = 0; j < logColumns; j++) {
			print("%g%s", logBuffer[i][j], j < logColumns - 1 ? "," : "\n");
		}
	}
}
