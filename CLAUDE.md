# CLAUDE.md — UAV Lon ESP8266

> Tài liệu nội bộ cho AI/dev. Hướng dẫn thao tác chi tiết cho sinh viên: `INSTRUKCIYA_STUDENT_RU.md`.

## 1. Tổng quan

UAV quad **treo trên thanh cố định** (bench 1 trục — chủ yếu **pitch**). PC chạy **Qt5 ground station**; firmware trên **ESP32**; **ESP8266** relay USB serial.

```
Qt (PC) ── /dev/ttyUSB0 115200 ──► ESP8266 ── UART ──► ESP32 ── PWM ──► 4 motor
                                         └── WiFi AP: flix / flixwifi, MAVLink UDP 14550
```

**Mặc định bench:** chỉ kênh pitch bật (`dscnl`), roll setpoint = 0 (CLI `SetRl`), tab Qt mở **Classic PID**.

---

## 2. Cấu trúc repo

```
UAV_Lon_ESP8266/
├── flixESPDrone/          # Firmware ESP32 (Arduino)
│   ├── flixESPDrone.ino   # setup/loop
│   ├── control.ino        # STAB → controlAttitude → controlTorque → motors
│   ├── imu.ino            # MPU6050 → acc, rates, roll_H/pitch_H
│   ├── estimate.ino       # Complementary + Kalman pitch
│   ├── log.ino            # Telemetry serial + TakeLog buffer
│   ├── cli.ino            # Lệnh text từ Qt
│   ├── parameters.ino     # Flash (Preferences namespace "flix")
│   ├── KrenCtrl.*         # PD-PI tầng (CtrlAlg=0)
│   ├── pid.h              # Cascade PID + yaw rate
│   ├── Kalman.*           # Lọc Kalman pitch
│   └── motors/wifi/mavlink/...
├── Qt_file/               # Ground station Qt5 → binary ./Qt_Flix
│   ├── dialog.cpp/.h/.ui
│   └── Qt_Flix.pro
└── log/                   # CSV TakeLog thí nghiệm
```

---

## 3. Phần cứng

| Thành phần | Chi tiết |
|---|---|
| UAV MCU | ESP32 Dev Module |
| PC bridge | ESP8266 USB → UART tới ESP32 |
| IMU | MPU6050 (gyro ±2000°/s, acc ±4G) |
| Motor | 4× DC brushed, PWM 100 Hz 12-bit |
| Motor pins | RL=32, RR=33, FR=26, FL=27 |
| Serial PC | `/dev/ttyUSB0` @ 115200 (hardcoded Qt, đổi nếu cần) |

---

## 4. Vòng lặp firmware

```
readIMU → step(dt) → control() → sendMotors() → handleInput() → processMavlink()
→ logData() [nếu glog≠0] → continueDumpTakeLog()
```

`controlAttitude()` chọn thuật toán theo **`CtrlAlg`**. Yaw luôn **`yawRatePID`** + `rates.z`.

**Reset bộ điều khi:** `arm` / `disarm`, đổi `CtrlAlg`, `dscnl` bật kênh (val=0) — reset PD-PI + 4 PID cascade + SMC.

---

## 5. Thuật toán điều khiển (`CtrlAlg`)

| `CtrlAlg` | Chế độ | Output pitch |
|---|---|---|
| **0** | PD-PI (`KrenCtrl`, `_slCtrl_2PD`) | `pdpiPitch` → `torqueTarget.y` (scale ÷1000) |
| **1** | Cascade PID | θ-loop → ω-sp → rate PID → `torqueTarget.y` (±1) |
| **2** | SMC pitch | `e = cmd − θ`, `s = λe − K_rate·ω`, `u = K·tanh(s/Φ) + Ki∫e` → LPF → ±1 |

Chỉnh **`CtrlAlg` khi disarm**; `save` để giữ Flash.

### 5.1 PD-PI (`CtrlAlg=0`)

Vòng ngoài góc → vòng trong rate → lọc ESC/tích phân → moment. Pitch dùng `getPitchAngle()` (mrad); roll dùng `roll_H`.

Tham số Flash (prefix **R** roll / **P** pitch): `RTV/PTV`, `RTM/PTM`, `RTE/PTE`, `RTI/PTI`, `RTmu/PTmu`, `RKPF/PKPF`, `RKPV/PKPV`, `RKDV/PKDV`, `rKlmf/pKlmf`, `rKlmv/pKlmv`. Setpoint: **`SetPt`** / **`SetRl`** (mrad). Gọi `setCtrlParam()` tự tính `Kpv`, `Kdv`, `Kpf` từ `mTe`, `mTv`, `mTmu`, `mTf`.

### 5.2 Cascade PID (`CtrlAlg=1`)

| Flash | Vòng |
|---|---|
| `PITCH_P/I/D`, `PRateP/I/D` | Pitch (bench chính trên Qt) |
| `ROLL_P/I/D`, `RRateP/I/D` | Roll (CLI hoặc tune riêng) |

Feedback rate: `rates.x/y` (rad/s, LPF). Feedback góc pitch: `getPitchAngle()`; roll: `roll_acc_rad`.

### 5.3 SMC (`CtrlAlg=2`)

```
e = cmd − θ,  v_sp = constrain(λ·e, ±Vmax),  s = v_sp − K_rate·ω
u = Umax·tanh(K·s/Φ) + Ki·∫e   → LPF (SMCLpfa / SMCLpft)   → torqueTarget.y (Umax=±1)
```

| Flash | Mặc định | Ý nghĩa |
|---|---|---|
| `SMCLm` | 3.0 | λ |
| `SMCK` | 0.1 | K trong tanh |
| `SMCPhi` | 3.0 | Φ |
| `SMCKrt` | 1.0 | K_rate |
| `SMCKi` | 0.0 | Ki |
| `SMCImax` | 5.0 | giới hạn ∫e |
| `SMCVmax` | 30.0 | giới hạn \|v_sp\| (cùng đơn vị λ·e) |
| `SMCLpfa` | 0.1 | α LPF đầu ra (khi SMCLpft=0) |
| `SMCLpft` | 0.0 | τ LPF: α=dt/τ nếu τ>dt |

`pitch_cmd` = `SetPt` (mrad→rad); `ω` = `rates.y` (rad/s).

---

## 6. Nguồn góc pitch (`getPitchAngle()`)

Dùng cho PD-PI pitch, cascade PID, SMC, log, TakeLog.

| `QuatEn` | `LPF_En` | Nguồn |
|---|---|---|
| 1 | 0 hoặc 1 | `attitude.getPitch()` (Mahony/quaternion) |
| 0 | 2 | `pitch_comp_rad` (Complementary, `CFAlpha`) |
| 0 | 3 | `pitch_kalman_rad` (`KQang`, `KRmeas`; `KQbias=0.003` cố định) |
| 0 | 0 | `pitch_acc_rad` (acc thô) |

**Qt:** `groupBoxPitchSource` (Quaternion / No Quaternion) + `groupBoxLPF` / `groupBoxComp` / `groupBoxKalman` → `syncPitchSourceToFirmware()`.

`LPF_En ≥ 0.5`: LPF chung cho `acc`, `rates`, đạo hàm PID-D (`LPFAlp`).

---

## 7. Giao tiếp serial (Qt ↔ ESP32)

### 7.1 Lệnh Qt thường dùng

| Lệnh | Tác dụng |
|---|---|
| `arm` / `disarm` | Arm/disarm + reset controllers |
| `p` / `p <name> <val>` | Đọc/ghi tham số |
| `save` | Ghi Flash |
| `mtr 4 <0..1000>` | Thrust chung |
| `dscnl <1..4> <0/1>` | 0=bật moment kênh roll/pitch/yaw/all; 1=tắt |
| `log <0/1/2>` | Tắt / telemetry pitch / cùng format |
| `takelog` | Dump buffer 6000 mẫu → CSV |
| `logtel <0..7>` | Bitmask cột telemetry (0→7) |
| `pit` | Debug nguồn pitch |
| `imu` / `sys` / `reboot` | Thông tin / reset |

### 7.2 Telemetry live (`log 1` hoặc `2`, ≤50 Hz)

Tối đa 3 số nguyên (bitmask `pitchTelMask`):

```
col0: getPitchAngle() × 1000        (milli-rad)
col1: rates.y hoặc gyroBc.y × 572.96 (deci-deg/s; LPF nếu LPF_En≥0.5)
col2: torqueTarget.y × 1000
```

Qt parse **cùng scale** → plot pitch (°), ω (°/s), torque.

### 7.3 TakeLog

```
TAKELOG_START
pitch_e4, gyro_dDegS, torque_e4   (int16 CSV)
TAKELOG_END
```

Qt lưu `log/takelog_<timestamp>.csv` — cột `pitch_deg, gyro_dps, torque`.

---

## 8. Qt ground station (`Qt_file/`)

```bash
cd Qt_file && qmake Qt_Flix.pro && make -j$(nproc) && ./Qt_Flix
```

Modules: Widgets, SerialPort, 3D (Core/Render/Input/Extras), PrintSupport, QCustomPlot.

| Nhóm UI | Chức năng |
|---|---|
| Arm/Disarm, thrust slider | An toàn: không arm nếu thrust>0; disarm → thrust=0 + pitch-only |
| `checkBoxRoll/Pitch/Yaw` | `dscnl` — mặc định chỉ Pitch |
| Tab **Classic PID** (mặc định) | 6 field pitch θ + ω |
| Tab **PD-PI** | RK*/RT*/PK*/PT* + Kalman EMA |
| Tab **SMC** | SMCK, SMCLm, SMCPhi, SMCKrt, SMCKi, SMCImax |
| Nút **PD-PI / PID / SMC** | `p CtrlAlg 0/1/2` (lưu QSettings) |
| **Pitch deg** | `p SetPt` (° → mrad) — không còn Roll deg trên UI |
| Log on / Log Um / Log off | `log 1/2/0`; plot 3 kênh pitch |
| Save Log | `takelog` |
| 3D `uav.stl` | Pitch từ telemetry; `m_pitchSign` trong `dialog.h` nếu ngược |

Mở serial: `disarm` → pitch-only → `sendAllParams()` → **không** tự bật log.

---

## 9. Build firmware

- Arduino IDE hoặc VS Code + arduino-cli / PlatformIO
- Board: **ESP32 Dev Module**
- Libs: `MPU6050`, MAVLink, Arduino-ESP32
- Entry: `flixESPDrone/flixESPDrone.ino`

---

## 10. Quy tắc quan trọng

- **Cập nhật file này** khi đổi API serial, tham số Flash, hoặc hành vi Qt/firmware.
- **Flash:** chỉ ghi khi `save`; namespace `"flix"`.
- **Torque mặc định firmware:** `bnRll=1`, `bnPtch=0`, `bnYaw=1` — pitch moment tắt cho đến khi `dscnl 2 0`.
- **IMU:** `rates` (rad/s) = feedback rate cho PD-PI và PID; `gyro` (mrad/s) legacy/MAVLink.
- **`rotateIMU()`:** pass-through — sửa nếu IMU lệch trục.
- **`MAV_Disable`≠0:** WiFi nhận lệnh text thay MAVLink.
- Mode bay: **STAB**; PD-PI macro `__SelCtrl = 1`.
