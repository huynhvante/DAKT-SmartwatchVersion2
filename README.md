# ESP32-C3 SmartWatch — Firmware `v1.1.0`

Firmware cho thiết bị đồng hồ thông minh tự chế dựa trên **ESP32-C3**, tích hợp đo nhịp tim (HR), SpO2, phát hiện té ngã, hiển thị OLED và truyền dữ liệu qua BLE.

---

## Mục lục

- [Tính năng](#tính-năng)
- [Phần cứng](#phần-cứng)
- [Sơ đồ chân](#sơ-đồ-chân)
- [Thư viện phụ thuộc](#thư-viện-phụ-thuộc)
- [Kiến trúc phần mềm](#kiến-trúc-phần-mềm)
- [Pipeline xử lý tín hiệu MAX30102](#pipeline-xử-lý-tín-hiệu-max30102)
- [AGC — Automatic Gain Control](#agc--automatic-gain-control)
- [Giao diện OLED](#giao-diện-oled)
- [BLE Protocol](#ble-protocol)
- [Nút bấm](#nút-bấm)
- [Cấu hình & Hằng số](#cấu-hình--hằng-số)
- [Build & Flash](#build--flash)
- [Debug qua Serial](#debug-qua-serial)

---

## Tính năng

| Tính năng | Mô tả |
|---|---|
| Nhịp tim (HR) | Đo BPM theo thời gian thực bằng thuật toán peak-detection trên tín hiệu IR |
| SpO2 | Tính nồng độ oxy máu từ tỉ lệ AC/DC của IR và Red LED |
| AGC thông minh | Tự động điều chỉnh độ sáng LED theo DC level của tín hiệu |
| Phát hiện té ngã | Phát hiện free-fall + impact từ gia tốc kế MPU6050 |
| BLE | Gửi dữ liệu HR/SpO2/gia tốc/té ngã qua Bluetooth Low Energy |
| OLED 128×64 | 3 layout hiển thị: tổng quan, nhịp tim chi tiết, gia tốc & té ngã |
| RTC nội bộ | Đồng bộ thời gian qua BLE, tự chạy sau đó |
| Đo chính xác | Chế độ đo BPM trung bình 20 giây kích hoạt bằng nút bấm |
| BPM History | Lưu lịch sử nhịp tim 30 mẫu × 30 giây, hiển thị min/max |
| FreeRTOS | 3 task song song: MPU, MAX30102, OLED |

---

## Phần cứng

- **MCU:** ESP32-C3 (hoặc ESP32-C3 Super Mini)
- **Cảm biến nhịp tim / SpO2:** MAX30102
- **Cảm biến gia tốc:** MPU6050
- **Màn hình:** OLED SSD1306 128×64 (I2C)
- **Nút bấm:** 2 nút (active LOW, dùng internal pull-up)

---

## Sơ đồ chân

```
ESP32-C3        Thiết bị ngoại vi
─────────────────────────────────────
GPIO 8  (SDA)  → MAX30102 SDA
                → MPU6050 SDA
                → OLED SSD1306 SDA
GPIO 9  (SCL)  → MAX30102 SCL
                → MPU6050 SCL
                → OLED SSD1306 SCL

GPIO 10         → Nút 1 (Switch Layout) — nối GND khi nhấn
GPIO 3          → Nút 2 (Confirm/Measure) — nối GND khi nhấn

I2C Addresses:
  OLED    → 0x3C
  MAX30102→ 0x57
  MPU6050 → 0x68
```

> Tất cả thiết bị ngoại vi dùng chung bus I2C. `Wire.setClock(400000)` — Fast Mode 400 kHz.

---

## Thư viện phụ thuộc

| Thư viện | Nguồn |
|---|---|
| `Adafruit GFX Library` | Arduino Library Manager |
| `Adafruit SSD1306` | Arduino Library Manager |
| `SparkFun MAX3010x Pulse and Proximity Sensor Library` | Arduino Library Manager (`MAX30105.h`) |
| `NimBLE-Arduino` | Arduino Library Manager |
| `FreeRTOS` | Có sẵn trong ESP32 Arduino core |

> **Lưu ý:** Firmware này **không** dùng `spo2_algorithm.h` của SparkFun. SpO2 được tính bằng thuật toán R-ratio nội bộ.

---

## Kiến trúc phần mềm

```
┌─────────────────────────────────────────────────────┐
│                      setup()                        │
│  Wire / OLED / BLE / MAX30102 / MPU6050 / Buttons   │
└────────────┬────────────────────────────────────────┘
             │ xTaskCreate × 3
    ┌────────▼────────┐  ┌─────────────────┐  ┌──────────────┐
    │   taskMPU       │  │  taskMAX30102   │  │  taskOLED    │
    │  priority 1     │  │  priority 2     │  │  priority 1  │
    │  ~40 Hz         │  │  ~100 Hz        │  │  5 Hz        │
    │                 │  │                 │  │              │
    │ MPU6050 read    │  │ Sensor poll     │  │ handleButtons│
    │ Fall detect     │  │ processSample() │  │ renderOLED() │
    │ BLE send MPU    │  │ AGC update      │  │              │
    └────────┬────────┘  └────────┬────────┘  └──────┬───────┘
             │                   │                   │
             └──────────┬────────┘                   │
                        │   Shared data (dataMutex)   │
                        │   g_dispBPM, g_dispSpO2     │
                        │   g_fingerOn, g_fallDetected│
                        └───────────────────────────►─┘
```

**Mutex được sử dụng:**

| Mutex | Bảo vệ |
|---|---|
| `i2cMutex` | Toàn bộ giao tiếp I2C (sensor, OLED) |
| `dataMutex` | Shared data: BPM, SpO2, finger, fall, meas state |
| `bleMutex` | Ghi BLE characteristic |
| `dtMutex` | RTC state |
| `histMutex` | BPM history array |

---

## Pipeline xử lý tín hiệu MAX30102

Mỗi sample IR/Red đi qua các bước sau trong `processSample()`:

```
Raw IR / Red (18-bit)
        │
        ▼
  [DC Tracking]  — EMA với alpha nhanh (warmup) / chậm (ổn định)
        │
        ▼
  [AGC Update]   — Điều chỉnh LED brightness nếu DC lệch khỏi target
        │ (bỏ qua sample nếu đang settle)
        ▼
  AC = Raw − DC
        │
        ▼
  [Kalman 1D]    — Lọc nhiễu measurement (Q=0.02, R=0.3)
        │
        ▼
  [Butterworth Bandpass]  — 2-stage, passband ~0.5–3.5 Hz (30–210 BPM)
        │
        ├──────────────────────────────────────────────►
        │  IR channel                    Red channel   │
        ▼                                              │
  [Peak Detection]                              [SpO2 accumulator]
  - Dynamic threshold = 35% × ampMax             RMS AC²  + avg DC
  - Min peak distance: 100 ms                    mỗi 100 sample → calcSpO2()
  - RRI → BPM (EMA smoothing)                    R = (ACrms_Red/DCavg_Red)
                                                     /(ACrms_IR /DCavg_IR )
                                                  SpO2 = 110 − 25×R
```

**Thông số bộ lọc Butterworth:**

Stage 1 (high-pass đặc tính):
- `b = [0.06745527, 0, -0.06745527]`
- `a = [-1.82267479, 0.86506072]`

Stage 2 (low-pass đặc tính):
- `b = [1, 0, -1]`
- `a = [-1.96521036, 0.96601021]`

---

## AGC — Automatic Gain Control

AGC giữ DC level của tín hiệu IR trong vùng tối ưu `[25000, 45000]` bằng cách tự động tăng/giảm độ sáng LED IR và Red.

```
DC < 25000  →  tăng LED (INCREASE)  +0x08 bước  (×2 nếu DC < 12500)
DC > 45000  →  giảm LED (DECREASE)  −0x04 bước  (×2 nếu DC > 54000)
```

| Tham số | Giá trị | Ý nghĩa |
|---|---|---|
| `AGC_LED_MIN` | `0x3F` | Brightness tối thiểu |
| `AGC_LED_MAX` | `0xFF` | Brightness tối đa |
| `AGC_INTERVAL_MS` | `500 ms` | Tần suất điều chỉnh |
| `AGC_SETTLE_SAMPLES` | `20` | Mẫu bị bỏ qua sau mỗi lần adjust |
| `DC_WARMUP_SAMPLES` | `100` | Mẫu khởi động trước khi AGC active |

Khi nhấc tay, AGC reset về `CFG_LED_BRIGHTNESS = 0x7F`.

---

## Giao diện OLED

Chuyển layout bằng **Nút 1** (BTN_SWITCH). Dấu chấm tròn góc phải trên hiển thị layout hiện tại.

### Layout 0 — Tổng quan
```
┌────────────────────────────┐
│ HH:MM          DD/MM       │
│ ● ○ ○                      │
├────────────────────────────┤
│ HR     │  SpO2    G:XX     │
│  NNN   │   NN%             │
│  bpm   │                   │
├────────────────────────────┤
│ M=N.NN==>  SAFE / !FALL!  │
│ BLE:Connected              │
└────────────────────────────┘
```

### Layout 1 — Nhịp tim chi tiết
```
┌────────────────────────────┐
│ HEART RATE          ○ ● ○  │
│ [──BPM history chart──]    │
├────────────────────────────┤
│ Min:NN  Max:NNN            │
├────────────────────────────┤
│ HR: NNN BPM                │
│ Press BTN2 to start        │
│  (hoặc progress bar 20s)   │
└────────────────────────────┘
```

### Layout 2 — Gia tốc & Té ngã
```
┌────────────────────────────┐
│ FALL DETECTION      ○ ○ ●  │
│ Risk: LOW / !!Risk: HIGH!! │
│ Mag:N.NNg    X: N.N        │
│ Y: N.N  Z: N.N             │
├────────────────────────────┤
│ Uptime: Nh NNm             │
│ BLE:Connected          vX.X│
└────────────────────────────┘
```

---

## BLE Protocol

**Device name:** `ESP32-SmartWatch_TE`  
**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

| Characteristic | UUID (suffix) | Property | Format |
|---|---|---|---|
| MPU Data | `...26a8` | READ, NOTIFY | `M:v1\|v2\|...\|v8` (magnitude batch) |
| Health Data | `...26aa` | READ, NOTIFY | `B:<bpm>,S:<spo2>,F:<0/1>,FALL:<0/1>` |
| Command | `...26a9` | WRITE | `FALL:YES`, `FALL:NO`, `RESET` |
| Datetime | `...26ab` | WRITE | `YYYY-MM-DD HH:MM:SS` |

**Health Data** gửi mỗi 1 giây khi có kết nối BLE.  
**MPU Data** gửi theo batch 8 mẫu (~200 ms / batch ở 40 Hz).

**Đồng bộ thời gian:** Ghi chuỗi `2025-01-15 08:30:00` vào Datetime characteristic để sync RTC nội bộ.

---

## Nút bấm

Cả hai nút dùng **internal pull-up**, active LOW (nhấn nối GPIO xuống GND).

| Nút | GPIO | Chức năng |
|---|---|---|
| BTN1 (Switch) | 10 | Chuyển layout 0 → 1 → 2 → 0... |
| BTN2 (Confirm) | 3 | Layout 1: bắt đầu / xem lại đo 20s; Layout 2: reset fall alert |

Debounce bằng ISR: 250 ms.

---

## Cấu hình & Hằng số

### MAX30102 Sensor

| Define | Giá trị | Mô tả |
|---|---|---|
| `CFG_LED_BRIGHTNESS` | `0x7F` | Độ sáng LED khởi tạo |
| `CFG_SAMPLE_AVG` | `4` | Số mẫu trung bình trong FIFO |
| `CFG_SAMPLE_RATE` | `100` Hz | Tần số lấy mẫu |
| `CFG_PULSE_WIDTH` | `411` | Pulse width (µs) |
| `CFG_ADC_RANGE` | `16384` | ADC range (14-bit) |
| `FINGER_THR` | `15000` | Ngưỡng IR để nhận diện có ngón tay |

### Peak Detection & HR

| Define | Giá trị | Mô tả |
|---|---|---|
| `PEAK_THRESHOLD_FACTOR` | `0.35` | Ngưỡng peak = 35% × ampMax |
| `PEAK_MIN_DISTANCE_MS` | `100 ms` | Khoảng cách tối thiểu giữa 2 peak |
| `BPM_MIN / BPM_MAX` | `40 / 200` | Giới hạn BPM hợp lệ |
| `SPO2_WINDOW_SIZE` | `100` samples | Cửa sổ tính SpO2 |

### Fall Detection

| Define | Giá trị | Mô tả |
|---|---|---|
| `FREE_FALL_THR` | `0.4 g` | Ngưỡng gia tốc để nhận diện free-fall |
| `IMPACT_THR` | `2.5 g` | Ngưỡng va chạm sau free-fall |
| `FALL_WINDOW_MS` | `500 ms` | Cửa sổ thời gian giữa free-fall và impact |
| `FALL_ALERT_MS` | `5000 ms` | Thời gian hiển thị cảnh báo té ngã |

---

## Build & Flash

### PlatformIO (khuyến nghị)

```ini
; platformio.ini
[env:esp32-c3-devkitm-1]
platform = espressif32
board = esp32-c3-devkitm-1
framework = arduino
monitor_speed = 115200
lib_deps =
    adafruit/Adafruit GFX Library
    adafruit/Adafruit SSD1306
    sparkfun/SparkFun MAX3010x Pulse and Proximity Sensor Library
    h2zero/NimBLE-Arduino
```

```bash
pio run --target upload
pio device monitor --baud 115200
```

### Arduino IDE

1. Cài đặt ESP32 board package (Espressif)
2. Chọn board: **ESP32C3 Dev Module**
3. Cài các thư viện trong Library Manager (xem bảng ở trên)
4. Upload và mở Serial Monitor @ 115200 baud

---

## Debug qua Serial

Firmware in log chi tiết ra Serial @ 115200 baud:

```
[BOOT] ===  ESP32-C3 SmartWatch BOOT v1.1.0 ===
[I2C]  Device at 0x3C   ← OLED
[I2C]  Device at 0x57   ← MAX30102
[I2C]  Device at 0x68   ← MPU6050
[MAX30102] OK — LED=0x7F  Target DC=[25000, 45000]
[AGC]  Activated — initial LED=0x7F
[AGC]  #1 DC=18432 → LED=0x87 (INCREASE)
RRI=750ms → BPM=80 (raw=80, amp=1240.3, thr=434.1, LED=0x87)
[MAX]  BPM=80  SpO2=97  finger=Y  LED=0x87  AC=1240.3
[FALL] DETECTED! mag=3.21
[BLE]  Connected
[BLE-Health] B:80,S:97,F:1,FALL:0
[MEAS] Started 20s measurement
[MEAS] Done. Result=78 BPM (12 samples)
```