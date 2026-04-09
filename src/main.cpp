#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include <NimBLEDevice.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <Arduino.h>

// UUID
#define SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_MPU_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_HEALTH_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define CHAR_COMMAND_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_DATETIME_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab"

// Pin & OLED
#define I2C_SDA   8
#define I2C_SCL   9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
#define OLED_RST  -1

// Button Pins — PULL UP, active LOW (nhấn nối GND, thả = HIGH)
#define BTN_SWITCH_PIN   10   // Nút 1: chuyển layout
#define BTN_CONFIRM_PIN  3    // Nút 2: confirm / đo
#define BTN_DEBOUNCE_MS  250  // ms chống rung

// MPU6050
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT    0x6B
#define MPU_ACCEL_XOUT  0x3B
#define MPU_INTERVAL_MS 25
#define MPU_BATCH_SIZE  8

// ── MAX30102 — Sensor config (từ max.cpp) ───────────────────
#define CFG_LED_BRIGHTNESS  0x7F
#define CFG_SAMPLE_AVG         4
#define CFG_SAMPLE_RATE      100   // Hz
#define CFG_PULSE_WIDTH      411
#define CFG_ADC_RANGE      16384
#define CFG_LED_MODE_RED_IR    2

#define FINGER_THR         15000UL

// DC block
#define DC_ALPHA_FAST  0.80f
#define DC_ALPHA_SLOW  0.98f
#define DC_WARMUP_SAMPLES 100

// Peak detection
#define PEAK_THRESHOLD_FACTOR   0.350f
#define PEAK_MIN_DISTANCE_MS    100
#define PEAK_MAX_DISTANCE_MS    1500

// HR
#define BPM_HIST_SIZE_MAX   5
#define BPM_MIN        40
#define BPM_MAX        200

// SpO2
#define SPO2_WINDOW_SIZE  100
#define SPO2_MIN           85
#define SPO2_MAX          100

// AGC
#define AGC_DC_TARGET_LOW    25000.0f
#define AGC_DC_TARGET_HIGH  45000.0f
#define AGC_DC_OPTIMAL      100000.0f
#define AGC_LED_MIN         0x3F
#define AGC_LED_MAX         0xFF
#define AGC_STEP_UP         0x08
#define AGC_STEP_DOWN       0x04
#define AGC_INTERVAL_MS     500
#define AGC_SETTLE_SAMPLES  20

// Fall Detection
#define FREE_FALL_THR  0.4f
#define IMPACT_THR     2.5f
#define FALL_WINDOW_MS 500
#define FALL_ALERT_MS  5000

// BPM History
#define BPM_HIST_SIZE        30
#define BPM_HIST_INTERVAL_MS 30000UL

// Chế độ đo chính xác (Layout 1, Nút 2)
#define MEAS_DURATION_MS 20000UL

// Firmware version
#define FW_VERSION "1.1.0"

// Số layout
#define NUM_LAYOUTS 3

// Health BLE interval
#define HEALTH_BLE_INTERVAL_MS 1000UL

// ── AGC struct (từ max.cpp) ──────────────────────────────────
struct AGC {
    uint8_t  ledLevel      = CFG_LED_BRIGHTNESS;
    float    dcAccum       = 0.0f;
    int      dcCount       = 0;
    uint32_t lastAdjustMs  = 0;
    int      settleCount   = 0;
    bool     active        = false;
    int      adjustCount   = 0;

    bool update(float dc, uint32_t nowMs, MAX30105& sensor, bool fingerOn) {
        if (!fingerOn) { reset(); return false; }
        if (!active) return false;
        if (settleCount > 0) { settleCount--; return true; }

        dcAccum += dc;
        dcCount++;

        if (nowMs - lastAdjustMs < AGC_INTERVAL_MS) return false;
        if (dcCount == 0) return false;

        float avgDC = dcAccum / dcCount;
        dcAccum = 0; dcCount = 0;
        lastAdjustMs = nowMs;

        uint8_t newLevel = ledLevel;

        if (avgDC < AGC_DC_TARGET_LOW) {
            int step = AGC_STEP_UP;
            if (avgDC < AGC_DC_TARGET_LOW * 0.5f) step = AGC_STEP_UP * 2;
            newLevel = (uint8_t)min((int)ledLevel + step, (int)AGC_LED_MAX);
        } else if (avgDC > AGC_DC_TARGET_HIGH) {
            int step = AGC_STEP_DOWN;
            if (avgDC > AGC_DC_TARGET_HIGH * 1.2f) step = AGC_STEP_DOWN * 2;
            newLevel = (uint8_t)max((int)ledLevel - step, (int)AGC_LED_MIN);
        }

        if (newLevel != ledLevel) {
            ledLevel    = newLevel;
            settleCount = AGC_SETTLE_SAMPLES;
            adjustCount++;
            sensor.setPulseAmplitudeIR(ledLevel);
            sensor.setPulseAmplitudeRed(ledLevel);
            Serial.printf("[AGC] #%d DC=%.0f → LED=0x%02X (%s)\n",
                adjustCount, avgDC, ledLevel,
                avgDC < AGC_DC_TARGET_LOW ? "INCREASE" : "DECREASE");
            return true;
        }

        static uint32_t lastLog = 0;
        if (nowMs - lastLog > 5000) {
            lastLog = nowMs;
            Serial.printf("[AGC] DC=%.0f  LED=0x%02X  status=STABLE\n", avgDC, ledLevel);
        }
        return false;
    }

    void reset() {
        ledLevel    = CFG_LED_BRIGHTNESS;
        dcAccum     = 0; dcCount = 0;
        lastAdjustMs = 0; settleCount = 0;
        active      = false; adjustCount = 0;
    }

    void activate(MAX30105& sensor) {
        active      = true;
        settleCount = AGC_SETTLE_SAMPLES;
        Serial.printf("[AGC] Activated — initial LED=0x%02X\n", ledLevel);
    }
} agc;

// ── Kalman 1D (từ max.cpp) ───────────────────────────────────
struct Kalman1D {
    float Q, R, P, x;
    Kalman1D(float q=0.02f, float r=0.3f): Q(q),R(r),P(1.0f),x(0.0f){}
    float process(float z){
        P += Q;
        float K = P/(P+R);
        x += K*(z-x);
        P *= (1.0f-K);
        return x;
    }
    void reset(){ P=1.0f; x=0.0f; }
};

// ── Butterworth Bandpass (từ max.cpp) ────────────────────────
struct ButterworthBP {
    const float S1b0= 0.06745527f, S1b1=0.0f, S1b2=-0.06745527f;
    const float S1a1=-1.82267479f, S1a2= 0.86506072f;
    float w1_0=0, w1_1=0;

    const float S2b0= 1.0f,        S2b1=0.0f, S2b2=-1.0f;
    const float S2a1=-1.96521036f, S2a2= 0.96601021f;
    float w2_0=0, w2_1=0;

    float process(float x){
        float y1 = S1b0*x + w1_0;
        w1_0 = S1b1*x - S1a1*y1 + w1_1;
        w1_1 = S1b2*x - S1a2*y1;

        float y2 = S2b0*y1 + w2_0;
        w2_0 = S2b1*y1 - S2a1*y2 + w2_1;
        w2_1 = S2b2*y1 - S2a2*y2;
        return y2;
    }
    void reset(){ w1_0=w1_1=w2_0=w2_1=0; }
};

// ── Median buf (giữ lại từ smw.cpp cho BPM history) ─────────
#define MED_LEN 5
struct MedianBuf { int32_t arr[MED_LEN] = {}; int head = 0, count = 0; };

enum FallState  { FALL_IDLE, FALL_FREEFALL, FALL_DETECTED };
enum MeasState  { MEAS_IDLE, MEAS_RUNNING, MEAS_DONE };

// ── FreeRTOS Mutex ────────────────────────────────────────────
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t bleMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t dtMutex;
SemaphoreHandle_t histMutex;

// ── Shared data (guarded by dataMutex) ───────────────────────
volatile int32_t   g_dispBPM      = 0;
volatile int32_t   g_dispSpO2     = 0;
volatile bool      g_fingerOn     = false;
volatile float     g_lastMag      = 1.0f;
volatile float     g_ax = 0, g_ay = 0, g_az = 0;

volatile bool      g_fallDetected  = false;
volatile uint32_t  g_fallAlertTime = 0;
volatile FallState g_fallState     = FALL_IDLE;
volatile uint32_t  g_freeFallTime  = 0;

// ── UI State ──────────────────────────────────────────────────
volatile int g_currentLayout = 0;

// ── Button ISR flags ─────────────────────────────────────────
volatile bool     g_btn1Pressed = false;
volatile bool     g_btn2Pressed = false;
volatile uint32_t g_btn1LastISR = 0;
volatile uint32_t g_btn2LastISR = 0;

void IRAM_ATTR isr_btn1() {
    uint32_t now = millis();
    if (now - g_btn1LastISR > BTN_DEBOUNCE_MS) {
        g_btn1LastISR = now; g_btn1Pressed = true;
    }
}
void IRAM_ATTR isr_btn2() {
    uint32_t now = millis();
    if (now - g_btn2LastISR > BTN_DEBOUNCE_MS) {
        g_btn2LastISR = now; g_btn2Pressed = true;
    }
}

// ── BPM History ───────────────────────────────────────────────
int32_t  g_bpmHist[BPM_HIST_SIZE]  = {};
int      g_bpmHistHead  = 0;
int      g_bpmHistCount = 0;
int32_t  g_bpmMinToday  = 999;
int32_t  g_bpmMaxToday  = 0;
uint32_t g_lastHistUpdate = 0;

// ── Measurement state ─────────────────────────────────────────
volatile MeasState g_measState  = MEAS_IDLE;
volatile uint32_t  g_measStart  = 0;
volatile int32_t   g_measAccum  = 0;
volatile int       g_measCount  = 0;
volatile int32_t   g_measResult = 0;

// ── RTC nội bộ ────────────────────────────────────────────────
struct RTCState {
    uint32_t syncEpoch  = 0;
    uint32_t syncMillis = 0;
    bool     synced     = false;
    uint16_t year = 2000; uint8_t mon = 1; uint8_t day = 1;
    uint8_t  hour = 0;    uint8_t min = 0; uint8_t sec = 0;
};
volatile RTCState g_rtc;

// ── NimBLE objects ────────────────────────────────────────────
NimBLEServer*         pServer       = nullptr;
NimBLECharacteristic* pMpuChar      = nullptr;
NimBLECharacteristic* pHealthChar   = nullptr;
NimBLECharacteristic* pCommandChar  = nullptr;
NimBLECharacteristic* pDatetimeChar = nullptr;
bool                  bleConnected  = false;

// ── MAX30102 objects (engine từ max.cpp) ─────────────────────
MAX30105    sensor;
Kalman1D    kalIR (0.02f, 0.3f), kalRed(0.02f, 0.3f);
ButterworthBP bpIR, bpRed;

// State variables (từ max.cpp)
float dcIR=0, dcRed=0;
bool  dcInit=false;

float    peakBuf[3]   = {};
uint32_t peakTimes[3] = {};
uint32_t lastPeakMs   = 0;
float    ampMax       = 0;
const float ampDecay  = 0.97f;

int warmupCount=0;

uint32_t rriHistory[BPM_HIST_SIZE_MAX] = {};
int rriHead=0, rriCount=0;

int32_t bpmHistory3[3] = {0};
int bpmHistoryIdx3 = 0;
int bpmHistoryCount3 = 0;

float sumAC2_IR=0, sumAC2_Red=0, sumDC_IR=0, sumDC_Red=0;
int   spo2Cnt=0;

int32_t  dispBPM_local=0, dispSpO2_local=0;
bool     dispFinger_local=false;
float    dispIR_AC=0;
uint32_t dispIR_Raw=0;
bool     peakNow=false;

#define WAVE_W 64
float waveBuf[WAVE_W]={};
int   waveIdx=0;

// ── OLED ──────────────────────────────────────────────────────
Adafruit_SSD1306 oled(OLED_W, OLED_H, &Wire, OLED_RST);

// ── BLE Callbacks ─────────────────────────────────────────────
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        bleConnected = true; Serial.println("[BLE] Connected");
    }
    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
        bleConnected = false;
        Serial.printf("[BLE] Disconnected reason=%d → re-advertising\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        if (val == "FALL:YES") {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_fallDetected = true; g_fallAlertTime = millis();
            xSemaphoreGive(dataMutex);
        } else if (val == "FALL:NO") {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_fallDetected = false;
            xSemaphoreGive(dataMutex);
        } else if (val == "RESET") {
            Serial.println("[BLE] RESET");
        }
    }
};

// ── RTC helpers ───────────────────────────────────────────────
static bool isLeap(uint16_t y) { return (y%4==0 && y%100!=0) || (y%400==0); }
static const uint8_t DAYS_IN_MONTH[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

static void rtcSync(const char* s) {
    if (!s || strlen(s) < 19) return;
    uint16_t yr = (s[0]-'0')*1000+(s[1]-'0')*100+(s[2]-'0')*10+(s[3]-'0');
    uint8_t  mo = (s[5]-'0')*10+(s[6]-'0');
    uint8_t  dy = (s[8]-'0')*10+(s[9]-'0');
    uint8_t  hh = (s[11]-'0')*10+(s[12]-'0');
    uint8_t  mm = (s[14]-'0')*10+(s[15]-'0');
    uint8_t  ss = (s[17]-'0')*10+(s[18]-'0');
    if (mo<1||mo>12||dy<1||dy>31) return;
    if (hh>23||mm>59||ss>59)      return;
    xSemaphoreTake(dtMutex, portMAX_DELAY);
    g_rtc.year=yr; g_rtc.mon=mo; g_rtc.day=dy;
    g_rtc.hour=hh; g_rtc.min=mm; g_rtc.sec=ss;
    g_rtc.syncMillis=millis(); g_rtc.synced=true;
    xSemaphoreGive(dtMutex);
    Serial.printf("[RTC] Synced: %04d-%02d-%02d %02d:%02d:%02d\n",yr,mo,dy,hh,mm,ss);
}

static void rtcGetParts(char* timeBuf, size_t tLen, char* dateBuf, size_t dLen) {
    xSemaphoreTake(dtMutex, portMAX_DELAY);
    bool synced = g_rtc.synced;
    uint16_t yr=g_rtc.year; uint8_t mo=g_rtc.mon, dy=g_rtc.day;
    uint8_t hh=g_rtc.hour, mm=g_rtc.min, ss=g_rtc.sec;
    uint32_t sm=g_rtc.syncMillis;
    xSemaphoreGive(dtMutex);

    if (!synced) {
        snprintf(timeBuf,tLen,"--:--");
        snprintf(dateBuf,dLen,"--/--");
        return;
    }
    uint32_t elapsed = (millis()-sm)/1000;
    ss += elapsed;
    if (ss>=60){mm+=ss/60;ss%=60;}
    if (mm>=60){hh+=mm/60;mm%=60;}
    if (hh>=24){
        uint32_t ex=hh/24; hh%=24;
        while(ex>0){
            uint8_t dMax=DAYS_IN_MONTH[mo-1];
            if(mo==2&&isLeap(yr))dMax=29;
            uint8_t rem=dMax-dy;
            if(ex<=rem){dy+=ex;ex=0;}
            else{ex-=rem+1;dy=1;mo++;if(mo>12){mo=1;yr++;}}
        }
    }
    snprintf(timeBuf, tLen, "%02d:%02d", hh, mm);
    snprintf(dateBuf, dLen, "%02d/%02d", dy, mo);
}

class DatetimeCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string val = c->getValue();
        rtcSync(val.c_str());
    }
};

// ── BLE init & helpers ────────────────────────────────────────
void bleInit() {
    NimBLEDevice::init("ESP32-SmartWatch_TE");
    NimBLEDevice::setMTU(64);
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    NimBLEService* pSvc = pServer->createService(SERVICE_UUID);
    pMpuChar    = pSvc->createCharacteristic(CHAR_MPU_UUID,     NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    pHealthChar = pSvc->createCharacteristic(CHAR_HEALTH_UUID,  NIMBLE_PROPERTY::READ|NIMBLE_PROPERTY::NOTIFY);
    pCommandChar= pSvc->createCharacteristic(CHAR_COMMAND_UUID, NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::WRITE_NR);
    pCommandChar->setCallbacks(new CommandCallbacks());
    pDatetimeChar=pSvc->createCharacteristic(CHAR_DATETIME_UUID,NIMBLE_PROPERTY::WRITE|NIMBLE_PROPERTY::WRITE_NR);
    pDatetimeChar->setCallbacks(new DatetimeCallbacks());
    pSvc->start(); pServer->start();
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("ESP32-SmartWatch_TE");
    pAdv->addServiceUUID(SERVICE_UUID);
    pAdv->enableScanResponse(true);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE] Advertising started");
}

static void bleSendMPU(float* mags, int count) {
    if (!bleConnected || !pMpuChar) return;
    char buf[64]; int pos = snprintf(buf, sizeof(buf), "M:");
    for (int i=0; i<count && pos<(int)sizeof(buf)-7; i++) {
        if(i>0) buf[pos++]='|';
        pos += snprintf(buf+pos, sizeof(buf)-pos, "%.2f", mags[i]);
    }
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5))==pdTRUE) {
        pMpuChar->setValue((uint8_t*)buf,(size_t)pos); pMpuChar->notify();
        xSemaphoreGive(bleMutex);
    }
}

static void bleSendHealth(int32_t bpm, int32_t spo2, bool finger, bool fall) {
    if (!bleConnected || !pHealthChar) return;
    char buf[40];
    int len = snprintf(buf, sizeof(buf), "B:%ld,S:%ld,F:%d,FALL:%d",
                       bpm, spo2, finger?1:0, fall?1:0);
    if (xSemaphoreTake(bleMutex, pdMS_TO_TICKS(5))==pdTRUE) {
        pHealthChar->setValue((uint8_t*)buf,(size_t)len); pHealthChar->notify();
        xSemaphoreGive(bleMutex);
        Serial.printf("[BLE-Health] %s\n", buf);
    }
}

// ── MPU6050 helpers ───────────────────────────────────────────
static void mpuWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
    Wire.endTransmission(true);
}
static bool mpuInit() {
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission(true)!=0) { Serial.printf("[MPU] Not found at 0x%02X\n",MPU_ADDR); return false; }
    mpuWriteReg(MPU_PWR_MGMT, 0x00); delay(100);
    Serial.printf("[MPU] OK at 0x%02X\n", MPU_ADDR); return true;
}
struct MpuData { float ax,ay,az,mag; };
static MpuData mpuRead() {
    MpuData d = { g_ax, g_ay, g_az, g_lastMag };
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR,(uint8_t)6,(uint8_t)true);
    if (Wire.available()<6) return d;
    int16_t rx=(Wire.read()<<8)|Wire.read();
    int16_t ry=(Wire.read()<<8)|Wire.read();
    int16_t rz=(Wire.read()<<8)|Wire.read();
    d.ax=rx/16384.0f; d.ay=ry/16384.0f; d.az=rz/16384.0f;
    d.mag=sqrtf(d.ax*d.ax+d.ay*d.ay+d.az*d.az);
    return d;
}

// ── Fall Detection ────────────────────────────────────────────
static void updateFallDetect(float mag) {
    uint32_t now = millis();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    switch (g_fallState) {
        case FALL_IDLE:
            if (mag < FREE_FALL_THR) { g_fallState=FALL_FREEFALL; g_freeFallTime=now; }
            break;
        case FALL_FREEFALL:
            if (mag > IMPACT_THR) {
                g_fallDetected=true; g_fallAlertTime=now; g_fallState=FALL_IDLE;
                Serial.printf("[FALL] DETECTED! mag=%.2f\n", mag);
            } else if (now-g_freeFallTime > FALL_WINDOW_MS) {
                g_fallState=FALL_IDLE;
            }
            break;
        default: g_fallState=FALL_IDLE; break;
    }
    if (g_fallDetected && (now-g_fallAlertTime > FALL_ALERT_MS))
        g_fallDetected=false;
    xSemaphoreGive(dataMutex);
}

// ── MAX30102 — BPM/SpO2 helpers (từ max.cpp) ─────────────────
static int32_t calcSpO2(){
    if(spo2Cnt<20) return 0;
    float rmsIR  = sqrtf(sumAC2_IR /spo2Cnt);
    float rmsRed = sqrtf(sumAC2_Red/spo2Cnt);
    float avgIR  = sumDC_IR /spo2Cnt;
    float avgRed = sumDC_Red/spo2Cnt;
    if(avgIR<1000.0f||avgRed<1000.0f) return 0;
    if(rmsIR<1.0f) return 0;
    float R    = (rmsRed/avgRed)/(rmsIR/avgIR);
    float spo2 = 110.0f - 25.0f*R;
    int32_t s=(int32_t)roundf(spo2);
    if(s<SPO2_MIN||s>SPO2_MAX) return 0;
    return s;
}

// ── processSample — engine từ max.cpp, tích hợp AGC ─────────
// Gọi từ taskMAX30102 với i2cMutex đã được giải phóng
static void processSample(uint32_t rawIR, uint32_t rawRed, uint32_t nowMs){

    if(!dcInit){
        dcIR=(float)rawIR; dcRed=(float)rawRed;
        kalIR.reset(); kalRed.reset();
        bpIR.reset();  bpRed.reset();
        ampMax=0; rriCount=0; rriHead=0;
        lastPeakMs=0; warmupCount=0;
        memset(peakBuf,0,sizeof(peakBuf));
        dcInit=true;
        return;
    }

    float alpha = (warmupCount < DC_WARMUP_SAMPLES) ? DC_ALPHA_FAST : DC_ALPHA_SLOW;
    dcIR  = alpha*dcIR  + (1.0f-alpha)*(float)rawIR;
    dcRed = alpha*dcRed + (1.0f-alpha)*(float)rawRed;

    // Kích hoạt AGC sau warmup
    if (warmupCount == DC_WARMUP_SAMPLES && !agc.active) {
        // Cần giữ i2cMutex khi gọi sensor methods
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            agc.activate(sensor);
            xSemaphoreGive(i2cMutex);
        } else {
            agc.active = true;
            agc.settleCount = AGC_SETTLE_SAMPLES;
        }
    }

    // AGC update
    bool agcSettle;
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        agcSettle = agc.update(dcIR, nowMs, sensor, true);
        xSemaphoreGive(i2cMutex);
    } else {
        agcSettle = false;
    }

    if (agcSettle) {
        if (warmupCount < DC_WARMUP_SAMPLES) warmupCount++;
        return;
    }

    float acIR  = (float)rawIR  - dcIR;
    float acRed = (float)rawRed - dcRed;

    float kIR  = kalIR.process(acIR);
    float kRed = kalRed.process(acRed);

    float fIR  = bpIR.process(kIR);
    float fRed = bpRed.process(kRed);

    dispIR_AC = fIR;

    bool isWarmup = (warmupCount < DC_WARMUP_SAMPLES);

    if(isWarmup){
        warmupCount++;
        ampMax = 0;
    } else {
        float absAC = fabsf(fIR);
        if(absAC > ampMax) {
            ampMax = absAC;
        } else {
            ampMax *= ampDecay;
        }
        if(ampMax > 15000) ampMax = 15000;
        if(ampMax < 10)    ampMax = 10;

        waveBuf[waveIdx] = fIR;
        waveIdx = (waveIdx + 1) % WAVE_W;

        peakBuf[0] = peakBuf[1];
        peakBuf[1] = peakBuf[2];
        peakBuf[2] = fIR;
        peakTimes[0] = peakTimes[1];
        peakTimes[1] = peakTimes[2];
        peakTimes[2] = nowMs;

        peakNow = false;

        float dynamicThr;
        if (ampMax > 500)
            dynamicThr = PEAK_THRESHOLD_FACTOR * ampMax;
        else
            dynamicThr = ampMax * 0.15f;

        bool isPeak = (peakBuf[1] > peakBuf[0]) &&
                      (peakBuf[1] > peakBuf[2]) &&
                      (peakBuf[1] > dynamicThr);

        if(isPeak && (peakTimes[1] - lastPeakMs) >= PEAK_MIN_DISTANCE_MS) {
            uint32_t rri = peakTimes[1] - lastPeakMs;
            if (rri > 1200 && ampMax < 80) {
                Serial.println("Rejected low-signal slow beat");
                return;
            }
            int ampThr = (ampMax > 500) ? 200 : 50;

            if(lastPeakMs != 0 && rri >= 500 && rri <= 1000 && ampMax > ampThr)
            {
                rriHistory[rriHead] = rri;
                rriHead = (rriHead + 1) % BPM_HIST_SIZE_MAX;
                if(rriCount < BPM_HIST_SIZE_MAX) rriCount++;

                int32_t rawBPM = 60000 / rri;
                if(dispBPM_local == 0) {
                    dispBPM_local = rawBPM;
                } else {
                    dispBPM_local = (dispBPM_local * 2 + rawBPM) / 3;
                }

                static uint32_t lastDebug = 0;
                if(millis() - lastDebug > 1000) {
                    lastDebug = millis();
                    Serial.printf("RRI=%dms → BPM=%d (raw=%d, amp=%.0f, thr=%.0f, LED=0x%02X)\n",
                                 rri, dispBPM_local, rawBPM, ampMax, dynamicThr, agc.ledLevel);
                }

                peakNow = true;
            }
            lastPeakMs = peakTimes[1];
        }
    }

    sumAC2_IR  += fIR * fIR;
    sumAC2_Red += fRed * fRed;
    sumDC_IR   += dcIR;
    sumDC_Red  += dcRed;
    spo2Cnt++;

    if(spo2Cnt >= SPO2_WINDOW_SIZE){
        int32_t s = calcSpO2();
        if(s > 0) dispSpO2_local = s;
        sumAC2_IR = sumAC2_Red = sumDC_IR = sumDC_Red = 0;
        spo2Cnt = 0;
    }
}

// ── BPM History helpers ───────────────────────────────────────
static void bpmHistoryPush(int32_t bpm) {
    xSemaphoreTake(histMutex, portMAX_DELAY);
    g_bpmHist[g_bpmHistHead] = bpm;
    g_bpmHistHead = (g_bpmHistHead + 1) % BPM_HIST_SIZE;
    if (g_bpmHistCount < BPM_HIST_SIZE) g_bpmHistCount++;
    if (bpm < g_bpmMinToday || g_bpmMinToday == 999) g_bpmMinToday = bpm;
    if (bpm > g_bpmMaxToday) g_bpmMaxToday = bpm;
    xSemaphoreGive(histMutex);
}

// ── Button Handler ────────────────────────────────────────────
static void handleButtons() {
    if (g_btn1Pressed) {
        g_btn1Pressed   = false;
        g_currentLayout = (g_currentLayout + 1) % NUM_LAYOUTS;
        Serial.printf("[BTN] Layout → %d\n", (int)g_currentLayout);
    }

    if (g_btn2Pressed) {
        g_btn2Pressed = false;
        int layout = g_currentLayout;

        if (layout == 1) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            MeasState ms = g_measState;
            xSemaphoreGive(dataMutex);

            if (ms == MEAS_IDLE || ms == MEAS_DONE) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measState  = MEAS_RUNNING;
                g_measStart  = millis();
                g_measAccum  = 0;
                g_measCount  = 0;
                g_measResult = 0;
                xSemaphoreGive(dataMutex);
                Serial.println("[MEAS] Started 20s measurement");
            }
        } else if (layout == 2) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            if (g_fallDetected) {
                g_fallDetected = false;
                g_fallState    = FALL_IDLE;
                Serial.println("[BTN] Fall alert reset by user");
            }
            xSemaphoreGive(dataMutex);
        }
    }
}

// ── OLED Draw helpers ─────────────────────────────────────────
static void drawPageDots(int y) {
    for (int i = 0; i < NUM_LAYOUTS; i++) {
        int px = 103 + i * 8;
        if (i == (int)g_currentLayout) oled.fillCircle(px, y, 2, SSD1306_WHITE);
        else                           oled.drawCircle(px, y, 2, SSD1306_WHITE);
    }
}

static void drawProgressBar(int x, int y, int w, int h, int pct) {
    pct = constrain(pct, 0, 100);
    oled.drawRect(x, y, w, h, SSD1306_WHITE);
    int fillW = (w - 2) * pct / 100;
    if (fillW > 0) oled.fillRect(x+1, y+1, fillW, h-2, SSD1306_WHITE);
}

static void drawBpmChart(int cx, int cy, int cw, int ch) {
    xSemaphoreTake(histMutex, portMAX_DELAY);
    int     count = g_bpmHistCount;
    int     head  = g_bpmHistHead;
    int32_t hist[BPM_HIST_SIZE];
    memcpy(hist, g_bpmHist, sizeof(g_bpmHist));
    xSemaphoreGive(histMutex);

    oled.drawRect(cx, cy, cw, ch, SSD1306_WHITE);

    if (count == 0) {
        oled.setTextSize(1);
        oled.setCursor(cx + 20, cy + ch/2 - 4);
        oled.print("No data yet");
        return;
    }

    int innerW = cw - 2;
    int innerH = ch - 2;
    float bw = (float)innerW / BPM_HIST_SIZE;

    for (int i = 0; i < count; i++) {
        int idx   = (head - count + i + BPM_HIST_SIZE) % BPM_HIST_SIZE;
        int32_t v = hist[idx];
        if (v <= 0) continue;
        int barH = map(constrain(v, 50, 150), 50, 150, 1, innerH);
        int bx   = cx + 1 + (int)(i * bw);
        int by   = cy + ch - 1 - barH;
        int bwi  = max(1, (int)bw);
        oled.fillRect(bx, by, bwi, barH, SSD1306_WHITE);
    }
}

// ── RENDER LAYOUT 0 — Tổng quan ──────────────────────────────
static void renderLayout0(int32_t bpm, int32_t spo2, bool finger, float mag, bool fall) {
    char timeBuf[8], dateBuf[8];
    rtcGetParts(timeBuf, sizeof(timeBuf), dateBuf, sizeof(dateBuf));

    oled.setTextSize(2); oled.setCursor(0, 0); oled.print(timeBuf);
    oled.setTextSize(1); oled.setCursor(68, 4); oled.print(dateBuf);

    drawPageDots(3);
    oled.drawLine(0, 17, OLED_W-1, 17, SSD1306_WHITE);

    if (!finger) {
        oled.setTextSize(1);
        oled.setCursor(18, 24); oled.print("wear device on");
        oled.setCursor(28, 34); oled.print("your wrist...");
    } else {
        oled.setTextSize(1); oled.setCursor(0, 19);  oled.print("HR");
        oled.setTextSize(2); oled.setCursor(0, 27);
        if (bpm > 0) oled.print(bpm); else oled.print("--");
        oled.setTextSize(1); oled.setCursor(36, 36); oled.print("bpm");

        oled.setTextSize(1); oled.setCursor(72, 19); oled.print("SpO2");
        oled.setTextSize(2); oled.setCursor(72, 27);
        if (spo2 > 0) oled.print(spo2); else oled.print("--");
        oled.setTextSize(1); oled.setCursor(110, 36); oled.print("%");

        // Hiển thị AGC LED level (debug)
        oled.setTextSize(1); oled.setCursor(90, 47);
        oled.printf("G:%02X", agc.ledLevel);
    }

    oled.drawLine(0, 44, OLED_W-1, 44, SSD1306_WHITE);
    oled.setTextSize(1); oled.setCursor(0, 47); oled.printf("M=%.2f==>", mag);
    oled.setCursor(54, 47);
    if (fall) {
        if ((millis()/400)%2 == 0) oled.print("!FALL!");
    } else {
        oled.print("SAFE");
    }
    oled.setCursor(0, 56);
    oled.print(bleConnected ? "BLE:Connected" : "BLE:Disconnected...");
}

// ── RENDER LAYOUT 1 — Nhịp tim chi tiết ──────────────────────
static void renderLayout1(int32_t bpm, bool finger) {
    oled.setTextSize(1); oled.setCursor(0, 0); oled.print("HEART RATE");
    drawPageDots(3);
    oled.drawLine(0, 9, OLED_W-1, 9, SSD1306_WHITE);

    drawBpmChart(0, 10, 128, 20);
    oled.drawLine(0, 31, OLED_W-1, 31, SSD1306_WHITE);

    xSemaphoreTake(histMutex, portMAX_DELAY);
    int32_t bMin = g_bpmMinToday;
    int32_t bMax = g_bpmMaxToday;
    xSemaphoreGive(histMutex);

    oled.setTextSize(1); oled.setCursor(0, 33);
    if (bMin == 999) oled.print("Min:--  Max:--");
    else             oled.printf("Min:%ld  Max:%ld", bMin, bMax);

    oled.drawLine(0, 42, OLED_W-1, 42, SSD1306_WHITE);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    MeasState ms     = g_measState;
    uint32_t  mStart = g_measStart;
    int32_t   mRes   = g_measResult;
    xSemaphoreGive(dataMutex);

    oled.setTextSize(1);
    if (ms == MEAS_IDLE) {
        oled.setCursor(0, 44); oled.print("HR: ");
        if (finger && bpm > 0) oled.printf("%ld BPM", bpm);
        else                   oled.print("-- BPM");
        oled.setCursor(0, 55); oled.print("Press BTN2 to start");
    } else if (ms == MEAS_RUNNING) {
        uint32_t elapsed = millis() - mStart;
        if (elapsed > MEAS_DURATION_MS) elapsed = MEAS_DURATION_MS;
        int pct = (int)(elapsed * 100UL / MEAS_DURATION_MS);
        oled.setCursor(0, 44);
        if ((millis()/600)%2 == 0) oled.print("Keep still!");
        else                       oled.print("Measuring...");
        drawProgressBar(0, 54, 100, 8, pct);
        oled.setCursor(104, 55); oled.printf("%d%%", pct);
    } else {
        oled.setCursor(0, 44);
        if (mRes > 0) oled.printf("Result: %ld BPM", mRes);
        else          oled.print("Result: Error!");
        oled.setCursor(0, 55); oled.print("[BTN2]measure again");
    }
}

// ── RENDER LAYOUT 2 — Gia tốc & Té ngã ──────────────────────
static void renderLayout2(float mag, float ax, float ay, float az, bool fall) {
    oled.setTextSize(1); oled.setCursor(0, 0); oled.print("FALL DETECTION");
    drawPageDots(3);
    oled.drawLine(0, 9, OLED_W-1, 9, SSD1306_WHITE);

    oled.setTextSize(1); oled.setCursor(0, 11);
    if (fall) {
        if ((millis()/400)%2 == 0) oled.print("!!Risk: HIGH!!");
    } else {
        oled.print("Risk: LOW");
    }

    oled.setCursor(0, 21); oled.printf("Mag:%.2fg", mag);
    oled.setCursor(70, 21); oled.printf("X:% .1f", ax);
    oled.setCursor(0, 30);  oled.printf("Y:% .1f  Z:% .1f", ay, az);
    oled.drawLine(0, 40, OLED_W-1, 40, SSD1306_WHITE);

    uint32_t upSec = millis() / 1000;
    uint32_t upH   = upSec / 3600;
    uint32_t upM   = (upSec % 3600) / 60;
    oled.setCursor(0, 42); oled.printf("Uptime: %luh %02lum", upH, upM);
    oled.setCursor(0, 52);
    oled.print(bleConnected ? "BLE:Connected" : "BLE:Disconnected...");
    oled.setCursor(80, 62); oled.print("v" FW_VERSION);

    if (fall) {
        oled.setCursor(0, 56);
        if ((millis()/600)%2 == 0) oled.print("[BTN2]Reset alert");
    }
}

// ── renderOLED — dispatcher chính ────────────────────────────
static void renderOLED() {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    int32_t bpm   = g_dispBPM;
    int32_t spo2  = g_dispSpO2;
    bool    finger= g_fingerOn;
    float   mag   = g_lastMag;
    float   ax    = g_ax, ay = g_ay, az = g_az;
    bool    fall  = g_fallDetected;
    xSemaphoreGive(dataMutex);

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(300)) != pdTRUE) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    switch ((int)g_currentLayout) {
        case 0:  renderLayout0(bpm, spo2, finger, mag, fall); break;
        case 1:  renderLayout1(bpm, finger);                  break;
        case 2:  renderLayout2(mag, ax, ay, az, fall);        break;
        default: renderLayout0(bpm, spo2, finger, mag, fall); break;
    }

    oled.display();
    xSemaphoreGive(i2cMutex);
}

// ── TASK 1 — MPU6050 @ ~40 Hz ─────────────────────────────────
void taskMPU(void* param) {
    TickType_t       xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod   = pdMS_TO_TICKS(MPU_INTERVAL_MS);
    float   magBatch[MPU_BATCH_SIZE];
    int     batchIdx = 0;
    MpuData lastMpu  = { 0.0f, 0.0f, 1.0f, 1.0f };

    while (true) {
        MpuData mpu;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            mpu = mpuRead();
            xSemaphoreGive(i2cMutex);
            lastMpu = mpu;
        } else {
            mpu = lastMpu;
        }

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_lastMag=mpu.mag; g_ax=mpu.ax; g_ay=mpu.ay; g_az=mpu.az;
        xSemaphoreGive(dataMutex);

        updateFallDetect(mpu.mag);

        magBatch[batchIdx++] = mpu.mag;
        if (batchIdx >= MPU_BATCH_SIZE) {
            bleSendMPU(magBatch, MPU_BATCH_SIZE);
            batchIdx = 0;
        }

        vTaskDelayUntil(&xLastWake, xPeriod);
    }
}

// ── TASK 2 — MAX30102 HR/SpO2 (engine từ max.cpp) ─────────────
// Mỗi vòng lặp: poll sensor → processSample() → cập nhật shared data
void taskMAX30102(void* param) {
    bool prevFinger = false;
    uint32_t lastHealthSend = millis();

    while (true) {
        // Poll sensor: lấy 1 sample với mutex ngắn
        bool ready = false;
        while (!ready) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                sensor.check();
                ready = sensor.available();
                xSemaphoreGive(i2cMutex);
            }
            if (!ready) vTaskDelay(1);
        }

        uint32_t rawIR = 0, rawRed = 0;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            rawIR  = sensor.getFIFOIR();
            rawRed = sensor.getFIFORed();
            sensor.nextSample();
            xSemaphoreGive(i2cMutex);
        }

        uint32_t nowMs   = millis();
        bool     finger  = (rawIR > FINGER_THR);
        dispIR_Raw = rawIR;
        dispFinger_local = finger;

        if (!finger && prevFinger) {
            // Nhấc tay — reset toàn bộ engine
            agc.reset();
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                sensor.setPulseAmplitudeIR(CFG_LED_BRIGHTNESS);
                sensor.setPulseAmplitudeRed(CFG_LED_BRIGHTNESS);
                xSemaphoreGive(i2cMutex);
            }
            dcInit=false; ampMax=0;
            rriCount=0; rriHead=0; warmupCount=0;
            spo2Cnt=0;
            bpmHistoryCount3 = 0;
            sumAC2_IR=sumAC2_Red=sumDC_IR=sumDC_Red=0;
            memset(waveBuf,0,sizeof(waveBuf));
            peakNow=false;
            dispBPM_local  = 0;
            dispSpO2_local = 0;

            xSemaphoreTake(dataMutex, portMAX_DELAY);
            g_dispBPM=0; g_dispSpO2=0; g_fingerOn=false;
            if (g_measState == MEAS_RUNNING) g_measState = MEAS_IDLE;
            xSemaphoreGive(dataMutex);
        }
        prevFinger = finger;

        if (finger) {
            processSample(rawIR, rawRed, nowMs);
        }

        // Cập nhật shared data
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        g_dispBPM  = dispBPM_local;
        g_dispSpO2 = dispSpO2_local;
        g_fingerOn = finger;
        xSemaphoreGive(dataMutex);

        Serial.printf("[MAX] BPM=%ld  SpO2=%ld  finger=%c  LED=0x%02X  AC=%.1f\n",
            dispBPM_local, dispSpO2_local, finger?'Y':'N', agc.ledLevel, dispIR_AC);

        // BPM History (mỗi 30s)
        if (nowMs - g_lastHistUpdate >= BPM_HIST_INTERVAL_MS && finger && dispBPM_local > 0) {
            g_lastHistUpdate = nowMs;
            bpmHistoryPush(dispBPM_local);
        }

        // Chế độ đo chính xác 20s
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        MeasState ms = g_measState;
        xSemaphoreGive(dataMutex);

        if (ms == MEAS_RUNNING) {
            uint32_t elapsed = millis() - g_measStart;
            if (elapsed >= MEAS_DURATION_MS) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measResult = (g_measCount > 0) ? (g_measAccum / g_measCount) : 0;
                g_measState  = MEAS_DONE;
                xSemaphoreGive(dataMutex);
                Serial.printf("[MEAS] Done. Result=%ld BPM (%d samples)\n",
                              g_measResult, g_measCount);
            } else if (finger && dispBPM_local > 0) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                g_measAccum += dispBPM_local;
                g_measCount++;
                xSemaphoreGive(dataMutex);
            }
        }

        // Gửi health qua BLE mỗi 1s
        if (nowMs - lastHealthSend >= HEALTH_BLE_INTERVAL_MS) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            bool fall = g_fallDetected;
            xSemaphoreGive(dataMutex);
            bleSendHealth(dispBPM_local, dispSpO2_local, finger, fall);
            lastHealthSend = nowMs;
        }

        vTaskDelay(1);
    }
}

// ── TASK 3 — OLED render @ 5 Hz ──────────────────────────────
void taskOLED(void* param) {
    vTaskDelay(pdMS_TO_TICKS(100));
    while (true) {
        handleButtons();
        renderOLED();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// ── I2C scan (debug) ─────────────────────────────────────────
static void i2cScan() {
    Serial.println("[I2C] Scanning...");
    uint8_t found = 0;
    for (uint8_t addr=1; addr<127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission()==0) {
            Serial.printf("  [I2C] Device at 0x%02X\n", addr); found++;
        }
    }
    if (!found) Serial.println("  [I2C] No devices found!");
    else        Serial.printf("  [I2C] %d device(s).\n", found);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
    unsigned long t0 = millis();
    while (!Serial && millis()-t0 < 3000) delay(10);
#else
    delay(500);
#endif
    Serial.println("\n=== ESP32-C3 SmartWatch BOOT v" FW_VERSION " ===");

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000UL);
    Wire.setTimeOut(3);
    delay(100);
    i2cScan();

    pinMode(BTN_SWITCH_PIN,  INPUT_PULLUP);
    pinMode(BTN_CONFIRM_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BTN_SWITCH_PIN),  isr_btn1, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_CONFIRM_PIN), isr_btn2, FALLING);
    Serial.printf("[BTN] Switch=GPIO%d  Confirm=GPIO%d (pull-up, active LOW)\n",
                  BTN_SWITCH_PIN, BTN_CONFIRM_PIN);

    i2cMutex  = xSemaphoreCreateMutex();
    bleMutex  = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    dtMutex   = xSemaphoreCreateMutex();
    histMutex = xSemaphoreCreateMutex();

    if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] FAILED");
    } else {
        oled.clearDisplay();
        oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(28, 24); oled.print("Booting v" FW_VERSION);
        oled.display();
        Serial.println("[OLED] OK");
    }

    bleInit();

    // MAX30102 — khởi tạo với config từ max.cpp
    if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("[MAX30102] FAILED");
        oled.clearDisplay();
        oled.setCursor(0,0); oled.print("MAX30102 FAIL");
        oled.setCursor(0,12); oled.print("Check 0x57 wiring");
        oled.display();
        while (true) delay(1000);
    }
    sensor.setup(CFG_LED_BRIGHTNESS, CFG_SAMPLE_AVG,
                 CFG_LED_MODE_RED_IR, CFG_SAMPLE_RATE,
                 CFG_PULSE_WIDTH, CFG_ADC_RANGE);
    Serial.printf("[MAX30102] OK — LED=0x%02X  Target DC=[%.0f, %.0f]\n",
                  CFG_LED_BRIGHTNESS, AGC_DC_TARGET_LOW, AGC_DC_TARGET_HIGH);

    if (!mpuInit()) Serial.println("[MPU6050] FAILED — fall detect disabled");

    oled.clearDisplay();
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(22, 20); oled.print("Initializing...");
    oled.setCursor(5, 32);  oled.print("BTN1:Switch Layout");
    oled.setCursor(5, 44);  oled.print("BTN2:Measure/Confirm");
    oled.display();
    delay(100);

    xTaskCreate(taskMPU,      "taskMPU",  4096, nullptr, 1, nullptr);
    xTaskCreate(taskOLED,     "taskOLED", 4096, nullptr, 1, nullptr);
    xTaskCreate(taskMAX30102, "taskMAX",  8192, nullptr,1, nullptr);

    Serial.println("[BOOT] All systems go!");
    Serial.printf("[BOOT] BTN1(GPIO%d)=Switch layout (pull-up)\n", BTN_SWITCH_PIN);
    Serial.printf("[BOOT] BTN2(GPIO%d)=Confirm/Measure (pull-up)\n", BTN_CONFIRM_PIN);
}

// ── LOOP — idle ───────────────────────────────────────────────
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}