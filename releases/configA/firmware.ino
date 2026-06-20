/*
 * M5Stack PowerHub — APRS Tracker Foundation  (Step 1)
 *
 * Features:
 *   - GPS Unit v1.1 on PORT.C (blue Grove): status LED via UART port LED
 *   - Battery level → large POWER_L LED (4-segment, green-heavy)
 *   - Big top button (BTN_OK) single press → upload current GPS position
 *   - Yellow round button (GPIO11) double-click → toggle USB-A power output
 *                                  long-press 3s → power-saving shutdown
 *   - USB-A load detection via current sensor
 *   - Power saving: 80 MHz CPU (floor — USB-Serial/JTAG needs the PLL), WiFi/BT off,
 *                   unused ports down, adaptive beacon rate (SmartBeaconing + decay)
 *
 * Wiring:
 *   GPS Unit v1.1 → PORT.C (blue Grove connector)
 *     G1 yellow → GPS RX
 *     G2 white  ← GPS TX
 *
 * LED mapping:
 *   LED_UART_P (idx 2) = PORT.C adjacent LED  → GPS status
 *   LED_PWR_L  (idx 6) = large LED near WiFi  → battery level
 *                        ↑ swap to LED_PWR_R if your unit differs
 *   LED_USB_A  (idx 1) = USB-A adjacent LED   → USB power state
 *
 * Libraries required:
 *   TinyGPS++ (install via Library Manager, or copy from ref/TinyGPSPlus-master)
 *
 * Board: "ESP32S3 Dev Module"
 *   Flash: 16MB (128Mb)  PSRAM: OPI PSRAM  USB-CDC: Disabled  Upload: UART0
 */

#include <Wire.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <sys/time.h>    // gettimeofday / settimeofday for ESP32 RTC
#include <string.h>      // strcmp (serial commands)
#include <math.h>        // fabsf (heading delta)
#include <Preferences.h> // NVS：GPS 一次性配置标志（让后续开机热启动）

// ═══════════════════════════════════════════════════════════════════════════
// Hardware constants
// ═══════════════════════════════════════════════════════════════════════════

#define SYS_SDA          45
#define SYS_SCL          48
#define GPS_RX_PIN        2   // PORT.C white  ← GPS TX
#define GPS_TX_PIN        1   // PORT.C yellow → GPS RX
#define BTN_SELECT_GPIO  11   // USR_SW2 yellow button (direct GPIO, unused here)

// ── CatM (SIM7080G) on PORT.A ────────────────────────────────────────────────
#define CATM_RX_PIN  16          // PORT.A yellow ← CatM TX
#define CATM_TX_PIN  15          // PORT.A white  → CatM RX
#define CATM_BAUD    115200

// ── GNSS 来源选择 ────────────────────────────────────────────────────────────
//   1 = 配置B：SIM7080G 二合一(Unit CatM GNSS)内置 GNSS 与 LTE 分时共用 PORT.A，
//             腾出 PORT.C 给 LCD。位置靠 CGNSINF 轮询；发包时 GNSS↔LTE 切换。
//   0 = 配置A：PORT.C 独立 ATGM336H 连续 NMEA + PORT.A 的 SIM7080G 专做 4G。
// 换硬件改这一个数即可；两套逻辑都在，互不影响。
#define GNSS_TIMESHARE 0
// 部署配置（APN / 服务器域名·端口·路径）抽到 config.h，不提交到 git。
// 首次编译前：复制 config.example.h 为 config.h 并填入自己的值。
#include "config.h"

#if GNSS_TIMESHARE
// 配置B：PORT.C 腾出的口接 Unit LCD 1.14"（I2C，ST7789 135x240，板载控制器 0x3E）。
// 由 M5GFX 的 M5UnitLCD 驱动；走 ESP32 I2C 端口1（端口0 被 PowerHub 占用）。
#include <M5UnitLCD.h>
#endif

// ── PowerHub I2C ─────────────────────────────────────────────────────────────
#define PH_ADDR   0x50
#define REG_PWR   0x00   // power control base; +n for each port (see PC_* below)
#define REG_CHG   0x50   // charge status
#define REG_PSUP  0x51   // power supply status
#define REG_BTN   0xA0   // button state register
#define REG_OFF   0xE0   // write 1 to power off

// ── Port power indices (REG_PWR + index) ─────────────────────────────────────
enum : uint8_t { PC_LED=0, PC_USB, PC_I2C, PC_UART, PC_BUS, PC_VAMETER, PC_CHARGE };

// ── LED indices (0–7) ─────────────────────────────────────────────────────────
enum : uint8_t {
    LED_USB_C=0, LED_USB_A, LED_UART_P, LED_BUS_P,
    LED_I2C_P,   LED_BAT_C, LED_PWR_L,  LED_PWR_R
};
#define GPS_LED     LED_UART_P
#define BAT_LED     LED_PWR_R    // large LED near WiFi antenna (right side)
#define USB_LED     LED_USB_A
#define CHG_LED     LED_BAT_C    // LED adjacent to small round button → external power
#define CATM_LED    LED_I2C_P    // PORT.A adjacent LED → CatM status

// ── VA monitor indices ────────────────────────────────────────────────────────
enum : uint8_t { VM_BAT=0, VM_CAN, VM_RS485, VM_USB, VM_I2C_V, VM_UART_V };

// ── Button bit positions in REG_BTN ──────────────────────────────────────────
#define BTN_OK_BIT  0
#define BTN_K2_BIT  1

// ── Register address tables ───────────────────────────────────────────────────
static const uint8_t LED_CR[8] = {0x60,0x64,0x68,0x6C,0x70,0x74,0x78,0x7C};
static const uint8_t LED_BR[8] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87};
static const uint8_t VA_VR[6]  = {0x30,0x34,0x38,0x3C,0x40,0x44};
static const uint8_t VA_IR[6]  = {0x32,0x36,0x3A,0x3E,0x42,0x46};

// ═══════════════════════════════════════════════════════════════════════════
// Colors (0x00RRGGBB)
// ═══════════════════════════════════════════════════════════════════════════

#define C_OFF       0x000000UL
#define C_RED       0xFF0000UL
#define C_GREEN     0x00FF00UL
#define C_BLUE      0x0000FFUL
#define C_YELLOW    0xFFFF00UL
#define C_WHITE     0xFFFFFFUL

// ═══════════════════════════════════════════════════════════════════════════
// Tuning parameters
// ═══════════════════════════════════════════════════════════════════════════

// Must stay >= 80 MHz. This board's console is the USB-Serial/JTAG controller
// (FQBN USBMode=hwcdc), whose 48 MHz USB clock comes from the PLL. Selecting any
// sub-80 MHz freq (40/20/10) switches the CPU to the bare XTAL and powers the PLL
// DOWN, which kills the USB link (no serial, no esptool auto-reset → looks bricked,
// recover via hold-reset download mode) and can trip the IWDT. So 80 is the floor
// while USB matters. For lower idle power on battery, light-sleep or scale to 10 MHz
// ONLY when external power is absent (USB already gone) — see notes, not done here.
static const uint8_t  CPU_MHZ           = 80;
static const uint32_t GPS_DETECT_MS     = 10000;  // max wait before declaring "no module"
static const uint32_t GPS_RAW_DUMP_MS   = 0;//600000; // print raw NMEA for first 60 s (diagnostic)
static const uint32_t GPS_INIT_SHOW_MS  = 1500;   // duration of init-OK blue flash
static const uint32_t BAT_READ_MS       = 15000;  // battery read period
static const uint32_t LED_REFRESH_MS    = 300;    // LED update period (also controls blink)
static const uint32_t BLINK_HALF_MS     = 500;    // half-period of low-battery blink
static const uint32_t USB_CHK_MS        = 3000;   // USB current check period
static const uint32_t BTN_SCAN_MS       = 30;     // button polling period (faster = more responsive)
static const uint32_t DBLCLICK_MS       = 600;    // max gap for double-click
static const uint32_t LONGPRESS_MS      = 3000;   // yellow button hold → power off
static const int16_t  USB_LOAD_MA       = 30;     // current threshold: load present
static const uint8_t  BR_GPS            = 12;     // GPS LED brightness  (0-255)
static const uint8_t  BR_BAT            = 50;     // battery indicator LED brightness (kept bright)
static const uint8_t  BR_USB            = 12;     // USB-A and side USB-C LED brightness
static const uint8_t  BR_CHG            = 12;     // external-power indicator LED brightness
static const uint8_t  BR_CATM           = 12;     // CatM status LED brightness
static const uint32_t PWRLOG_MS         = 120000UL; // power-log sample period (2 min)
static const uint16_t PWRLOG_CAP        = 450;      // ring → ~15 h window at 2 min

// ── CatM network robustness (our server is IPv4-only) ─────────────────────────
// povo's LTE-M defaults to an IPv4v6 bearer and often hands out an IPv6-only
// address, which can't reach the IPv4-only server → SHCONN fails (red). We pin
// the PDP to IPv4 and, on repeated failure, force a re-attach to renegotiate it.
static const uint8_t  CATM_FAIL_REATTACH = 3;       // consecutive send fails → CFUN re-attach
static const uint32_t CATM_FAIL_RETRY_MS = 60000UL; // after a failed send, retry this soon

// ── Store-and-forward track queue ─────────────────────────────────────────────
static const uint16_t TRACK_QUEUE_CAP    = 1000;    // RAM ring of unsent points (~20 KB)
static const uint8_t  TRACK_BATCH        = 8;       // points per catch-up POST (body < BODYLEN 1024)
static const uint8_t  TRACK_FLUSH_BATCHES = 3;      // max batches pushed per successful live send

// ── Adaptive beaconing (SmartBeaconing + decay) ───────────────────────────────
// Tuned for WALKING-primary use, occasional bicycle, at most a 原付/moped later —
// NOT cars. So the "fast rate" is reached by brisk-cycling speed (~25 km/h), not
// the 60 km/h car default.  GPS is always sampled; only the *send* cadence varies.
//   moving : interval interpolates linearly SLOW↔FAST between LOW↔HIGH speed
//   stopped: decay — interval doubles each beacon, START→MAX (heartbeat)
//   plus immediate sends on "moved > MOVE_THRESH" and on sharp turns (corner peg)
static const float    SB_LOW_SPEED_KMH  = 2.5f;     // ≤ this = stopped → decay
static const float    SB_HIGH_SPEED_KMH = 25.0f;    // ≥ this = fastest rate (cycle/原付)
static const uint32_t SB_FAST_MS        = 45000UL;  // 45 s at/above high speed
static const uint32_t SB_SLOW_MS        = 180000UL; // 3 min just-moving backstop
static const uint32_t DECAY_START_MS    = 300000UL; // 5 min: first stopped interval
static const uint32_t DECAY_MAX_MS      = 1800000UL;// 30 min: stopped cap (5→10→20→30)
static const float    MOVE_THRESH_M     = 40.0f;    // displacement from anchor → "moved"
static const float    HDOP_MAX          = 2.5f;     // trust position only if HDOP better
static const uint8_t  SAT_MIN           = 4;        // and ≥ this many satellites
static const float    TURN_MIN_SPEED    = 8.0f;     // km/h: corner-peg only above this
static const float    TURN_MIN_DEG      = 25.0f;    // base turn angle to beacon
static const float    TURN_SLOPE        = 120.0f;   // turnThresh = TURN_MIN_DEG + SLOPE/speed
static const uint32_t TURN_MIN_MS       = 15000UL;  // min gap between turn beacons
static const uint32_t MIN_TX_GAP_MS     = 30000UL;  // hard floor between ANY two beacons
                                                    // (a send takes ~20 s; also caps the
                                                    // "moved" trigger at cycling/原付 speed)

// ═══════════════════════════════════════════════════════════════════════════
// GPS state machine
// ═══════════════════════════════════════════════════════════════════════════

enum GpsState : uint8_t {
    GS_DETECTING,   // startup, waiting for any NMEA data
    GS_NO_MODULE,   // timeout — no data received, leave LED off
    GS_INIT_OK,     // first valid sentence — brief blue flash
    GS_SEARCHING,   // module active, no valid fix yet  → yellow
    GS_FIX_GOOD,    // valid fix with HDOP < threshold   → green
    GS_INIT_FAIL    // data received but all checksums bad → red
};

// ═══════════════════════════════════════════════════════════════════════════
// Global state
// ═══════════════════════════════════════════════════════════════════════════

static TinyGPSPlus    gps;
static HardwareSerial gpsSerial(1);   // UART1

// 统一位置源：配置A 由 TinyGPS++(gps) 喂，配置B 由 CGNSINF 轮询喂。
// buildTrackPoint / beaconDue / recordAnchor 一律通过下面的 fixXxx() 访问器读它，
// 两种来源对上层完全透明。
struct LiveFix {
    bool     valid       = false;   // 有有效定位
    double   lat = 0, lon = 0;
    float    altM = 0, spdKmh = 0, courseDeg = -1, hdop = 25.5f;
    bool     courseValid = false;
    uint8_t  sats        = 0;
    uint32_t tMs         = 0;       // 上次更新 millis（判新鲜度）
};
static LiveFix   liveFix;
#if GNSS_TIMESHARE
static bool      gnssTracking  = false;   // 配置B：模块当前是否处于 GNSS 跟踪模式
static uint32_t  tLastGnssPoll = 0;       // CGNSINF 轮询计时

// ── LCD 显示（Unit LCD 1.14"，PORT.C/G1·G2，I2C 端口1，竖屏黑底浅字）──────────────
static M5UnitLCD lcd;
static M5Canvas  lcdCanvas(&lcd);         // 离屏画布，整帧一次推送 → 无闪烁
static bool      lcdOK        = false;    // LCD 初始化成功
static bool      lcdCanvasOK  = false;    // 画布分配成功（否则直接画到屏，省内存）
static bool      displayOn    = false;    // 当前是否亮屏
static uint32_t  tDisplayOff  = 0;        // 自动息屏时刻（millis）
static uint32_t  tLastLcdDraw = 0;        // 上次重绘时刻
static const uint32_t DISPLAY_ON_MS = 30000;   // 短按/开机亮屏时长（30s）
static const uint32_t LCD_DRAW_MS   = 1000;    // 亮屏时状态重绘周期
static const uint8_t  LCD_BRIGHTNESS = 110;    // 亮屏亮度（0-255）
#endif

// ── GNSS 信号诊断（从 GSV/TXT 自解析，由电量日志按采样间隔快照）─────────────────
// 槽位: 0=GPS 1=GLONASS 2=BDS北斗 3=Galileo 4=QZSS 5=SBAS/其它
static uint8_t gnssInView[6] = {0};  // 各系统可见卫星数（最近一个完整 GSV 周期）
static uint8_t gnssCN0[6]    = {0};  // 各系统最强 CN0/信噪比 dBHz（最近一个周期）
static uint8_t gnssAccCN0[6] = {0};  // 当前进行中周期内的 CN0 累加器
static uint8_t gnssAnt       = 0;    // 天线: 0 未知 / 1 OK / 2 开路 / 3 短路
static char    nmeaLine[100];        // NMEA 整行装配缓冲（最长 82 字符）
static uint8_t nmeaLen       = 0;

static GpsState  gpsState      = GS_DETECTING;
static bool      usbEnabled    = false;
static int       batPct        = 0;
static uint16_t  batMv         = 0;       // 最近一次电池电压(mV)，供 LCD 显示
static bool      hasExtPower   = false;   // external power present (REG_PSUP != 0)
static bool      batValid      = false;   // true once first non-zero battery voltage read
static bool      blinkPhase    = false;

static uint32_t  tBoot         = 0;
static uint32_t  tGpsInitData  = 0;    // when first valid NMEA sentence arrived
static uint32_t  tLastBatRead  = 0;
static uint32_t  tLastLed      = 0;
static uint32_t  tLastUsbChk   = 0;
static uint32_t  tLastBtn      = 0;
static uint32_t  tLastBlink    = 0;
static uint32_t  tLastGpsLog   = 0;

static uint8_t   clickCount    = 0;
static uint32_t  tFirstClick   = 0;
static bool      prevBtnOK     = false;   // big top button (BTN_OK) previous state
static bool      prevBtnSel    = false;   // small round button (GPIO11) previous state
static uint32_t  tSelPress     = 0;       // when the yellow button was last pressed
static bool      selLongFired  = false;   // long-press action already fired this hold
static bool      manualSendReq = false;   // top-button short press → upload current GPS position now
static uint32_t  tOkPress      = 0;       // when the top button (BTN_OK) was last pressed
static bool      okLongFired   = false;   // top-button long-press action already fired this hold
static bool      forceSendReq  = false;   // top-button long-press → bench test: force CatM upload, ignore GPS fix

static uint32_t  ledCache[8];          // tracks last written color per LED

// ── CatM state ────────────────────────────────────────────────────────────────
enum CatmState : uint8_t {
    CM_OFF,       // not initialized
    CM_INIT,      // module initializing (yellow)
    CM_READY,     // network ready, idle (blue)
    CM_SENDING,   // HTTP POST in progress (white)
    CM_OK,        // last POST succeeded (green)
    CM_ERR        // last operation failed (red)
};
static CatmState catmState      = CM_OFF;
static bool      catmReady      = false;
static uint8_t   catmFailStreak = 0;       // consecutive sendGpsData failures (0 on success)
static bool      catmTimeSynced = false;   // true after first successful time sync
static bool      edrxChecked    = false;   // one-shot: read granted eDRX cycle after attach
static uint32_t  tLastSend      = 0;
static uint32_t  tLastCatmRecover = 0;   // 红灯且无 GPS 定位时的恢复重试计时器（与 beacon 节奏的 tLastSend 互不干扰）
static uint32_t  tLastSyncAttempt = 0;
static uint32_t  tLastPwrLog    = 0;

// ── Adaptive beaconing state ──────────────────────────────────────────────────
static double    lastTxLat      = 0.0;    // anchor = position of last beacon
static double    lastTxLon      = 0.0;
static double    lastTxCourse   = -1.0;   // course at last beacon (-1 = unknown)
static bool      haveAnchor     = false;  // false until the first beacon
static uint32_t  decayInterval  = DECAY_START_MS;  // current stopped interval (grows)

// ── Store-and-forward track queue ─────────────────────────────────────────────
// When a beacon can't be sent (no network / IPv6-only bearer), the point is kept
// here and re-sent later (batched, oldest first) once a send next succeeds — so a
// dead-cell / wrong-PDP gap doesn't punch a hole in the track. Lives in RAM
// (lost on a full reboot, which is acceptable — this is short-term gap insurance).
// Ring buffer: once full it overwrites the oldest point (keep the newest track).
struct __attribute__((packed)) TrackPoint {
    uint32_t ts;       // epoch seconds (capture time; 0 = RTC unsynced)
    int32_t  lat;      // degrees × 1e7
    int32_t  lon;      // degrees × 1e7
    int16_t  alt;      // meters
    uint8_t  spd;      // km/h (capped 255)
    uint8_t  sat;
    uint8_t  hdop;     // HDOP × 10 (capped 25.5)
    uint16_t bat_mv;
    uint8_t  bat_pct;
};                     // 20 bytes/point
static TrackPoint trackQueue[TRACK_QUEUE_CAP];
static uint16_t   trackHead  = 0;     // next write slot
static uint16_t   trackCount = 0;     // queued points (0..CAP)

// ── Power log (battery voltage/current history) ───────────────────────────────
// Stored in RTC slow memory so it survives software/brownout/watchdog resets
// (NOT a full power-off — when REG 0xE0 cuts power the RTC RAM is lost, which is
// itself the signal "the device powered off").  Ring buffer: once full it
// overwrites the oldest entry, so it can never overflow RTC memory.  Pull-based:
// dump/clear over the USB serial console with the `log` / `logclear` commands.
#define PWRLOG_MAGIC 0x50574C33UL    // 'PWL3' — bump to invalidate old layout
struct __attribute__((packed)) PwrLogEntry {
    uint32_t ts;     // epoch seconds (0 = RTC not yet synced)
    uint16_t mv;     // battery voltage (mV)
    int16_t  ma;     // battery current (mA, signed: - = charging/into batt, + = discharge/draw)
    uint8_t  pct;    // battery percent
    uint8_t  flags;  // b0 extPwr  b1 charging  b2 gpsFix ; b4-7 catmState
    uint8_t  cn0;    // 最强卫星 CN0/信噪比 (dBHz)，0 = 无星。信号强弱的关键指标
    uint8_t  gnss;   // b0 GPS b1 GLONASS b2 BDS b3 Galileo b4 QZSS b5 SBAS/其它 ; b6-7 天线(0 未知/1 OK/2 开路/3 短路)
    uint8_t  sats;   // 各星座可见卫星总数
};
RTC_NOINIT_ATTR static uint32_t    pwrlogMagic;
RTC_NOINIT_ATTR static uint16_t    pwrlogHead;   // next write slot (0..CAP-1)
RTC_NOINIT_ATTR static uint16_t    pwrlogCount;  // valid entries (0..CAP)
RTC_NOINIT_ATTR static PwrLogEntry pwrlogBuf[PWRLOG_CAP];

// ═══════════════════════════════════════════════════════════════════════════
// PowerHub I2C helpers
// ═══════════════════════════════════════════════════════════════════════════

static void phRd(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(PH_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)PH_ADDR, n);
    for (uint8_t i = 0; i < n; i++)
        buf[i] = Wire.available() ? Wire.read() : 0;
}
static void phWr(uint8_t reg, const uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(PH_ADDR);
    Wire.write(reg);
    Wire.write(buf, n);
    Wire.endTransmission();
}
static uint8_t phRd8(uint8_t reg)            { uint8_t v; phRd(reg,&v,1); return v; }
static void    phWr8(uint8_t reg, uint8_t v) { phWr(reg,&v,1); }

static bool phInit() {
    Wire.end();                   // reset any previous state
    Wire.begin(SYS_SDA, SYS_SCL);
    Wire.setClock(100000);        // start at 100 kHz — more forgiving during boot

    // STM32 on PowerHub needs ~500 ms after power-on to initialise its I2C.
    // (The official IDF firmware has vTaskDelay(500) here for the same reason.)
    delay(700);

    for (int attempt = 0; attempt < 8; attempt++) {
        Wire.beginTransmission(PH_ADDR);
        if (Wire.endTransmission() == 0) {
            Wire.setClock(400000);    // switch to 400 kHz once confirmed
            Serial.printf("[PH] Found at 0x%02X (attempt %d)\n", PH_ADDR, attempt + 1);
            return true;
        }
        delay(150);
    }

    // Not found — scan the entire bus so the user can see what is actually there
    Serial.println("[PH] Scanning I2C bus for diagnostics:");
    bool any = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found: 0x%02X\n", a);
            any = true;
        }
    }
    if (!any) Serial.println("  (no devices found — check SDA/SCL wiring)");
    return false;
}

static void phPower(uint8_t port, bool on) {
    phWr8(REG_PWR + port, on ? 1 : 0);
}

// Only issues I2C write when color actually changes (cache-guarded)
static void phLedSet(uint8_t idx, uint32_t rgb) {
    if (idx >= 8 || ledCache[idx] == rgb) return;
    ledCache[idx] = rgb;
    uint8_t b[3] = {(uint8_t)rgb, (uint8_t)(rgb>>8), (uint8_t)(rgb>>16)};
    phWr(LED_CR[idx], b, 3);
}

static void phLedBright(uint8_t idx, uint8_t br) {
    if (idx < 8) phWr8(LED_BR[idx], br);
}

static uint16_t phVolt(uint8_t mon) {
    uint8_t b[2]; phRd(VA_VR[mon],b,2); return (uint16_t)(b[1]<<8)|b[0];
}
static int16_t phCurr(uint8_t mon) {
    uint8_t b[2]; phRd(VA_IR[mon],b,2); return (int16_t)((b[1]<<8)|b[0]);
}
static bool phBtn(uint8_t bitPos) {
    return (phRd8(REG_BTN) >> bitPos) & 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Battery: 2S LiPo voltage → percentage
// ═══════════════════════════════════════════════════════════════════════════

static int voltToPercent(uint16_t mv) {
    // 2S LiPo discharge curve. This PowerHub board's charge IC terminates at
    // ~8.1V (≈4.05V/cell) by design — it never reaches the 8.4V cell maximum,
    // to extend battery life. So we treat 8100mV and above as 100% and rescale
    // the rest of the curve accordingly. 6.8V (≈3.4V/cell) is the cut-off (0%).
    static const uint16_t V[] = {6800,7000,7200,7400,7600,7800,8000,8100};
    static const int      P[] = {   0,  10,  20,  35,  55,  75,  92, 100};
    const int N = 8;
    if (mv <= V[0])   return 0;
    if (mv >= V[N-1]) return 100;
    for (int i = 1; i < N; i++) {
        if (mv <= V[i])
            return P[i-1] + (int)((int32_t)(mv-V[i-1])*(P[i]-P[i-1])/(V[i]-V[i-1]));
    }
    return 100;
}

static void readBattery() {
    uint16_t mv  = phVolt(VM_BAT);
    uint8_t  chg = phRd8(REG_CHG);
    uint8_t  pwr = phRd8(REG_PSUP);
    batMv = mv;                 // 供 LCD 显示（INA226 就绪前为 0）
    // INA226 needs ~200 ms after PC_VAMETER is enabled to produce the first
    // reading; before that phVolt() returns 0.  Only latch valid when mv>1V.
    if (mv > 1000) {
        batPct   = voltToPercent(mv);
        batValid = true;
    }
    hasExtPower = (pwr != 0);
    Serial.printf("[BAT] %u mV  %d%%  chg=0x%02X pwr=0x%02X extPwr=%d valid=%d\n",
                  mv, batPct, chg, pwr, hasExtPower, batValid);
}

// ═══════════════════════════════════════════════════════════════════════════
// GNSS 信号诊断解析（解析一整行 NMEA）
// 只看 GSV（星座/CN0/可见星数）与 TXT（天线状态）；不碰 TinyGPS++（它另收同样字节）。
// ═══════════════════════════════════════════════════════════════════════════
static void gnssDiagLine(const char* s) {
    if (s[0] != '$') return;
    const char* ty = s + 3;                       // 句型在 talker(2) 之后
    if (ty[0]=='T' && ty[1]=='X' && ty[2]=='T') { // 天线状态 $xxTXT,...,ANTENNA OK/OPEN/SHORT
        if      (strstr(s, "ANTENNA OK"))    gnssAnt = 1;
        else if (strstr(s, "ANTENNA OPEN"))  gnssAnt = 2;
        else if (strstr(s, "ANTENNA SHORT")) gnssAnt = 3;
        return;
    }
    if (!(ty[0]=='G' && ty[1]=='S' && ty[2]=='V')) return;
    const char* tk = s + 1;                       // talker 两字符
    int slot;
    if      (tk[0]=='G' && tk[1]=='P') slot = 0;  // GPS（QZSS 旧固件也可能混在此，按 PRN 另判）
    else if (tk[0]=='G' && tk[1]=='L') slot = 1;  // GLONASS
    else if ((tk[0]=='B'&&tk[1]=='D') || (tk[0]=='G'&&tk[1]=='B')) slot = 2;  // BDS
    else if (tk[0]=='G' && tk[1]=='A') slot = 3;  // Galileo
    else if (tk[0]=='G' && tk[1]=='Q') slot = 4;  // QZSS
    else slot = 5;                                 // SBAS/其它

    // 收集逗号字段起点。GSV: f1=总条数 f2=本条序号 f3=可见星数，之后每 4 个 {prn,elev,az,snr}
    const char* f[24];
    int nf = 0;
    f[nf++] = s;
    for (const char* p = s; *p && *p!='*' && *p!='\r' && *p!='\n' && nf < 24; p++)
        if (*p == ',') f[nf++] = p + 1;
    if (nf < 4) return;
    int totalMsgs = atoi(f[1]);
    int msgNum    = atoi(f[2]);
    int numSV     = atoi(f[3]);
    if (msgNum <= 0) return;
    if (msgNum == 1) gnssAccCN0[slot] = 0;        // 本系统新周期开始，清累加器
    gnssInView[slot] = (uint8_t)numSV;            // 一个周期内各条相同
    for (int b = 4; b + 3 < nf; b += 4) {         // 逐颗卫星块
        int prn = atoi(f[b]);
        int snr = atoi(f[b + 3]);                 // 空字段 atoi=0
        if (snr > gnssAccCN0[slot]) gnssAccCN0[slot] = (uint8_t)snr;
        if (slot == 0 && prn >= 193 && prn <= 202 && snr > 0) {  // QZSS 混在 GPGSV 的兜底
            if (gnssInView[4] == 0) gnssInView[4] = 1;
            if (snr > gnssCN0[4]) gnssCN0[4] = (uint8_t)snr;
        }
    }
    if (msgNum >= totalMsgs) gnssCN0[slot] = gnssAccCN0[slot];   // 周期结束，提交本系统最强 CN0
}

// ═══════════════════════════════════════════════════════════════════════════
// Power log (RTC ring buffer) + USB serial command console
// ═══════════════════════════════════════════════════════════════════════════

// Validate RTC contents; re-init on cold boot (NOINIT garbage) or corruption.
static void pwrlogInit() {
    if (pwrlogMagic != PWRLOG_MAGIC || pwrlogHead >= PWRLOG_CAP
            || pwrlogCount > PWRLOG_CAP) {
        pwrlogMagic = PWRLOG_MAGIC;
        pwrlogHead  = 0;
        pwrlogCount = 0;
    }
}

static void pwrlogClear() {
    pwrlogMagic = PWRLOG_MAGIC;
    pwrlogHead  = 0;
    pwrlogCount = 0;
}

// Append one sample. Ring buffer: overwrites the oldest entry once full, so it
// can never overflow RTC memory.
static void pwrlogAppend() {
    PwrLogEntry e;
    time_t now = time(nullptr);
    e.ts  = (now > 1735689600L) ? (uint32_t)now : 0;   // >2025-01-01 → RTC synced
    e.mv  = phVolt(VM_BAT);
    e.ma  = phCurr(VM_BAT);
    e.pct = (uint8_t)batPct;
    e.flags = (hasExtPower ? 0x01 : 0)
            | (phRd8(REG_CHG) ? 0x02 : 0)
            | (gpsState == GS_FIX_GOOD ? 0x04 : 0)
            | ((uint8_t)catmState << 4);
    // GNSS 信号快照：星座位掩码 + 最强 CN0 + 可见星总数 + 天线
    uint8_t mask = 0, sats = 0, cn0 = 0;
    for (int k = 0; k < 6; k++) {
        if (gnssInView[k]) { mask |= (1 << k); sats += gnssInView[k]; }
        if (gnssCN0[k] > cn0) cn0 = gnssCN0[k];
    }
    e.cn0  = cn0;
    e.gnss = mask | (uint8_t)((gnssAnt & 0x03) << 6);
    e.sats = sats;
    pwrlogBuf[pwrlogHead] = e;
    pwrlogHead = (pwrlogHead + 1) % PWRLOG_CAP;
    if (pwrlogCount < PWRLOG_CAP) pwrlogCount++;
}

// Dump the whole log as CSV (oldest → newest) plus a battery-only current summary.
static void pwrlogDump() {
    Serial.printf("[PWRLOG] %u entries (cap %u, every %lus). "
                  "ma: -=charging(into batt), +=discharge(draw). on battery(ext=0) ma=draw\n",
                  pwrlogCount, PWRLOG_CAP, (unsigned long)(PWRLOG_MS / 1000));
    Serial.println("idx,ts,mv,ma,pct,ext,chg,fix,catm,cn0,gnss,sats");
    uint16_t start = (pwrlogCount == PWRLOG_CAP) ? pwrlogHead : 0;
    long sumDisc = 0; int nDisc = 0; int16_t minMa = 32767, maxMa = -32768;
    for (uint16_t i = 0; i < pwrlogCount; i++) {
        PwrLogEntry &e = pwrlogBuf[(start + i) % PWRLOG_CAP];
        uint8_t ext = e.flags & 1, chg = (e.flags >> 1) & 1, fix = (e.flags >> 2) & 1;
        uint8_t catm = (e.flags >> 4) & 0x0F;
        Serial.printf("%u,%lu,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u\n",
                      i, (unsigned long)e.ts, e.mv, e.ma, e.pct, ext, chg, fix, catm,
                      e.cn0, e.gnss, e.sats);
        if (!ext) { sumDisc += e.ma; nDisc++;
                    if (e.ma < minMa) minMa = e.ma; if (e.ma > maxMa) maxMa = e.ma; }
        if ((i & 0x1F) == 0x1F) Serial.flush();   // help a slow CDC reader keep up
    }
    if (nDisc > 0)
        Serial.printf("[PWRLOG] battery-only: %d samples, avg ma=%ld (min=%d max=%d)\n",
                      nDisc, sumDisc / nDisc, minMa, maxMa);
    else
        Serial.println("[PWRLOG] no battery-only samples yet (always on external power)");
    Serial.println("[PWRLOG] end");
    Serial.flush();
}

static void gnssSwitchTest();   // 前置声明：定义在 catmCmd 之后
static void atScan();           // 前置声明：PORT.A 波特率扫描诊断
static String catmCmd(const String& cmd, unsigned long timeout);  // 前置声明：供 checkSerialCommands 原始 AT 透传
#if GNSS_TIMESHARE
static void gnssNmeaTest();     // 前置声明：配置B NMEA(CGNSTST)采样诊断
#endif

// Non-blocking single-line command reader on the USB serial console:
//   log  = dump power log | logclear = erase | gnsstest = 分时切换测速 | help
static void checkSerialCommands() {
    static char buf[80];
    static uint8_t len = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (len == 0) continue;
            buf[len] = 0; len = 0;
            // 原始 AT 透传：以 "at"/"AT" 开头的整行（保留大小写）直接转发给模组，
            // 打印应答。用于现场对锁死/异常的 SIM7080G 逐条试探与恢复实验。
            if ((buf[0] == 'a' || buf[0] == 'A') && (buf[1] == 't' || buf[1] == 'T')) {
                Serial.printf("[AT>] %s\n", buf);
                Serial.printf("[AT<] %s\n", catmCmd(String(buf), 16000).c_str());
                continue;
            }
            for (char *p = buf; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
            if      (!strcmp(buf, "log"))      pwrlogDump();
            else if (!strcmp(buf, "logclear")) { pwrlogClear(); Serial.println("[PWRLOG] cleared"); }
            else if (!strcmp(buf, "sendtest"))  { forceSendReq = true; Serial.println("[CMD] 强制发包"); }
            else if (!strcmp(buf, "gnsstest")) gnssSwitchTest();
            else if (!strcmp(buf, "atscan"))   atScan();
            else if (!strcmp(buf, "help"))     Serial.println("[CMD] log | logclear | sendtest | at<cmd> | gnsstest | atscan | help");
            else Serial.printf("[CMD] unknown: '%s' (try: help)\n", buf);
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            len = 0;   // overflow → discard the line
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// LED refresh functions
// ═══════════════════════════════════════════════════════════════════════════

static void refreshGpsLed() {
    uint32_t c;
    switch (gpsState) {
        case GS_DETECTING:  c = C_OFF;    break;
        case GS_NO_MODULE:  c = C_OFF;    break;
        case GS_INIT_OK:    c = C_BLUE;   break;
        case GS_SEARCHING:  c = C_YELLOW; break;
        case GS_FIX_GOOD:   c = C_GREEN;  break;
        case GS_INIT_FAIL:  c = C_RED;    break;
        default:            c = C_OFF;
    }
    phLedSet(GPS_LED, c);
}

// Battery LED — always shows battery level.  Blue = reading not yet available.
static void refreshBatLed() {
    uint32_t c;
    if (!batValid) {
        c = C_BLUE;                        // INA226 not ready yet at startup
    } else if (batPct < 10) {
        c = blinkPhase ? C_RED : C_OFF;   // critical blink
    } else if (batPct < 30) {
        c = C_RED;
    } else if (batPct < 60) {
        c = C_YELLOW;
    } else {
        c = C_GREEN;
    }
    phLedSet(BAT_LED, c);
}

// External-power indicator — LED adjacent to small round button.
// Blue = any external power present (charging OR data cable).  Off = on battery.
static void refreshPowerLed() {
    phLedSet(CHG_LED, hasExtPower ? C_BLUE : C_OFF);
}

// CatM status LED (PORT.A adjacent)
static void refreshCatmLed() {
    uint32_t c;
    switch (catmState) {
        case CM_OFF:     c = C_OFF;    break;
        case CM_INIT:    c = C_YELLOW; break;
        case CM_READY:   c = C_BLUE;   break;
        case CM_SENDING: c = C_WHITE;  break;
        case CM_OK:      c = C_GREEN;  break;
        case CM_ERR:     c = C_RED;    break;
        default:         c = C_OFF;
    }
    phLedSet(CATM_LED, c);
}

// ═══════════════════════════════════════════════════════════════════════════
// CatM AT command layer (SIM7080G via Serial2 on PORT.A)
// Response lines are collapsed to pipe-separated tokens, e.g.  "|OK|"
// ═══════════════════════════════════════════════════════════════════════════

static String catmCmd(const String& cmd, unsigned long timeout) {
    String ret;
    Serial2.flush();
    while (Serial2.available()) Serial2.read();
    Serial2.println(cmd);

    unsigned long t0 = millis();
    while (millis() - t0 < timeout) {
        if (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\r' || c == '\n') c = '|';
            ret += c;
            while (ret.indexOf("||") >= 0) ret.replace("||", "|");
        }
        // Early-exit conditions
        // Verbose error (CMEE=2) terminates any command — recognise it so we
        // don't wait out the full timeout on "+CME ERROR: ..." responses.
        if (ret.indexOf("+CME ERROR") >= 0 && ret.endsWith("|")
                && !cmd.startsWith("AT+CREBOOT") && !cmd.startsWith("AT+CFUN=1,1")) {
            break;
        }
        if (cmd.startsWith("AT+CREBOOT") || cmd.startsWith("AT+CFUN=1,1")) {
            if (ret.indexOf("|RDY|") >= 0) {
                uint32_t r = millis();
                while (millis() - r < 3000) {
                    if (Serial2.available()) {
                        char c = Serial2.read();
                        if (c == '\r' || c == '\n') c = '|';
                        ret += c;
                        while (ret.indexOf("||") >= 0) ret.replace("||", "|");
                    }
                    delay(1);
                }
                break;
            }
            if (ret.indexOf("|ERROR|") >= 0) break;
        } else if (cmd.startsWith("AT+SHREQ=")) {
            if (ret.indexOf("+SHREQ") >= 0 && ret.endsWith("|")) break;
        } else if (cmd.startsWith("AT+SHREAD=")) {
            if (ret.indexOf("+SHREAD:") >= 0) {
                int ci = cmd.lastIndexOf(',');
                int reqLen = (ci >= 0) ? cmd.substring(ci + 1).toInt() : 0;
                if ((int)ret.length() >= 14 + (int)String(reqLen).length() + reqLen
                        && ret.endsWith("|")) break;
            }
        } else if (cmd.startsWith("AT+CNACT=")) {
            if ((ret.indexOf("+APP PDP") >= 0 || ret.indexOf("+CNACT:") >= 0)
                    && ret.endsWith("|")) break;
            if (ret.indexOf("|ERROR|") >= 0) break;
        } else if (cmd == "AT+CNTP") {
            // NTP sync URC: +CNTP: 1 (success) or +CNTP: 0 (fail)
            if (ret.indexOf("+CNTP:") >= 0 && ret.endsWith("|")) break;
        } else if (cmd.startsWith("AT+SHBOD=")) {
            if (ret.indexOf(">") >= 0) break;
            if (ret.indexOf("|ERROR|") >= 0) break;
        } else if (cmd.startsWith("AT+CAOPEN=")) {
            // URC "+CAOPEN: <cid>,<result>" arrives after the OK line
            if (ret.indexOf("+CAOPEN:") >= 0 && ret.endsWith("|")) break;
            if (ret.indexOf("|ERROR|") >= 0) break;
        } else if (cmd.startsWith("AT+CDNSGIP=")) {
            // URC "+CDNSGIP: 1,\"host\",\"ip\"" (or 0,err) after the OK line
            if (ret.indexOf("+CDNSGIP:") >= 0 && ret.endsWith("|")) break;
            if (ret.indexOf("|ERROR|") >= 0) break;
        } else {
            if (ret.endsWith("|OK|") || ret.endsWith("|ERROR|")) break;
        }
        delay(1);
    }
    Serial.printf("[CM] %s -> %s (%ums)\n",
                  cmd.c_str(), ret.c_str(), (unsigned)(millis() - t0));
    return ret;
}

// True if the CNACT? response shows cid 0 holding a usable IPv4 address
// (dotted-decimal, not an IPv6 "::" form, and not the 0.0.0.0 placeholder).
static bool catmHasIPv4(const String& cnact) {
    int p = cnact.indexOf("+CNACT: 0,1,\"");
    if (p < 0) return false;
    int s = p + 13;                       // first char of the address
    int q = cnact.indexOf('"', s);
    if (q < 0) return false;
    String ip = cnact.substring(s, q);
    return ip.indexOf('.') >= 0 && ip.indexOf(':') < 0 && ip != "0.0.0.0";
}

// Pin the default PDP context to IPv4-only and re-attach so the bearer is
// renegotiated. PDP type only takes effect at attach, so this must run detached
// (CFUN=0). Used at init and as the escalation step after repeated send fails —
// it is roughly what a cell handover would force, which is why it can recover a
// bearer that got stuck IPv6-only out in the field.
static void catmForceIPv4() {
    Serial.println("[CM] forcing IPv4-only PDP (CGDCONT \"IP\" + re-attach)");
    catmCmd("AT+CFUN=0", 10000);
    delay(500);
    catmCmd("AT+CGDCONT=1,\"IP\",\"" CATM_APN "\"", 3000);
    catmCmd("AT+CFUN=1", 10000);
    delay(2000);
    for (int i = 0; i < 10; i++) {        // wait for re-registration (CEREG 1/5)
        String r = catmCmd("AT+CEREG?", 2000);
        int c = r.indexOf(',');
        if (c >= 0 && (r[c+1] == '1' || r[c+1] == '5')) {
            Serial.printf("[CM] re-registered (iter=%d): %s\n", i, r.c_str());
            break;
        }
        if (i < 9) delay(2000);
    }
    catmCmd("AT+CNCFG=0,1,\"" CATM_APN "\"", 3000);   // APP PDP: IPv4 too
}

// Probe module, configure LTE-M mode and APN, verify SIM.
// Does NOT wait for network registration — that is catmCheckNet's job.
static bool catmInit() {
    Serial.println("[CM] Init...");
    catmState = CM_INIT;
    refreshCatmLed();

    // ESP32 may reset while CatM stays powered and mid-HTTPS session.
    // Flush buffer and send SHDISC to break any lingering HTTP connection
    // before probing with AT, so the module returns to command mode.
    Serial2.flush();
    while (Serial2.available()) Serial2.read();
    Serial2.println("AT+SHDISC");
    delay(1500);
    while (Serial2.available()) Serial2.read();
    Serial.println("[CM] pre-cleanup done, probing...");

    // 冷启动/热插拔后模块（尤其带 GNSS 的二合一）要十几秒才 AT 就绪。
    // 先在 ~25s 窗口内耐心反复探 AT，给它启动时间；模块已就绪时首发即过、零等待。
    // 实在不应答才 CREBOOT（避免对正在启动的模块再来一发重启，反而更慢）。
    String ret;
    bool alive = false;
    for (int i = 0; i < 25; i++) {
        ret = catmCmd("AT", 1000);
        if (ret.indexOf("|OK|") >= 0) { alive = true; break; }
        if (i == 2) Serial.println("[CM] 等模块启动…（冷启/热插拔可能要十几秒）");
        delay(300);
    }
    if (!alive) {
        Serial.println("[CM] No AT response after ~25s — trying CREBOOT");
        ret = catmCmd("AT+CREBOOT", 16000);
        if (ret.indexOf("|RDY|") == -1) {
            Serial.println("[CM] CREBOOT no RDY — trying CFUN=1,1");
            ret = catmCmd("AT+CFUN=1,1", 16000);
            if (ret.indexOf("|RDY|") == -1) {
                Serial.println("[CM] Init FAIL: module not responding");
                catmState = CM_ERR;
                refreshCatmLed();
                return false;
            }
        }
        Serial.println("[CM] Module rebooted OK");
    } else {
        Serial.println("[CM] AT alive");
    }

    catmCmd("ATE0", 2000);
    catmCmd("AT+CSCLK=0", 2000);
    catmCmd("AT+CMEE=2", 2000);   // verbose error reporting (+CME ERROR: text)

    // ★ Guide §8.5: deterministically restore full functionality.
    // A prior CFUN=0 (parking, or a catmForceIPv4 interrupted by an ESP32 reset
    // between its CFUN=0 and CFUN=1) leaves the radio off and the SIM NOT READY,
    // yet the AT probe above still succeeds because UART stays alive at CFUN=0.
    // If we relied on catmForceIPv4 below to send CFUN=1, that protection would
    // be incidental — change that path and the SIM-not-ready boot loop returns.
    // So force CFUN=1 unconditionally HERE, independent of anything downstream.
    catmCmd("AT+CFUN=1", 10000);
    delay(1500);                  // let the SIM finish becoming ready

    // Log firmware version once
    Serial.printf("[CM] ATI=%s\n", catmCmd("ATI", 2000).c_str());

    // LTE-M only (CNMP=38, CMNB=1)
    bool needCnmp = (catmCmd("AT+CNMP?", 2000).indexOf("+CNMP: 38") == -1);
    bool needCmnb = (catmCmd("AT+CMNB?", 2000).indexOf("+CMNB: 1")  == -1);
    if (needCnmp) {
        Serial.println("[CM] Setting LTE-M only (CNMP=38)");
        catmCmd("AT+CNMP=38", 3000);
        delay(1000);
    }
    if (needCmnb) {
        Serial.println("[CM] Setting Cat-M band (CMNB=1)");
        catmCmd("AT+CMNB=1", 3000);
        delay(4000);
    }

    // APN + pin IPv4-only PDP (server is IPv4-only; an IPv6-only bearer = red).
    Serial.printf("[CM] APN=%s\n", CATM_APN);
    catmForceIPv4();
    catmCmd("AT+CEREG=0", 2000);
    catmCmd("AT+CPSMS=0", 2000);   // disable PSM deep sleep (DTR not wired → would sleep-lock)

    // Guide §8.3: enable eDRX. Extends the paging cycle without sleeping the UART;
    // zero downside for an uplink-only tracker (we never need to be paged). The
    // network-granted cycle is read back once after attach (CEDRXRDP reads 0 if
    // queried this early), see the one-shot probe in loop().
    catmCmd("AT+CEDRXS=1,4,\"1001\"", 3000);   // Act_type 4 = WB-S1 (Cat-M1); request ~163.84 s

    // SIM check
    for (int i = 0; i < 6; i++) {
        ret = catmCmd("AT+CPIN?", 3000);
        if (ret.indexOf("+CPIN: READY") >= 0) {
            // Log ICCID so user can verify correct SIM
            Serial.printf("[CM] SIM OK  ICCID=%s\n", catmCmd("AT+ICCID", 3000).c_str());
            Serial.println("[CM] Init OK — blue");
            catmState = CM_READY;
            refreshCatmLed();
            return true;
        }
        Serial.printf("[CM] CPIN attempt %d: %s\n", i + 1, ret.c_str());
        delay(1000);
    }

    Serial.println("[CM] Init FAIL: SIM not ready");
    catmState = CM_ERR;
    refreshCatmLed();
    return false;
}

// 切回 LTE 后等模组重新附着到网络再判网。
// 二合一是 GNSS/LTE 分时共用射频：GNSS 跟踪期(CGNSPWR=1)LTE 被挂起，CGNSPWR=0 把
// 射频交还 LTE 后，模组要数秒才能重新搜网+附着。切完射频**立刻**查 CEREG/激活 PDP
// 必然看到“没注册”而误判失败——这是配置B 红灯锁死的根因。轮询 CEREG 直到注册
// (stat=1 home / 5 roaming)或超时；已注册时首查即过、零等待。
#if GNSS_TIMESHARE
static bool catmWaitReg(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    for (;;) {
        String r = catmCmd("AT+CEREG?", 2000);
        int comma = r.indexOf(',');
        if (comma >= 0 && (r[comma + 1] == '1' || r[comma + 1] == '5')) return true;
        if (millis() - t0 >= timeoutMs) return false;
        delay(1500);
    }
}
#endif

// Ensure PDP context is active (has IP address).
// On cold start after power cycle, LTE registration completes quickly but
// bearer (PDP) activation needs a few more seconds to stabilize.
// We wait for CEREG=1/5 before activating to avoid immediate DEACTIVE.
// Returns true ONLY when the PDP is active AND holds a usable IPv4 address.
// An active-but-IPv6-only bearer is treated as failure (can't reach our IPv4
// server) and re-activated; if it still won't yield IPv4 the caller escalates
// to a full re-attach (catmForceIPv4) via the failure counter.
static bool catmCheckNet() {
    for (int attempt = 0; attempt < 3; attempt++) {
        String r = catmCmd("AT+CNACT?", 5000);
        bool active = r.indexOf("+CNACT: 0,1,") >= 0;
        if (active && catmHasIPv4(r)) {
            Serial.printf("[CM] PDP active (IPv4): %s\n", r.c_str());
            return true;
        }
        if (active) {
            // Active but IPv6-only — unusable for our IPv4 server. Drop it.
            Serial.printf("[CM] PDP active but no IPv4: %s — re-activating\n", r.c_str());
            catmCmd("AT+CNACT=0,0", 5000);
            delay(1500);
        } else {
            // Not active — wait for LTE-M registration (stat=1 home, 5 roaming).
            Serial.println("[CM] Waiting for LTE registration...");
            for (int i = 0; i < 8; i++) {
                r = catmCmd("AT+CEREG?", 2000);
                int comma = r.indexOf(',');
                if (comma >= 0 && (r[comma+1] == '1' || r[comma+1] == '5')) break;
                if (i < 7) delay(3000);
            }
        }
        Serial.printf("[CM] CNACT activate attempt %d...\n", attempt + 1);
        catmCmd("AT+CNCFG=0,1,\"" CATM_APN "\"", 3000);   // re-assert IPv4 request
        catmCmd("AT+CNACT=0,1", 15000);
        delay(3000);
    }

    Serial.println("[CM] PDP fail (no usable IPv4) — diagnostics:");
    Serial.printf("[CM]   CNACT=%s\n",   catmCmd("AT+CNACT?",   3000).c_str());
    Serial.printf("[CM]   CEREG=%s\n",   catmCmd("AT+CEREG?",   2000).c_str());
    Serial.printf("[CM]   CGPADDR=%s\n", catmCmd("AT+CGPADDR=1", 3000).c_str());
    return false;
}

// On SHCONN failure, log where the connection breaks (bearer / DNS / TCP) so a
// field failure can be diagnosed from the serial log. TLS itself is opaque
// (SHCONN only returns OK/ERROR), but these layers cover the common causes.
static void catmDiagConn() {
    Serial.println("[CM] === conn diag ===");
    catmCmd("AT+CMEE=2", 3000);   // verbose error reporting
    Serial.printf("[CM]   CNACT? %s\n", catmCmd("AT+CNACT?", 5000).c_str());
    Serial.printf("[CM]   CEREG? %s\n", catmCmd("AT+CEREG?", 3000).c_str());
    Serial.printf("[CM]   DNS    %s\n",
                  catmCmd("AT+CDNSGIP=\"" SERVER_HOST "\"", 10000).c_str());
    catmCmd("AT+CACLOSE=0", 2000);
    Serial.printf("[CM]   TCP    %s\n",
                  catmCmd("AT+CAOPEN=0,0,\"TCP\",\"" SERVER_HOST "\","
                          + String(SERVER_PORT), 20000).c_str());
    catmCmd("AT+CACLOSE=0", 2000);
    Serial.println("[CM] === end diag ===");
}

// Also acts as a network connectivity check after catmInit().
static bool catmSyncTime() {
    Serial.println("[CM] SyncTime: connecting...");
    catmState = CM_SENDING;
    refreshCatmLed();

    if (!catmCheckNet()) {
        Serial.println("[CM] SyncTime: no network — yellow, will retry");
        catmState = CM_INIT;
        refreshCatmLed();
        return false;
    }

    // Follow Cardputer order exactly: SHDISC → NTP → SHCONF → SHCONN
    catmCmd("AT+SHDISC", 3000);
    delay(500);   // 500ms like Cardputer (was 300ms)

    // NTP sync after SHDISC (Cardputer order)
    Serial.println("[CM] NTP sync...");
    catmCmd("AT+CNTP=\"ntp.nict.jp\",36", 3000);
    String ntpRet = catmCmd("AT+CNTP", 15000);
    bool ntpOk = (ntpRet.indexOf("+CNTP: 1") >= 0 || ntpRet.indexOf("+CNTP:1") >= 0);
    Serial.printf("[CM] NTP %s: %s\n", ntpOk ? "OK" : "FAIL", ntpRet.c_str());
    if (ntpOk) Serial.printf("[CM] CCLK=%s\n", catmCmd("AT+CCLK?", 2000).c_str());

    // SHCONF URL with retry (Cardputer style)
    bool urlOk = false;
    for (int attempt = 0; attempt < 2 && !urlOk; attempt++) {
        if (attempt > 0) { catmCmd("AT+SHDISC", 2000); delay(1000); }
        urlOk = catmCmd("AT+SHCONF=\"URL\",\"" SERVER_BASE "\"", 5000).indexOf("|OK|") >= 0;
    }
    if (!urlOk) {
        Serial.println("[CM] SyncTime: SHCONF URL fail");
        catmState = CM_INIT;
        refreshCatmLed();
        return false;
    }
    catmCmd("AT+SHCONF=\"BODYLEN\",1024", 3000);   // 1024 like Cardputer
    catmCmd("AT+SHCONF=\"HEADERLEN\",350", 3000);
    catmCmd("AT+CSSLCFG=\"ignorertctime\",1,1", 3000);
    catmCmd("AT+CSSLCFG=\"sslversion\",1,3", 3000);
    catmCmd("AT+CSSLCFG=\"sni\",1,\"" SERVER_HOST "\"", 3000);
    catmCmd("AT+SHSSL=1,\"\"", 3000);

    if (catmCmd("AT+SHCONN", 10000).indexOf("|OK|") == -1) {
        Serial.println("[CM] SyncTime: SHCONN fail");
        catmCmd("AT+SHDISC", 3000);
        catmDiagConn();
        catmState = CM_INIT;
        refreshCatmLed();
        return false;
    }
    // Verify connection state (Cardputer style)
    if (catmCmd("AT+SHSTATE?", 5000).indexOf("+SHSTATE: 1") == -1) {
        Serial.println("[CM] SyncTime: SHSTATE not 1");
        catmCmd("AT+SHDISC", 3000);
        catmState = CM_INIT;
        refreshCatmLed();
        return false;
    }
    Serial.println("[CM] SyncTime: connected");

    // GET PATH_TIME（对时端点）→ {"ts":<epoch>,"str":"yyyy/mm/dd hh:mm:ss"}
    String ret = catmCmd("AT+SHREQ=\"" PATH_TIME "\",1", 15000);
    int code = 0, bodyLen = 0;
    int ci = ret.indexOf("\"GET\",");
    if (ci >= 0) {
        String s = ret.substring(ci + 6);
        code = s.substring(0, s.indexOf(",")).toInt();
        String ls = s.substring(s.indexOf(",") + 1);
        int pp = ls.indexOf("|");
        if (pp >= 0) ls = ls.substring(0, pp);
        bodyLen = ls.toInt();
    }
    Serial.printf("[CM] SyncTime: HTTP %d  body %d bytes\n", code, bodyLen);

    bool ok = false;
    if (code == 200 && bodyLen > 0 && bodyLen <= 80) {
        ret = catmCmd("AT+SHREAD=0," + String(bodyLen), 10000);
        Serial.printf("[CM] SyncTime body: %s\n", ret.c_str());

        // Isolate the HTTP body: it follows "+SHREAD: <n>|" in the response.
        String body = ret;
        int sh = ret.indexOf("+SHREAD:");
        if (sh >= 0) {
            int bar = ret.indexOf('|', sh);
            if (bar >= 0) body = ret.substring(bar + 1);
        }
        body.replace("|", "");

        // Accept either JSON {"ts":<epoch>,...} (clean UTC epoch) or the plain
        // "yyyy/mm/dd hh:mm:ss" form (server-local JST, so subtract 9h for UTC).
        time_t serverTs = 0;
        int tsIdx = body.indexOf("\"ts\":");
        if (tsIdx >= 0) {
            const char* p = body.c_str() + tsIdx + 5;
            while (*p && !isdigit((unsigned char)*p)) p++;
            serverTs = (time_t)atol(p);
        } else {
            int y, mo, d, h, mi, s;
            if (sscanf(body.c_str(), "%d/%d/%d %d:%d:%d",
                       &y, &mo, &d, &h, &mi, &s) == 6) {
                struct tm tm = {0};
                tm.tm_year = y - 1900; tm.tm_mon = mo - 1; tm.tm_mday = d;
                tm.tm_hour = h; tm.tm_min = mi; tm.tm_sec = s;
                serverTs = mktime(&tm) - 9 * 3600;   // JST wall-clock → UTC epoch
            }
        }

        if (serverTs > 1700000000L) {   // sanity: after Nov 2023
            struct timeval tv_now;
            gettimeofday(&tv_now, nullptr);
            long diff = (long)serverTs - (long)tv_now.tv_sec;
            Serial.printf("[CM] SyncTime: server=%ld  local=%ld  diff=%lds\n",
                          (long)serverTs, (long)tv_now.tv_sec, diff);
            if (labs(diff) > 60) {
                struct timeval tv_new = { serverTs, 0 };
                settimeofday(&tv_new, nullptr);
                Serial.printf("[CM] RTC updated -> %ld\n", (long)serverTs);
            } else {
                Serial.println("[CM] RTC already within 60s, no update");
            }
            ok = true;
        } else {
            Serial.printf("[CM] SyncTime: could not parse time from body\n");
        }
    }

    catmCmd("AT+SHDISC", 3000);

    if (ok) {
        catmState = CM_OK;
        refreshCatmLed();
        delay(2000);          // briefly show green to confirm network OK
        catmState = CM_READY; // blue = network confirmed working
    } else {
        catmState = CM_INIT;  // yellow = initialized but network not yet confirmed
    }
    refreshCatmLed();
    return ok;
}

// Format one track point as a JSON object into buf; returns the length written.
static int fmtPoint(char* buf, int cap, const TrackPoint& p) {
    return snprintf(buf, cap,
        "{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%d,\"spd\":%u,\"sat\":%u,"
        "\"hdop\":%.1f,\"bat_mv\":%u,\"bat_pct\":%u,\"ts\":%lu}",
        p.lat / 1e7, p.lon / 1e7, p.alt, p.spd, p.sat,
        p.hdop / 10.0, p.bat_mv, p.bat_pct, (unsigned long)p.ts);
}

// Snapshot the current GPS fix + battery into a compact TrackPoint.
// ── 统一位置访问器：上层只认这些，不直接碰 gps / liveFix ─────────────────────
#if GNSS_TIMESHARE
static inline bool    fixHasLoc()    { return liveFix.valid; }
static inline double  fixLat()       { return liveFix.lat; }
static inline double  fixLon()       { return liveFix.lon; }
static inline float   fixAltM()      { return liveFix.altM; }
static inline bool    fixHasSpeed()  { return liveFix.valid; }
static inline float   fixSpdKmh()    { return liveFix.spdKmh; }
static inline bool    fixHasHdop()   { return liveFix.valid; }
static inline float   fixHdop()      { return liveFix.hdop; }
static inline bool    fixHasSats()   { return liveFix.valid; }
static inline uint8_t fixSats()      { return liveFix.sats; }
static inline bool    fixHasCourse() { return liveFix.courseValid; }
static inline float   fixCourseDeg() { return liveFix.courseDeg; }

// 轮询一次 CGNSINF，解析进 liveFix。仅在 GNSS 跟踪模式调用（CGNSPWR=1）。
// 字段: 0run 1fix 2utc 3lat 4lon 5alt 6spd 7course 8mode 10HDOP 14satsView 15satsUsed
static void pollGnssIntoLiveFix() {
    String r = catmCmd("AT+CGNSINF", 3000);
    int p = r.indexOf("+CGNSINF:");
    if (p < 0) return;
    int e = r.indexOf('|', p);
    String body = (e > p) ? r.substring(p + 9, e) : r.substring(p + 9);
    String f[24]; int nf = 0, from = 0;
    for (int i = 0; i <= body.length() && nf < 24; i++)
        if (i == body.length() || body[i] == ',') { f[nf++] = body.substring(from, i); from = i + 1; }
    auto fld = [&](int i) -> String { return (i < nf) ? f[i] : String(); };
    if (fld(1).toInt() == 1) {
        liveFix.valid       = true;
        liveFix.lat         = atof(fld(3).c_str());      // atof=double，保住经纬度精度
        liveFix.lon         = atof(fld(4).c_str());
        liveFix.altM        = atof(fld(5).c_str());
        liveFix.spdKmh      = atof(fld(6).c_str());
        liveFix.courseValid = fld(7).length() > 0;
        liveFix.courseDeg   = liveFix.courseValid ? atof(fld(7).c_str()) : -1.0f;
        liveFix.hdop        = fld(10).length() ? atof(fld(10).c_str()) : 25.5f;
        liveFix.sats        = (uint8_t)fld(14).toInt();
    } else {
        liveFix.valid = false;
    }
    liveFix.tMs = millis();
}

// 配置B：CGNSINF 不带每星 CN0/星座。临时开 NMEA 流(CGNSTST=1)抓 ~1.5s 的 GSV/TXT，
// 喂 gnssDiagLine 填 gnssInView/gnssCN0/gnssAnt，供电量日志的 cn0/gnss/sats 列；
// 完事关流(=0)恢复 AT 通道。约每 2 分钟随采样跑一次，开销可忽略。
static void gnssSampleNmea() {
    if (!catmReady || !gnssTracking) return;
    nmeaLen = 0;
    Serial2.flush();
    while (Serial2.available()) Serial2.read();
    Serial2.println("AT+CGNSTST=1");           // GNSS NMEA 输出到 UART
    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {             // 抓约 1.5s（GSV ~1Hz，够一整轮）
        while (Serial2.available()) {
            char c = Serial2.read();
            if (c == '\n' || c == '\r') {
                if (nmeaLen) { nmeaLine[nmeaLen] = 0; gnssDiagLine(nmeaLine); nmeaLen = 0; }
            } else if (nmeaLen < (int)sizeof(nmeaLine) - 1) {
                nmeaLine[nmeaLen++] = c;
            } else nmeaLen = 0;                 // 行超长丢弃
        }
        delay(2);
    }
    Serial2.println("AT+CGNSTST=0");           // 关 NMEA 流
    delay(150);
    Serial2.flush();
    while (Serial2.available()) Serial2.read();  // 清残留 NMEA，免污染下次 AT
    nmeaLen = 0;
}

// 串口诊断 nmeatest：就地跑一次 NMEA 采样，打印 CN0/星座，并确认 CGNSINF 通道已恢复。
static void gnssNmeaTest() {
    if (!catmReady || !gnssTracking) { Serial.println("[NT] 需 CatM 就绪且在 GNSS 跟踪模式"); return; }
    Serial.println("[NT] 抓 NMEA(CGNSTST=1) ~1.5s 填 CN0/星座…");
    for (int k = 0; k < 6; k++) { gnssInView[k] = 0; gnssCN0[k] = 0; }   // 清旧值看本次
    gnssSampleNmea();
    const char* names[6] = {"GPS", "GLONASS", "BDS", "Galileo", "QZSS", "SBAS"};
    bool any = false;
    for (int k = 0; k < 6; k++) if (gnssInView[k] || gnssCN0[k]) {
        Serial.printf("[NT]   %s: 可见%u  最强CN0=%u dBHz\n", names[k], gnssInView[k], gnssCN0[k]);
        any = true;
    }
    if (!any) Serial.println("[NT]   (无星：室内/没见天属正常；CN0 需有星才有值)");
    Serial.printf("[NT] 天线=%u  通道恢复检查 CGNSINF -> %s\n",
                  gnssAnt, catmCmd("AT+CGNSINF", 3000).c_str());
}
#else
static inline bool    fixHasLoc()    { return gps.location.isValid(); }
static inline double  fixLat()       { return gps.location.lat(); }
static inline double  fixLon()       { return gps.location.lng(); }
static inline float   fixAltM()      { return gps.altitude.meters(); }
static inline bool    fixHasSpeed()  { return gps.speed.isValid(); }
static inline float   fixSpdKmh()    { return gps.speed.kmph(); }
static inline bool    fixHasHdop()   { return gps.hdop.isValid(); }
static inline float   fixHdop()      { return gps.hdop.hdop(); }
static inline bool    fixHasSats()   { return gps.satellites.isValid(); }
static inline uint8_t fixSats()      { return gps.satellites.value(); }
static inline bool    fixHasCourse() { return gps.course.isValid(); }
static inline float   fixCourseDeg() { return gps.course.deg(); }
#endif

static void buildTrackPoint(TrackPoint& p) {
    time_t now = time(nullptr);
    p.ts  = (now > 1735689600L) ? (uint32_t)now : 0;     // >2025-01-01 → RTC synced
    p.lat = (int32_t)lround(fixLat() * 1e7);
    p.lon = (int32_t)lround(fixLon() * 1e7);
    p.alt = (int16_t)fixAltM();
    float spd = fixHasSpeed() ? fixSpdKmh() : 0.0f;
    p.spd = spd > 255.0f ? 255 : (uint8_t)spd;
    p.sat = fixHasSats() ? fixSats() : 0;
    float h = fixHasHdop() ? fixHdop() : 25.5f;
    p.hdop = h * 10.0f > 255.0f ? 255 : (uint8_t)(h * 10.0f);
    p.bat_mv  = phVolt(VM_BAT);
    p.bat_pct = (uint8_t)batPct;
}

// Append a point to the queue. Ring buffer: overwrites the oldest when full, so
// it can never overflow — a long outage just keeps the most recent CAP points.
static void trackEnqueue(const TrackPoint& p) {
    trackQueue[trackHead] = p;
    trackHead = (trackHead + 1) % TRACK_QUEUE_CAP;
    if (trackCount < TRACK_QUEUE_CAP) trackCount++;
    Serial.printf("[Q] queued point (depth=%u/%u)\n", trackCount, TRACK_QUEUE_CAP);
}

// SIM7080G 的 SH(HTTPS)应用会退化进「每个射频周期只放行一次 SHCONN、之后一律
// +CME ERROR: operation not allowed」的锁死态（与 GNSS/信号/PDP/注册都无关——
// 2026-06-19 台面 + 外场复现：CNACT/CEREG/IPv4 全正常，仍连不上）。实测恢复手段：
//   · CFUN=0/1（射频重启，catmForceIPv4 用的）→ 清不掉，只换来「再一次」会话又锁；
//   · CFUN=1,1（整模块软重启）→ 真正解锁，恢复连续多会话（实测连发 8 次全 200）。
// 故 SH 锁死必须靠 CFUN=1,1。本函数整模块重启并重建到「可发 HTTPS」的最小状态。
static bool catmSHRecover() {
    Serial.println("[CM] === SH 锁死恢复：AT+CFUN=1,1 整模块软重启 ===");
    catmCmd("AT+CFUN=1,1", 20000);    // 等模块重启到 RDY
    delay(1000);
    catmCmd("ATE0", 2000);            // 重启后回显默认 ON，必须关掉，否则应答解析错位
    catmCmd("AT+CMEE=2", 2000);
    catmCmd("AT+CSCLK=0", 2000);
    bool reg = false;                 // 等重新注册（CEREG 1 home / 5 roaming）
    for (int i = 0; i < 15; i++) {
        String r = catmCmd("AT+CEREG?", 2000);
        int comma = r.indexOf(',');
        if (comma >= 0 && (r[comma+1] == '1' || r[comma+1] == '5')) { reg = true; break; }
        delay(1500);
    }
    catmCmd("AT+CNCFG=0,1,\"" CATM_APN "\"", 3000);   // 重建 IPv4 bearer（CGDCONT 在 NVRAM 保留）
    catmCmd("AT+CNACT=0,1", 15000);
    delay(2000);
    bool ip = catmHasIPv4(catmCmd("AT+CNACT?", 3000));
    Serial.printf("[CM] SH 恢复完成：reg=%d ipv4=%d\n", reg, ip);
    return reg && ip;
}

// Low-level HTTPS POST of a ready-made JSON body to PATH_APRS. Returns the HTTP
// status code, or -1 on a connection/transport failure. Caller owns net-check,
// LED/state and failure-count handling. Headers MUST be after SHCONN and SHCHEAD
// before SHAHEAD, else +CME ERROR → no Content-Type → server can't parse → 400.
// bodyCap = the SHCONF BODYLEN to set (>= bodyLen; SIM7080G max 1024).
static int catmPostBody(const char* body, int bodyLen, int bodyCap) {
    // SHCONF/SSL 配置 + SHCONN 建链；若首次 SHCONN 命中 SH 锁死（operation not
    // allowed），CFUN=1,1 整模块重启后把整段重做一遍再连（见 catmSHRecover）。
    bool connected = false;
    for (int attempt = 0; attempt < 2 && !connected; attempt++) {
        catmCmd("AT+SHDISC", 3000);
        delay(500);
        catmCmd("AT+SHCONF=\"URL\",\"" SERVER_BASE "\"", 5000);
        catmCmd("AT+SHCONF=\"BODYLEN\"," + String(bodyCap), 3000);
        catmCmd("AT+SHCONF=\"HEADERLEN\",350", 3000);
        catmCmd("AT+CSSLCFG=\"ignorertctime\",1,1", 3000);
        catmCmd("AT+CSSLCFG=\"sslversion\",1,3", 3000);
        catmCmd("AT+CSSLCFG=\"sni\",1,\"" SERVER_HOST "\"", 3000);
        catmCmd("AT+SHSSL=1,\"\"", 3000);

        String shret = catmCmd("AT+SHCONN", 10000);
        if (shret.indexOf("|OK|") >= 0) { connected = true; break; }

        if (attempt == 0 && shret.indexOf("operation not allowed") >= 0) {
            Serial.println("[CM] SHCONN 'operation not allowed' → SH 栈锁死，CFUN=1,1 恢复后重试");
            catmSHRecover();          // 整模块重启解锁；下一轮 for 重建 SH 会话再连
        } else {
            Serial.printf("[CM] SHCONN fail — CEER=%s\n", catmCmd("AT+CEER", 2000).c_str());
            break;
        }
    }
    if (!connected) return -1;

    if (catmCmd("AT+SHSTATE?", 5000).indexOf("+SHSTATE: 1") == -1) {
        Serial.println("[CM] SHSTATE not 1");
        catmCmd("AT+SHDISC", 3000);
        return -1;
    }

    catmCmd("AT+SHCHEAD", 3000);
    catmCmd("AT+SHAHEAD=\"Content-Type\",\"application/json\"", 3000);

    String ret = catmCmd("AT+SHBOD=" + String(bodyLen) + ",3000", 5000);
    if (ret.indexOf(">") >= 0) {
        Serial2.write((const uint8_t*)body, bodyLen);
        Serial2.flush();
        delay(400);
    }
    ret = catmCmd("AT+SHREQ=\"" PATH_APRS "\",3", 30000);

    // Parse HTTP status code from +SHREQ: "POST",<code>,<len>
    int ci = ret.indexOf("\"POST\",");
    int code = 0;
    if (ci >= 0) {
        String s = ret.substring(ci + 7);
        code = s.substring(0, s.indexOf(",")).toInt();
    }
    catmCmd("AT+SHDISC", 3000);
    return code;
}

// Re-send backlogged points, oldest first, in batches (JSON array body). Stops at
// the first failed batch (network still down) — those stay queued. Capped per
// call so one flush can't block the loop too long.
static void trackFlush() {
    if (trackCount == 0) return;
    static char body[1024];
    uint8_t batches = 0;
    while (trackCount > 0 && batches < TRACK_FLUSH_BATCHES) {
        uint16_t take = trackCount < TRACK_BATCH ? trackCount : TRACK_BATCH;
        uint16_t tail = (trackHead - trackCount + TRACK_QUEUE_CAP) % TRACK_QUEUE_CAP;
        int pos = 0;
        body[pos++] = '[';
        for (uint16_t i = 0; i < take; i++) {
            const TrackPoint& p = trackQueue[(tail + i) % TRACK_QUEUE_CAP];
            if (i) body[pos++] = ',';
            pos += fmtPoint(body + pos, sizeof(body) - pos - 2, p);
        }
        body[pos++] = ']';
        body[pos]   = 0;

        if (!catmCheckNet()) { Serial.println("[Q] flush: net gone — keep queued"); return; }
        int code = catmPostBody(body, pos, 1024);
        Serial.printf("[Q] flush %u pts -> HTTP %d (depth was %u)\n", take, code, trackCount);
        if (code == 200 || code == 201) {
            trackCount -= take;          // dequeue the oldest `take`
            batches++;
        } else {
            return;                      // still failing → leave queued for next time
        }
    }
}

// POST the current GPS fix to the home server via HTTPS. On failure the point is
// queued for later (store-and-forward); on success any backlog is flushed too.
// queueOnFail=false for the bench/forced path, where coords may be stale or 0,0.
#if GNSS_TIMESHARE
// 配置B：GNSS↔LTE 分时。当前在 GNSS 跟踪模式 → 切 LTE → 发包 → 切回 GNSS 继续跟踪。
static bool sendGpsData(bool queueOnFail) {
    TrackPoint cur;
    buildTrackPoint(cur);                 // 用当前 liveFix
    catmState = CM_SENDING; refreshCatmLed();

    catmCmd("AT+CGNSPWR=0", 3000);        // 出 GNSS（让出射频给 LTE）
    gnssTracking = false;
    if (catmFailStreak >= CATM_FAIL_REATTACH) {   // 连续失败 → 强制 IPv4 重附着
        Serial.printf("[CM] %u consecutive failures -> IPv4 re-attach\n", catmFailStreak);
        catmForceIPv4();
        catmFailStreak = 0;
    }
    // ★ 切回 LTE 后等重新附着再激活 PDP——分时共用射频，CGNSPWR=0 那刻 LTE 还在搜网，
    //   原注释“~0.2s 可发、CEREG 不掉”是错的：立刻 CNACT/判网会误判失败 → 红灯。
    catmWaitReg(20000);
    catmCmd("AT+CNACT=0,1", 12000);       // 恢复 PDP（已附着后再激活）

    bool ok = false;
    if (catmCheckNet()) {
        char body[160];
        int bodyLen = fmtPoint(body, sizeof(body), cur);
        Serial.printf("[CM] body(%d): %s\n", bodyLen, body);
        int code = catmPostBody(body, bodyLen, 1024);
        Serial.printf("[CM] POST %s -> HTTP %d\n", PATH_APRS, code);
        ok = (code == 200 || code == 201);
        if (ok) { catmFailStreak = 0; trackFlush(); }   // 仍在 LTE，顺手补发积压
    } else {
        Serial.println("[CM] Net unavailable (no usable IPv4)");
    }

    catmCmd("AT+CNACT=0,0", 5000);        // 让位
    catmCmd("AT+CGNSPWR=1", 3000);        // 切回 GNSS 继续跟踪
    gnssTracking  = true;
    tLastGnssPoll = 0;                    // 立刻重新轮询定位

    if (ok) {
        catmState = CM_OK; refreshCatmLed(); delay(800); catmState = CM_READY;
    } else {
        catmFailStreak++; catmState = CM_ERR;
        if (queueOnFail) trackEnqueue(cur);
    }
    refreshCatmLed();
    return ok;
}
#else
static bool sendGpsData(bool queueOnFail) {
    catmState = CM_SENDING;
    refreshCatmLed();

    TrackPoint cur;
    buildTrackPoint(cur);

    // Escalate recovery: after repeated failures the bearer is likely stuck on
    // an unusable (IPv6-only) address or the context was lost. Re-attach with
    // IPv4 pinned before retrying — roughly what a cell handover would force,
    // which is what finally recovers a context that went bad out in the field.
    if (catmFailStreak >= CATM_FAIL_REATTACH) {
        Serial.printf("[CM] %u consecutive failures → IPv4 re-attach\n", catmFailStreak);
        catmForceIPv4();
        catmFailStreak = 0;
    }

    if (!catmCheckNet()) {
        Serial.println("[CM] Net unavailable (no usable IPv4)");
        catmState = CM_ERR;
        refreshCatmLed();
        catmFailStreak++;
        if (queueOnFail) trackEnqueue(cur);
        return false;
    }

    char body[160];
    int bodyLen = fmtPoint(body, sizeof(body), cur);
    Serial.printf("[CM] body(%d): %s\n", bodyLen, body);

    int code = catmPostBody(body, bodyLen, 1024);
    Serial.printf("[CM] POST %s -> HTTP %d\n", PATH_APRS, code);

    bool ok = (code == 200 || code == 201);
    if (ok) {
        catmFailStreak = 0;       // recovered — reset escalation
        catmState = CM_OK;        // green = upload succeeded
        refreshCatmLed();
        delay(2000);              // brief green confirmation flash
        catmState = CM_READY;     // back to blue = idle / network ready
        refreshCatmLed();
        trackFlush();             // network is up — push any backlog
    } else {
        catmFailStreak++;
        catmState = CM_ERR;       // red — stays until the next attempt
        refreshCatmLed();
        if (queueOnFail) trackEnqueue(cur);
    }
    return ok;
}
#endif  // GNSS_TIMESHARE sendGpsData

// USB output LEDs — side USB-C (LED_USB_C) and USB-A (LED_USB_A) are paired outputs.
// Both follow the same double-click toggle and share one current sensor (VM_USB).
// Blue = output enabled, no load.  Green = load detected on either port.
static void refreshUsbLed() {
    if (!usbEnabled) {
        phLedSet(LED_USB_C, C_OFF);
        phLedSet(USB_LED,   C_OFF);
        return;
    }
    int16_t cur = phCurr(VM_USB);
    uint32_t c = (cur > USB_LOAD_MA || cur < -USB_LOAD_MA) ? C_GREEN : C_BLUE;
    phLedSet(LED_USB_C, c);
    phLedSet(USB_LED,   c);
}

// ═══════════════════════════════════════════════════════════════════════════
// GPS state machine update
// ═══════════════════════════════════════════════════════════════════════════
#if GNSS_TIMESHARE
// 配置B：无 PORT.C 模块；gpsState 由 liveFix 新鲜度决定（liveFix 在 loop 轮询维护）。
static void updateGps() {
    if (gpsState == GS_DETECTING) gpsState = GS_SEARCHING;   // 无检测期
    bool good = liveFix.valid && (millis() - liveFix.tMs < 5000);
    if (good && gpsState != GS_FIX_GOOD)
        Serial.printf("[GPS] Fix lat=%.6f lon=%.6f hdop=%.1f sat=%u (7080G)\n",
                      liveFix.lat, liveFix.lon, liveFix.hdop, liveFix.sats);
    else if (!good && gpsState == GS_FIX_GOOD)
        Serial.println("[GPS] Fix lost");
    gpsState = good ? GS_FIX_GOOD : GS_SEARCHING;
}
#else
static void updateGps() {
    uint32_t now = millis();

    if (gpsState == GS_DETECTING) {
        uint32_t chOk   = gps.passedChecksum();
        uint32_t chFail = gps.failedChecksum();
        uint32_t chAll  = gps.charsProcessed();

        if (chOk > 0) {
            gpsState      = GS_INIT_OK;
            tGpsInitData  = now;
            Serial.println("[GPS] Module OK – first valid sentence");
        } else if (chAll > 60 && chFail > 10) {
            // Stream present but corrupt – hardware or pin issue
            gpsState = GS_INIT_FAIL;
            Serial.printf("[GPS] Init FAIL – %u chars, %u bad\n", chAll, chFail);
        } else if (now - tBoot > GPS_DETECT_MS) {
            gpsState = GS_NO_MODULE;
            Serial.println("[GPS] No module detected");
        }
        return;
    }

    // Brief blue "init OK" → transition to searching
    if (gpsState == GS_INIT_OK && now - tGpsInitData > GPS_INIT_SHOW_MS) {
        gpsState = GS_SEARCHING;
    }

    // Searching ↔ fix good
    // Use location.age() rather than HDOP: HDOP is only committed by TinyGPS++
    // when GGA already has fix quality > 0, creating a circular dependency.
    // A fresh valid position (updated within the last 5 s) means we have a fix.
    if (gpsState == GS_SEARCHING || gpsState == GS_FIX_GOOD) {
        bool good = gps.location.isValid() && gps.location.age() < 5000;

        if (good && gpsState != GS_FIX_GOOD) {
            Serial.printf("[GPS] Fix  lat=%.6f lon=%.6f hdop=%.1f sat=%u\n",
                gps.location.lat(), gps.location.lng(),
                gps.hdop.isValid() ? gps.hdop.hdop() : -1.0f,
                gps.satellites.isValid() ? gps.satellites.value() : 0);
        } else if (!good && gpsState == GS_FIX_GOOD) {
            Serial.println("[GPS] Fix lost");
        }
        gpsState = good ? GS_FIX_GOOD : GS_SEARCHING;

        // Periodic diagnostic while searching
        if (gpsState == GS_SEARCHING && now - tLastGpsLog >= 10000) {
            tLastGpsLog = now;
            time_t rt = time(nullptr);
            char rtcStr[32] = "not-set";
            if (rt > 1735689600L) {   // > 2025-01-01 means RTC has been synced
                struct tm t;
                localtime_r(&rt, &t);
                snprintf(rtcStr, sizeof(rtcStr), "%04d-%02d-%02d %02d:%02d:%02d",
                         t.tm_year+1900, t.tm_mon+1, t.tm_mday,
                         t.tm_hour, t.tm_min, t.tm_sec);
            }
            Serial.printf("[GPS] searching — loc_valid=%d age=%lums hdop=%.1f sat=%u chars=%u ok=%u  rtc=%s\n",
                gps.location.isValid(),
                gps.location.isValid() ? (unsigned long)gps.location.age() : 0UL,
                gps.hdop.isValid() ? gps.hdop.hdop() : -1.0f,
                gps.satellites.isValid() ? gps.satellites.value() : 0,
                gps.charsProcessed(), gps.passedChecksum(), rtcStr);
        }
    }
}

#endif  // GNSS_TIMESHARE updateGps

// ═══════════════════════════════════════════════════════════════════════════
// Adaptive beaconing decision (SmartBeaconing + decay + corner pegging)
// ═══════════════════════════════════════════════════════════════════════════

// Snapshot the just-sent fix as the new anchor (position + course + time).
static void recordAnchor() {
    tLastSend = millis();
    if (fixHasLoc()) {
        lastTxLat  = fixLat();
        lastTxLon  = fixLon();
        haveAnchor = true;
    }
    lastTxCourse = fixHasCourse() ? fixCourseDeg() : -1.0;
}

// Decide whether a beacon is due right now. Call only with a good GPS fix.
// On true, *why points to a short reason string and *stopped tells the caller
// whether we're in the stopped (decay) regime, so it can grow/reset the decay.
static bool beaconDue(const char** why, bool* stopped) {
    uint32_t now     = millis();
    uint32_t elapsed = now - tLastSend;
    float    spd     = fixHasSpeed() ? fixSpdKmh() : 0.0f;
    bool     hdopOk  = fixHasHdop() && fixHdop() < HDOP_MAX
                       && fixHasSats() && fixSats() >= SAT_MIN;
    *stopped = (spd < SB_LOW_SPEED_KMH);

    // 1. First beacon after boot / fresh fix
    if (!haveAnchor) { *why = "first"; return true; }

    // Hard rate floor: a send already takes ~20 s, so never beacon more often
    // than this. This also tames the "moved" trigger — at cycling/原付 speed 40 m
    // is reached every few seconds, but we cap it to one beacon per MIN_TX_GAP.
    if (elapsed < MIN_TX_GAP_MS) return false;

    // 1b. Last send failed: retry sooner than the normal cadence so a transient
    //     IPv6-only bearer or a dead cell doesn't leave us stuck red for a whole
    //     stopped interval (5-30 min). sendGpsData() re-attaches/re-IPv4s as needed.
    if (catmState == CM_ERR && elapsed >= CATM_FAIL_RETRY_MS) { *why = "retry"; return true; }

    // 2. Moved beyond threshold from the anchor (HDOP-gated to reject jitter)
    if (hdopOk) {
        double dist = TinyGPSPlus::distanceBetween(
            fixLat(), fixLon(), lastTxLat, lastTxLon);
        if (dist > MOVE_THRESH_M) { *why = "moved"; return true; }
    }

    // 3. Corner pegging — only when moving fast enough and heading is known
    //    (GPS course is noisy at walking speed; walking turns are caught by #2).
    if (spd > TURN_MIN_SPEED && lastTxCourse >= 0.0 && fixHasCourse()) {
        float diff = fabsf(fixCourseDeg() - (float)lastTxCourse);
        if (diff > 180.0f) diff = 360.0f - diff;
        if (diff > TURN_MIN_DEG + TURN_SLOPE / spd && elapsed > TURN_MIN_MS) {
            *why = "turn"; return true;
        }
    }

    // 4. Interval elapsed: decay when stopped, SmartBeaconing linear when moving
    uint32_t interval;
    if (*stopped) {
        interval = decayInterval;
    } else if (spd >= SB_HIGH_SPEED_KMH) {
        interval = SB_FAST_MS;
    } else {
        float frac = (SB_HIGH_SPEED_KMH - spd) / (SB_HIGH_SPEED_KMH - SB_LOW_SPEED_KMH);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;
        interval = SB_FAST_MS + (uint32_t)(frac * (SB_SLOW_MS - SB_FAST_MS));
    }
    if (elapsed >= interval) { *why = "interval"; return true; }

    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
// Power-saving shutdown
// Cuts every peripheral power rail (so the coprocessor's standby current is
// minimal — the built-in BtnPWR double-click leaves these on, which drains the
// battery), then writes the coprocessor power-off command (REG 0xE0).  Charging
// is left enabled so it can still charge while off.  If power isn't actually cut
// (e.g. external power present), we reboot to restore a known-good state.
// Powers the device off, so it normally never returns.  Press BtnPWR to wake.
// ═══════════════════════════════════════════════════════════════════════════
static void powerSaveShutdown() {
    Serial.println("[PWR] long-press → power-saving shutdown");
    phPower(PC_USB,     false);
    phPower(PC_UART,    false);   // PORT.C — GPS module
    phPower(PC_I2C,     false);   // PORT.A — CatM module
    phPower(PC_VAMETER, false);   // INA226 current/voltage monitors
    phPower(PC_BUS,     false);   // RS485/CAN
    phPower(PC_LED,     false);   // all LEDs
    // PC_CHARGE left ON so the battery still charges while powered off.
    delay(50);
    Serial.println("[PWR] writing power-off (REG 0xE0)");
    Serial.flush();
    phWr8(REG_OFF, 1);            // coprocessor cuts power
    delay(2500);
    // Still alive → power-off was blocked (VIN present?). Reboot to restore.
    Serial.println("[PWR] still powered — rebooting to restore rails");
    ESP.restart();
}

#if GNSS_TIMESHARE
// ═══════════════════════════════════════════════════════════════════════════
// LCD 状态屏（Unit LCD 1.14"，竖屏 135x240，黑底浅色字）
// 显示：定位状态/卫星·HDOP、网络状态、电量、上次上报、积压队列、JST 时间。
// 仅在 displayOn 时绘制；息屏时让面板睡眠省电（sleep() 内含亮度0）。
// 颜色用 ili9341_colors 的裸名（GREEN/YELLOW/…，M5GFX.h 已全局 using）。
// ═══════════════════════════════════════════════════════════════════════════
// 只引入需要的字体/对齐符号——不能 `using namespace m5gfx`（它含 lgfx::v1::millis/
// delay，会和 Arduino 的 millis()/delay() 在全局撞名导致歧义）。
namespace fonts = lgfx::fonts;
using lgfx::textdatum_t;

// 初始化 LCD（PORT.C，I2C 端口1；端口0 被 PowerHub 占用）。失败不致命，只是没屏。
static void lcdInit() {
    // S3 上 Unit LCD 默认 SDA=G2(白)/SCL=G1(黄)，正好是 PORT.C 接线。
    // 控制器上电后要一会儿才应答 → 主方向多探几次（带间隔），都不行再试互换接线。
    for (int i = 0; i < 4 && !lcdOK; i++) {
        if (i) delay(250);
        lcdOK = lcd.init(2 /*SDA G2白*/, 1 /*SCL G1黄*/, 400000, 1, 0x3E);
    }
    if (!lcdOK) {
        Serial.println("[LCD] (SDA=2,SCL=1) 多次无应答，试互换 (SDA=1,SCL=2)…");
        for (int i = 0; i < 2 && !lcdOK; i++) {
            if (i) delay(250);
            lcdOK = lcd.init(1, 2, 400000, 1, 0x3E);
        }
    }
    if (!lcdOK) { Serial.println("[LCD] init FAIL（检查 PORT.C 接线/供电）"); return; }
    lcd.setRotation(0);                 // 0 = 竖屏 135(宽) x 240(高)
    lcd.setBrightness(LCD_BRIGHTNESS);
    lcd.fillScreen(BLACK);
    // 离屏画布：优先 PSRAM，失败回退内部 RAM；都失败则直接画到屏（会略闪）。
    lcdCanvas.setColorDepth(16);
    lcdCanvasOK = (lcdCanvas.createSprite(135, 240) != nullptr);
    if (!lcdCanvasOK) {
        lcdCanvas.setPsram(false);
        lcdCanvasOK = (lcdCanvas.createSprite(135, 240) != nullptr);
    }
    Serial.printf("[LCD] init OK（画布 %s）\n", lcdCanvasOK ? "PSRAM/RAM" : "无→直绘");
}

// 当前定位状态 → 显示词 + 颜色（与 GPS LED 同义）。标签缩成单字母 G，故状态词用全拼。
static const char* gpsWord(int* col) {
    switch (gpsState) {
        case GS_FIX_GOOD:  *col = GREEN;    return "FIX";
        case GS_SEARCHING: *col = YELLOW;   return "SEARCH";
        case GS_INIT_OK:   *col = CYAN;     return "INIT";
        case GS_INIT_FAIL: *col = RED;      return "ERR";
        default:           *col = DARKGREY; return "OFF";
    }
}
// 当前网络状态 → 显示词 + 颜色（与 CatM LED 同义）。标签缩成单字母 N，故状态词用全拼。
static const char* netWord(int* col) {
    switch (catmState) {
        case CM_READY:   *col = CYAN;     return "READY";
        case CM_SENDING: *col = WHITE;    return "SEND";
        case CM_OK:      *col = GREEN;    return "OK";
        case CM_ERR:     *col = RED;      return "FAIL";
        case CM_INIT:    *col = YELLOW;   return "INIT";
        default:         *col = DARKGREY; return "OFF";
    }
}

// 把状态画进 g（离屏画布或直接是屏）。
static void lcdRender(LovyanGFX* g) {
    char line[40];
    int  col;
    g->fillScreen(BLACK);
    g->setTextWrap(false);

    // 正文统一 Font4(26px)；细节(卫星行)也用 Font4 居中，运行时长/TX 用 Font2(16px)。
    // ── 时间（与其它行同字号 Font4，仅 HH:MM，居中）──
    // RTC 存 UTC epoch；+9h 用 gmtime 出 JST 墙钟，不动全局 TZ
    // （动 TZ 会让 catmSyncTime 里 mktime 的 JST→UTC 换算重复减 9h 出错）。
    time_t rt = time(nullptr);
    g->setFont(&fonts::Font4);
    g->setTextDatum(textdatum_t::top_center);
    if (rt > 1735689600L) {
        time_t jst = rt + 9 * 3600;
        struct tm t; gmtime_r(&jst, &t);
        snprintf(line, sizeof(line), "%02d:%02d", t.tm_hour, t.tm_min);
        g->setTextColor(WHITE, BLACK);
    } else {
        snprintf(line, sizeof(line), "--:--");
        g->setTextColor(DARKGREY, BLACK);
    }
    g->drawString(line, 67, 8);
    g->drawFastHLine(4, 38, 127, DARKGREY);

    // ── 定位 ──
    g->setFont(&fonts::Font4);
    g->setTextDatum(textdatum_t::top_left);
    g->setTextColor(LIGHTGREY, BLACK);
    g->drawString("G", 4, 44);
    const char* gw = gpsWord(&col);
    g->setTextColor(col, BLACK);
    g->setTextDatum(textdatum_t::top_right);
    g->drawString(gw, 131, 44);
    // 卫星数：Font4 居中
    g->setTextDatum(textdatum_t::top_center);
    g->setTextColor(WHITE, BLACK);
    snprintf(line, sizeof(line), "%u sate", (unsigned)fixSats());
    g->drawString(line, 67, 76);
    g->drawFastHLine(4, 106, 127, DARKGREY);

    // ── 网络 ──
    g->setTextDatum(textdatum_t::top_left);
    g->setTextColor(LIGHTGREY, BLACK);
    g->drawString("N", 4, 112);
    const char* nw = netWord(&col);
    g->setTextColor(col, BLACK);
    g->setTextDatum(textdatum_t::top_right);
    g->drawString(nw, 131, 112);
    g->drawFastHLine(4, 142, 127, DARKGREY);

    // ── 电量（一行：左 % 按电量着色 + 右 电压一位小数）──
    int bcol = !batValid ? CYAN
             : batPct < 10 ? RED
             : batPct < 30 ? ORANGE
             : batPct < 60 ? YELLOW : GREEN;
    g->setTextDatum(textdatum_t::top_left);
    g->setTextColor(bcol, BLACK);
    snprintf(line, sizeof(line), "%d%%", batPct);
    g->drawString(line, 4, 148);
    g->setTextDatum(textdatum_t::top_right);
    g->setTextColor(WHITE, BLACK);
    if (batMv > 1000) snprintf(line, sizeof(line), "%.1fV", batMv / 1000.0);
    else              snprintf(line, sizeof(line), "--");
    g->drawString(line, 131, 148);
    g->drawFastHLine(4, 178, 127, DARKGREY);

    // ── 运行时长（Font2 小字、居中；分精度，只用 h/m。时数>2位时去掉 "up" 省宽）──
    {
        uint32_t upMin = millis() / 60000UL, uh = upMin / 60, um = upMin % 60;
        g->setFont(&fonts::Font2);
        g->setTextDatum(textdatum_t::top_center);
        g->setTextColor(LIGHTGREY, BLACK);
        if (uh < 100) snprintf(line, sizeof(line), "up %luh %lum",
                               (unsigned long)uh, (unsigned long)um);
        else          snprintf(line, sizeof(line), "%luh %lum",
                               (unsigned long)uh, (unsigned long)um);
        g->drawString(line, 67, 184);
    }

    // ── 上次上报 / 队列（底部一行小字）──
    g->setFont(&fonts::Font2);
    g->setTextDatum(textdatum_t::top_left);
    g->setTextColor(WHITE, BLACK);
    if (haveAnchor) {
        uint32_t ago = (millis() - tLastSend) / 1000;
        if (ago < 600) snprintf(line, sizeof(line), "TX %lus", (unsigned long)ago);
        else           snprintf(line, sizeof(line), "TX %lum", (unsigned long)(ago / 60));
    } else snprintf(line, sizeof(line), "TX --");
    g->drawString(line, 4, 214);
    g->setTextDatum(textdatum_t::top_right);
    if (catmFailStreak) {
        g->setTextColor(RED, BLACK);
        snprintf(line, sizeof(line), "q%u f%u",
                 (unsigned)trackCount, (unsigned)catmFailStreak);
    } else {
        g->setTextColor(trackCount ? ORANGE : DARKGREY, BLACK);
        snprintf(line, sizeof(line), "q%u", (unsigned)trackCount);
    }
    g->drawString(line, 131, 214);
}

// 重绘状态屏（仅亮屏时）。画布存在则整帧推送（无闪烁），否则直绘。
static void lcdDrawStatus() {
    if (!lcdOK || !displayOn) return;
    if (lcdCanvasOK) { lcdRender(&lcdCanvas); lcdCanvas.pushSprite(0, 0); }
    else             { lcdRender(&lcd); }
    tLastLcdDraw = millis();
}

// 引导/提示屏（初始化阶段，状态数据还没齐）。
static void lcdBootMsg(const char* msg) {
    if (!lcdOK) return;
    LovyanGFX* g = lcdCanvasOK ? (LovyanGFX*)&lcdCanvas : (LovyanGFX*)&lcd;
    g->fillScreen(BLACK);
    g->setFont(&fonts::Font2);
    g->setTextDatum(textdatum_t::middle_center);
    g->setTextColor(CYAN, BLACK);
    g->drawString("M5 APRS", 67, 100);
    g->setTextColor(LIGHTGREY, BLACK);
    g->drawString(msg, 67, 130);
    if (lcdCanvasOK) lcdCanvas.pushSprite(0, 0);
}

// 开/关屏。开 → 唤醒 + 设 30s 亮屏窗口 + 立即重绘；关 → 睡眠省电。
static void displaySetOn(bool on) {
    if (!lcdOK) { displayOn = on; return; }
    if (on) {
        displayOn   = true;
        tDisplayOff = millis() + DISPLAY_ON_MS;
        lcd.wakeup();
        lcd.setBrightness(LCD_BRIGHTNESS);
        lcdDrawStatus();
    } else {
        displayOn = false;
        lcd.sleep();            // 含亮度0
    }
}
#endif  // GNSS_TIMESHARE LCD

// ═══════════════════════════════════════════════════════════════════════════
// Buttons (all active-low: idle reads 1, pressed reads 0):
//   Big top button (BTN_OK, REG_BTN bit0)  → short press   → upload GPS now (needs fix)
//                                           → long-press 3s → FORCE CatM upload, ignore
//                                                             GPS fix (bench diagnostic)
//   Yellow round button (GPIO11/BTN_SELECT) → double-click  → toggle USB-A power
//                                           → long-press 3s → power-saving shutdown
// ═══════════════════════════════════════════════════════════════════════════

static void checkButtons() {
    uint32_t now = millis();

    // ── Big top button (BTN_OK) ──────────────────────────────────────────────
    //   short press   → 配置B: 亮屏/熄屏开关 | 配置A: upload current position
    //   long-press 3s → force a CatM upload regardless of GPS fix, so the POST
    //                   path can be exercised/diagnosed on the bench (indoors,
    //                   where there is no fix and uploads otherwise never fire).
    // REG_BTN is active-low, so a pressed button reads 0 → invert to get "down".
    bool okNow = !phBtn(BTN_OK_BIT);
    if (okNow && !prevBtnOK) {                   // press begins
        tOkPress    = now;
        okLongFired = false;
    }
    if (okNow && !okLongFired && now - tOkPress >= LONGPRESS_MS) {
        okLongFired  = true;                     // fire once, mid-hold
        forceSendReq = true;
        Serial.println("[BTN] top button long-press → FORCE CatM upload (ignore fix)");
    }
    if (!okNow && prevBtnOK) {                    // release
        if (!okLongFired) {                       // short press
#if GNSS_TIMESHARE
            // 配置B：短按 = 亮屏/熄屏开关。亮屏 30s 后自动熄；亮着时再按立刻熄。
            // （手动上传改由长按“强制上传”承担；移动中本就自动 beacon。）
            Serial.printf("[BTN] top short press → display %s\n", displayOn ? "OFF" : "ON");
            displaySetOn(!displayOn);
#else
            Serial.println("[BTN] top button short press → request GPS upload");
            manualSendReq = true;
#endif
        }
    }
    prevBtnOK = okNow;

    // ── Yellow round button (GPIO11) ─────────────────────────────────────────
    //   double-click  → toggle USB-A power
    //   long-press 3s → power-saving shutdown
    // Also active-low: pressed = LOW, so invert digitalRead (this was reversed
    // before, which caused a phantom click on every boot).
    bool selNow = !(bool)digitalRead(BTN_SELECT_GPIO);

    if (selNow && !prevBtnSel) {                 // press begins
        tSelPress    = now;
        selLongFired = false;
    }
    if (selNow && !selLongFired && now - tSelPress >= LONGPRESS_MS) {
        selLongFired = true;                     // fire once, mid-hold
        powerSaveShutdown();                     // powers off — does not return
    }
    if (!selNow && prevBtnSel) {                 // release
        if (!selLongFired) {                     // short press → double-click FSM
            if (clickCount == 0) {
                clickCount  = 1;
                tFirstClick = now;
            } else {
                if (now - tFirstClick <= DBLCLICK_MS) {
                    // ── Double-click confirmed ────────────────────────────────
                    usbEnabled = !usbEnabled;
                    phPower(PC_USB, usbEnabled);
                    refreshUsbLed();
                    Serial.printf("[USB] toggled → %s\n", usbEnabled ? "ON" : "OFF");
                }
                clickCount = 0;
            }
        }
    }
    if (clickCount == 1 && now - tFirstClick > DBLCLICK_MS)
        clickCount = 0;          // lone single click = no action

    prevBtnSel = selNow;
}

// ═══════════════════════════════════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(300);   // let USB-serial settle before first print
    Serial.println("\n=== M5Power APRS Tracker — Step 1 ===");

    // Validate/restore the RTC power log (survives resets, not full power-off).
    pwrlogInit();

    // ── Power saving ─────────────────────────────────────────────────────────
    setCpuFrequencyMhz(CPU_MHZ);
    WiFi.mode(WIFI_OFF);
    btStop();
    Serial.printf("[SYS] CPU @ %u MHz, WiFi/BT off\n", getCpuFrequencyMhz());

    // ── PowerHub ─────────────────────────────────────────────────────────────
    if (!phInit()) {
        // If PowerHub not found the device can't function — hang with serial error
        Serial.println("[PH] FATAL: not found on I2C 0x50");
        for (;;) delay(1000);
    }
    Serial.printf("[PH] OK  fw=0x%02X\n", phRd8(0xFE));

    // ── LEDs: clear all, then set brightness for used LEDs ───────────────────
    // Force-write OFF on first call by pre-loading cache with an invalid value
    for (int i = 0; i < 8; i++) ledCache[i] = 0xFFFFFFFFUL;
    for (int i = 0; i < 8; i++) phLedBright(i, 0);        // all dim
    phLedBright(LED_USB_C, BR_USB);    // side USB-C output (paired with USB-A)
    phLedBright(GPS_LED,   BR_GPS);
    phLedBright(BAT_LED,   BR_BAT);
    phLedBright(USB_LED,   BR_USB);
    phLedBright(CHG_LED,   BR_CHG);   // external-power indicator
    phLedBright(CATM_LED,  BR_CATM);  // CatM status
    for (int i = 0; i < 8; i++) phLedSet(i, C_OFF);       // all off

    // ── Port power defaults ──────────────────────────────────────────────────
    phPower(PC_LED,     true);    // LED power rail must be on
    phPower(PC_UART,    true);    // PORT.C — GPS module power
    phPower(PC_I2C,     true);    // PORT.A — CatM module power
    phPower(PC_VAMETER, true);    // voltage/current monitoring
    delay(200);                   // let INA226 complete first conversion before reading
    phPower(PC_CHARGE,  true);    // allow battery charging
    phPower(PC_USB,     false);   // USB-A output OFF by default
    phPower(PC_BUS,     false);   // RS485/CAN not used

    // ── GPIO ─────────────────────────────────────────────────────────────────
    pinMode(BTN_SELECT_GPIO, INPUT);   // small round button — USB toggle

    // ── CatM serial (must start before GPS so Serial2 is claimed first) ─────────
    Serial2.setRxBufferSize(4096);
    Serial2.begin(CATM_BAUD, SERIAL_8N1, CATM_RX_PIN, CATM_TX_PIN);
    Serial.println("[CM] UART2 started — 115200 8N1");

#if !GNSS_TIMESHARE
    // ── GPS serial ───────────────────────────────────────────────────────────
    // 配置B 不开 PORT.C 串口：G1/G2 留给 LCD 的 I2C。
    gpsSerial.setRxBufferSize(2048);   // larger buffer: send() can block ~20s
    gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("[GPS] UART1 started — 115200 8N1");

    // ── GPS module init (ATGM336H-6N / AT6668 PCAS commands) ─────────────────
    // 这颗 -6N(AT6668, 50 通道, 全模)出厂支持 GPS/QZSS+BDS+GLONASS+Galileo。
    // 旧代码每次开机都发 $PCAS10,9(出厂启动)→ 清星历 → 每次开机都冷启动(≤23s)，
    // 且 $PCAS04,7 只开 3 系统(漏了 Galileo=8、未显式带 QZSS)。
    //
    // 现改为「一次性配置」：用 NVS 记录配置版本，只在首刷/版本变更后配一次，
    //   配置写进模块自身 flash(PCAS00)。之后每次开机不发任何 GPS 命令 →
    //   模块凭 VBAT 后备星历热启动(≤1s, 灵敏度 -156dBm，比冷启动 -148 高 8dB)。
    // PCAS04 位掩码: GPS=1 BDS=2 GLONASS=4 Galileo=8 → 15=四系统全开(QZSS 随 GPS L1)。
    {
        const uint8_t GPS_CFG_VER = 2;     // 改配置时 +1，强制下次开机重配一次
        Preferences gpsPrefs;
        gpsPrefs.begin("gps", false);
        if (gpsPrefs.getUChar("cfgver", 0) != GPS_CFG_VER) {
            delay(200);
            gpsSerial.print("$PCAS10,9*15\r\n");   // 一次性出厂启动 + 使能串口&射频
            delay(500);
            gpsSerial.print("$PCAS04,15*2D\r\n");  // GPS(+QZSS)+BDS+GLONASS+Galileo 全开
            delay(100);
            gpsSerial.print("$PCAS00*01\r\n");     // 存入模块 flash，掉电不丢
            delay(200);
            gpsPrefs.putUChar("cfgver", GPS_CFG_VER);
            Serial.println("[GPS] one-time config: RF + GPS/QZSS+BDS+GLONASS+Galileo, saved");
        } else {
            Serial.println("[GPS] config persisted -> hot start (no re-config this boot)");
        }
        gpsPrefs.end();
    }
#endif  // !GNSS_TIMESHARE GPS init

    // ── Initial battery / power read ─────────────────────────────────────────
    readBattery();
    refreshBatLed();
    refreshPowerLed();

#if GNSS_TIMESHARE
    // ── LCD（PORT.C）：早点起，初始化期间显示进度 ─────────────────────────────
    lcdInit();
    displayOn = lcdOK;            // 开机即亮（含整个 init 过程）
    lcdBootMsg("starting...");
#endif

    // ── Timestamps ───────────────────────────────────────────────────────────
    tBoot = tLastBatRead = tLastLed = tLastUsbChk = tLastBtn = tLastBlink = tLastGpsLog = millis();
    tLastSend   = millis();   // don't send immediately — wait for GPS fix first
    tLastPwrLog = millis();   // first power-log sample one PWRLOG_MS from now
    Serial.printf("[PWRLOG] %u entries buffered — type 'log' to dump, 'logclear' to erase\n",
                  pwrlogCount);

    // ── CatM init (may take 10-30 s; LED yellow during init) ─────────────────
#if GNSS_TIMESHARE
    lcdBootMsg("LTE init...");
#endif
    catmReady = catmInit();
    Serial.printf("[CM] init %s\n", catmReady ? "OK" : "FAIL");

    // ── Time sync: verify network + set ESP32 RTC ────────────────────────────
    if (catmReady) {
        catmTimeSynced = catmSyncTime();
        Serial.printf("[CM] time sync %s\n", catmTimeSynced ? "OK" : "FAIL (will retry)");
    }
    tLastSyncAttempt = millis();

#if GNSS_TIMESHARE
    // 配置B：对时完成后进入 GNSS 跟踪模式（让出网络给 GNSS）。之后 loop 轮询 CGNSINF，
    // 到 beacon 时 sendGpsData() 临时切回 LTE 发包再切回。
    if (catmReady) {
        lcdBootMsg("GNSS init...");
        Serial.println("[CM] enter GNSS tracking mode (CNACT=0,0 -> CGNSPWR=1)");
        catmCmd("AT+CNACT=0,0", 5000);
        catmCmd("AT+CGNSPWR=1", 3000);
        gnssTracking  = true;
        gpsState      = GS_SEARCHING;
        catmState     = CM_READY;
        refreshCatmLed();
    }
    // 开机先亮 30s（相当于按了一下），随后自动息屏。
    displaySetOn(true);
#endif

    Serial.println("[BOOT] Setup complete\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// GNSS↔LTE 分时切换测速（评估 SIM7080G 二合一模块是否可行；串口命令 gnsstest 触发）
// 前提：PORT.A 插的是 Unit CatM GNSS（带 GNSS 天线，天线见天）。复用 catmCmd / 已就绪的模块。
// 测的关键数：切回 GNSS 后到再次 fix=1 的秒数（"秒切回来"成立与否）。
// ═══════════════════════════════════════════════════════════════════════════

// 轮询 CGNSINF 直到 fix=1 或超时。带出可见星数与最强 CN0(dBHz)。
// +CGNSINF: run(0),fix(1),utc(2),lat,lon,alt,spd,course,fixmode,res,HDOP(10),
//           PDOP,VDOP,res,sats_view(14),GPSused(15),GLOused(16),res,C/N0max(18)
static bool gnssWaitFix(uint32_t timeoutMs, int* sats, int* cn0) {
    *sats = 0; *cn0 = 0;
    uint32_t start = millis();
    while (millis() - start < timeoutMs) {
        String r = catmCmd("AT+CGNSINF", 3000);
        int p = r.indexOf("+CGNSINF:");
        if (p >= 0) {
            int e = r.indexOf('|', p);                 // 字段段止于下一个 '|'
            String body = (e > p) ? r.substring(p + 9, e) : r.substring(p + 9);
            // 逐字段切分（容忍空字段）
            int fix = 0, sv = 0, cn = 0, idx = 0, from = 0;
            for (int i = 0; i <= body.length(); i++) {
                if (i == body.length() || body[i] == ',') {
                    String f = body.substring(from, i); f.trim();
                    if (idx == 1)  fix = f.toInt();
                    if (idx == 14) sv  = f.toInt();
                    if (idx == 18) cn  = f.toInt();
                    idx++; from = i + 1;
                }
            }
            *sats = sv; *cn0 = cn;
            if (fix == 1) return true;
        }
        delay(800);
    }
    return false;
}

// PORT.A 波特率扫描：逐个波特率发 AT，看哪个能收到 OK。区分"波特率不对"vs"接反/没发"。
static void atScan() {
    const uint32_t bauds[] = {115200, 9600, 19200, 38400, 57600, 230400, 460800, 921600};
    Serial.println("[AS] 扫描 PORT.A(G16/G15) 各波特率找 AT 应答…");
    bool found = false;
    for (uint32_t b : bauds) {
        Serial2.end();
        Serial2.begin(b, SERIAL_8N1, CATM_RX_PIN, CATM_TX_PIN);
        delay(150);
        while (Serial2.available()) Serial2.read();    // 清空
        String r;
        for (int k = 0; k < 3; k++) {                  // 发 3 次 AT（兼顾 autobaud 同步）
            Serial2.print("AT\r\n");
            uint32_t t0 = millis();
            while (millis() - t0 < 350) {
                while (Serial2.available()) {
                    char c = Serial2.read();
                    if (c == '\r' || c == '\n') c = '|';
                    r += c;
                }
                delay(2);
            }
        }
        bool ok = r.indexOf("OK") >= 0;
        Serial.printf("[AS] %lu bps -> %s  原始(%d字节): %s\n",
                      (unsigned long)b, ok ? "✅有 OK！" : "无 OK",
                      r.length(), r.length() ? r.c_str() : "(空，无任何字节)");
        if (ok) { Serial.printf("[AS] ★★ 模块在 %lu bps 应答！\n", (unsigned long)b); found = true; }
    }
    Serial2.end();
    Serial2.begin(CATM_BAUD, SERIAL_8N1, CATM_RX_PIN, CATM_TX_PIN);
    if (!found)
        Serial.println("[AS] 所有波特率都无 OK：若各档全空=收不到模块任何字节(RX/TX 接反或模块没往外发)；"
                       "若有乱码字节=波特率族不对。");
    Serial.println("[AS] 扫描结束，已恢复 115200");
}

static void gnssSwitchTest() {
    if (!catmReady) { Serial.println("[GT] CatM 未就绪，先等模块初始化完再测"); return; }
    Serial.println("[GT] === GNSS<->LTE 分时切换测速（需 Unit CatM GNSS + 天线见天）===");

    // 1) 首次进 GNSS：先停 PDP（礼貌让位）→ 开 GNSS → 等首个 fix
    Serial.println("[GT] 进 GNSS: CNACT=0,0 -> CGNSPWR=1，等首次定位…");
    catmCmd("AT+CNACT=0,0", 5000);
    catmCmd("AT+CGNSPWR=1", 3000);
    uint32_t t0 = millis();
    int sats = 0, cn0 = 0;
    if (!gnssWaitFix(120000, &sats, &cn0)) {
        Serial.println("[GT] 120s 内未定位（天线没见天 / 无 GNSS 天线？）测试中止");
        catmCmd("AT+CGNSPWR=0", 3000);
        catmCmd("AT+CNACT=0,1", 12000);
        return;
    }
    Serial.printf("[GT] 首次定位 %.1fs  可见星=%d  CN0=%d dBHz\n",
                  (millis() - t0) / 1000.0, sats, cn0);

    // 2) 重复 N 次：切到 LTE → 精确测"真能发数据"(PDP active + 可用 IPv4) → 切回 GNSS
    const int N = 8;
    uint32_t racMin = 0xFFFFFFFF, racMax = 0, racSum = 0; int okR = 0;
    uint32_t pdpMin = 0xFFFFFFFF, pdpMax = 0, pdpSum = 0; int okP = 0;
    for (int i = 1; i <= N; i++) {
        catmCmd("AT+CGNSPWR=0", 3000);                 // 出 GNSS
        uint32_t a = millis();
        // 切走瞬间是否还在注册（guide 称 GNSS 窗口期 CEREG 不掉 → 只需重激活 PDP，快）
        String cr = catmCmd("AT+CEREG?", 2000);
        int cci = cr.indexOf("+CEREG:"); int cm = (cci >= 0) ? cr.indexOf(',', cci) : -1;
        char stat = (cm >= 0) ? cr.charAt(cm + 1) : '?';
        catmCmd("AT+CNACT=0,1", 12000);                // 重激活 PDP
        // 轮询 CNACT? 直到拿到点分 IPv4 = 真正可发数据的时刻
        uint32_t tPdp = 0; bool pdpOk = false; uint32_t s2 = millis();
        while (millis() - s2 < 25000) {
            if (catmHasIPv4(catmCmd("AT+CNACT?", 3000))) { tPdp = millis() - a; pdpOk = true; break; }
            delay(400);
        }
        delay(1500);                                    // 模拟上传占用
        catmCmd("AT+CNACT=0,0", 5000);                 // 再让位给 GNSS
        uint32_t b = millis();
        catmCmd("AT+CGNSPWR=1", 3000);                 // 切回 GNSS
        bool got = gnssWaitFix(60000, &sats, &cn0);
        uint32_t tRac = millis() - b;
        if (got)   { racSum += tRac; okR++; if (tRac < racMin) racMin = tRac; if (tRac > racMax) racMax = tRac; }
        if (pdpOk) { pdpSum += tPdp; okP++; if (tPdp < pdpMin) pdpMin = tPdp; if (tPdp > pdpMax) pdpMax = tPdp; }
        Serial.printf("[GT] #%d  CEREG=%c  LTE可发=%s  切回定位=%s  星=%d\n",
                      i, stat,
                      pdpOk ? (String(tPdp / 1000.0, 1) + "s").c_str() : "超时(>25s)",
                      got   ? (String(tRac / 1000.0, 1) + "s").c_str() : "超时(>60s)",
                      sats);
    }
    if (okP) Serial.printf("[GT] LTE真正可发(PDP+IPv4): 最快 %.1fs / 平均 %.1fs / 最慢 %.1fs (%d/%d)\n",
                           pdpMin / 1000.0, (pdpSum / okP) / 1000.0, pdpMax / 1000.0, okP, N);
    if (okR) Serial.printf("[GT] 切回定位: 最快 %.1fs / 平均 %.1fs / 最慢 %.1fs (%d/%d)\n",
                           racMin / 1000.0, (racSum / okR) / 1000.0, racMax / 1000.0, okR, N);
    catmCmd("AT+CGNSPWR=0", 3000);
    catmCmd("AT+CNACT=0,1", 12000);                    // 收尾：恢复网络
    Serial.println("[GT] === 测试结束，网络已恢复 ===");
}

// ═══════════════════════════════════════════════════════════════════════════
// loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

#if GNSS_TIMESHARE
    // 配置B：无 PORT.C 模块。GNSS 跟踪模式时定期轮询 CGNSINF 更新 liveFix。
    if (catmReady && gnssTracking && now - tLastGnssPoll >= 1500) {
        tLastGnssPoll = now;
        pollGnssIntoLiveFix();
    }
#else
    // ── Feed GPS parser (highest priority, runs every iteration) ─────────────
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
        // Raw NMEA dump for the first GPS_RAW_DUMP_MS — helps diagnose
        // whether the module sees any satellites (look for $GNGSV SNR values).
        if (now - tBoot < GPS_RAW_DUMP_MS)
            Serial.write(c);
        // 装配整行 NMEA → 喂 GNSS 信号诊断（CN0/星座/天线，进电量日志）
        if (c == '\n' || c == '\r') {
            if (nmeaLen) { nmeaLine[nmeaLen] = 0; gnssDiagLine(nmeaLine); nmeaLen = 0; }
        } else if (nmeaLen < sizeof(nmeaLine) - 1) {
            nmeaLine[nmeaLen++] = c;
        } else {
            nmeaLen = 0;   // 行超长，丢弃
        }
    }
#endif

    // ── GPS state machine ─────────────────────────────────────────────────────
    updateGps();

    // ── Blink ticker ──────────────────────────────────────────────────────────
    if (now - tLastBlink >= BLINK_HALF_MS) {
        tLastBlink = now;
        blinkPhase = !blinkPhase;
    }

    // ── LED refresh (GPS + battery + power indicator, rate-limited) ─────────
    if (now - tLastLed >= LED_REFRESH_MS) {
        tLastLed = now;
        // Re-read external-power flag here so the CHG_LED responds at 300 ms
        // rather than waiting for the 15 s battery read cycle.
        hasExtPower = (phRd8(REG_PSUP) != 0);
        refreshGpsLed();
        refreshBatLed();
        refreshPowerLed();
    }

#if GNSS_TIMESHARE
    // ── LCD：超时自动息屏 + 亮屏时定期刷新状态 ───────────────────────────────
    if (displayOn) {
        if ((int32_t)(now - tDisplayOff) >= 0) displaySetOn(false);
        else if (now - tLastLcdDraw >= LCD_DRAW_MS) lcdDrawStatus();
    }
#endif

    // ── USB current / LED check ───────────────────────────────────────────────
    if (now - tLastUsbChk >= USB_CHK_MS) {
        tLastUsbChk = now;
        refreshUsbLed();
    }

    // ── Battery read ──────────────────────────────────────────────────────────
    if (now - tLastBatRead >= BAT_READ_MS) {
        tLastBatRead = now;
        readBattery();
    }

    // ── Button scan ───────────────────────────────────────────────────────────
    if (now - tLastBtn >= BTN_SCAN_MS) {
        tLastBtn = now;
        checkButtons();
    }

    // ── Power log sample (RTC ring) ──────────────────────────────────────────
    if (now - tLastPwrLog >= PWRLOG_MS) {
        tLastPwrLog = now;
#if GNSS_TIMESHARE
        gnssSampleNmea();   // 先抓 NMEA 填 CN0/星座（配置B 的 CGNSINF 不带这些）
#endif
        pwrlogAppend();
    }

    // ── USB serial command console (log / logclear / help) ───────────────────
    checkSerialCommands();

#if !GNSS_TIMESHARE
    // ── CatM: time sync retry until successful (every 60 s) ─────────────────
    // catmSyncTime() may fail at boot if PDP isn't ready yet; retry here.
    // 配置B 跳过：GNSS 跟踪期跑 SHCONN 会和 GNSS 冲突；B 在开机时已对过一次时。
    if (catmReady && !catmTimeSynced && now - tLastSyncAttempt >= 60000) {
        tLastSyncAttempt = now;
        catmTimeSynced = catmSyncTime();
    }

    // ── CatM: one-shot eDRX granted-cycle readback (Guide §8.3) ──────────────
    // A successful time sync means the module is attached + PDP active, so the
    // network has finished negotiating eDRX — only now does CEDRXRDP report the
    // real granted cycle (the 3rd field), not 0. Logged once for diagnostics.
    if (catmReady && catmTimeSynced && !edrxChecked) {
        edrxChecked = true;
        Serial.printf("[CM] eDRX granted: %s\n", catmCmd("AT+CEDRXRDP", 3000).c_str());
    }
#endif

    // ── CatM: manual GPS upload (top button short press) ─────────────────────
    if (manualSendReq) {
        manualSendReq = false;
        if (!catmReady) {
            Serial.println("[CM] manual upload skipped — CatM not ready");
        } else if (gpsState != GS_FIX_GOOD) {
            Serial.println("[CM] manual upload skipped — no GPS fix");
        } else {
            Serial.println("[CM] manual upload (button)");
            sendGpsData(true);
            recordAnchor();               // treat as a beacon: reset anchor + timer
            decayInterval = DECAY_START_MS;
        }
    }

    // ── CatM: forced bench upload (top-button long-press) ────────────────────
    // Diagnostic only: runs the full sendGpsData() POST path even with no GPS
    // fix, so the "CatM red only outdoors" failure can be reproduced on the
    // bench. Coordinates will be whatever GPS last had (0,0 if never fixed).
    if (forceSendReq) {
        forceSendReq = false;
        if (!catmReady) {
            Serial.println("[CM] FORCED upload skipped — CatM not ready");
        } else {
            Serial.println("[CM] === FORCED bench upload (long-press, ignoring GPS fix) ===");
            sendGpsData(false);   // bench/diagnostic: don't pollute the track queue
            recordAnchor();
            decayInterval = DECAY_START_MS;
        }
    }

#if !GNSS_TIMESHARE
    // ── CatM 恢复：红灯但 GPS 没有定位时 ───────────────────────────────────────
    // 下面的 beacon 发送整块被 GS_FIX_GOOD 门控，因此一次失败留下的红灯只能在
    // “有定位”的前提下重试 / 重附着 / 补发积压点。一旦红灯期间又丢了定位（进楼、
    // 城市峡谷、地下、回到室内），恢复逻辑就永远不触发，红灯无限锁死——
    // 2026-06-13 实测：盲区丢定位后红灯卡死 30 分钟。这里把恢复做成不依赖定位：
    // 哪怕没定位也让模组有机会恢复、并把积压队列发出去。有定位的情况完全不变
    // （仍由下面 beaconDue 的 "retry" 处理），所以两条路径不会重复发送。
    // 配置B 跳过：模块在 GNSS 跟踪模式，恢复/重附着在 sendGpsData 切 LTE 时做。
    if (catmReady && catmState == CM_ERR && gpsState != GS_FIX_GOOD
            && millis() - tLastCatmRecover >= CATM_FAIL_RETRY_MS) {
        tLastCatmRecover = millis();
        // 连续失败够多 → 强制 IPv4 重附着（针对 IPv6-only PDP；SH 锁死另由
        // catmPostBody 内部的 CFUN=1,1 自愈，不在此处理）
        if (catmFailStreak >= CATM_FAIL_REATTACH) {
            Serial.printf("[CM] recover (no fix): %u fails -> IPv4 re-attach\n", catmFailStreak);
            catmForceIPv4();
            catmFailStreak = 0;
        }
        // ⚠️ 旧逻辑「catmCheckNet() 通就清红」是假恢复：SH 栈锁死时 PDP/IPv4 完全
        // 正常但 SHCONN 发不出，会把红灯清掉却一条没发出去（静默丢数据）——
        // 2026-06-19 实测坐实。改成：只有积压点「真的发出去了」（队列变短）才清红；
        // 没有积压可发时无法验证 SH 是否真通，红灯保持到下次成功发送，绝不假装恢复。
        if (trackCount > 0) {
            uint16_t before = trackCount;
            trackFlush();             // catmPostBody 会在 SH 锁死时 CFUN=1,1 自愈后重发
            if (trackCount < before) {
                Serial.println("[CM] recover (no fix): 积压补发成功 -> 清红");
                catmState = CM_READY;
                refreshCatmLed();
            } else {
                catmFailStreak++;     // 仍发不出，保持红，下个周期再试
            }
        }
    }
#endif

#if GNSS_TIMESHARE
    // ── 配置B 红灯处理（不依赖定位）─────────────────────────────────────────────
    // 红灯=发包失败；失败的 beacon 会把点入队(trackEnqueue)，所以红灯≈“有积压待发”。
    // 原则(用户 06-19“只做有必要的事”)：只有“确有积压要发”时才临时切 LTE 探网；
    // 没有待发数据就别折腾网络，安心追 GPS，并把无意义的红灯清回蓝。
    if (catmReady && gnssTracking && catmState == CM_ERR && gpsState != GS_FIX_GOOD) {
        if (trackCount == 0) {
            // 无积压 → 红灯没有意义（没东西要发）→ 清为蓝，继续追踪，不切网
            catmState = CM_READY;
            refreshCatmLed();
        } else if (millis() - tLastCatmRecover >= CATM_FAIL_RETRY_MS) {
            tLastCatmRecover = millis();
            Serial.printf("[CM] B 红灯恢复：%u 条积压待发，切 LTE 探网…\n", trackCount);
            catmCmd("AT+CGNSPWR=0", 3000);            // 出 GNSS，让出射频
            gnssTracking = false;
            // ★ 切完射频不能立刻判网：等模组重新附着（这正是原来一次性 CEREG 误判→
            //   永远红灯的根因）。已注册则秒过；最多等 ~20s 给它重新搜网。
            bool registered = catmWaitReg(20000);
            bool back = false;
            if (registered) {
                if (catmFailStreak >= CATM_FAIL_REATTACH) { catmForceIPv4(); catmFailStreak = 0; }
                catmCmd("AT+CNACT=0,1", 12000);
                back = catmHasIPv4(catmCmd("AT+CNACT?", 3000));
            }
            if (back) {
                Serial.println("[CM] B 恢复：网络回来 → 补发积压 + 清红");
                catmState = CM_READY; catmFailStreak = 0;
                trackFlush();                          // 仍在 LTE，顺手补发
            } else {
                catmFailStreak++;                      // 仍无网，保持红
            }
            catmCmd("AT+CNACT=0,0", 5000);            // 让位
            catmCmd("AT+CGNSPWR=1", 3000);            // 切回 GNSS 继续跟踪
            gnssTracking  = true;
            tLastGnssPoll = 0;
            refreshCatmLed();
        }
    }
#endif

    // ── CatM: adaptive GPS upload (SmartBeaconing + decay) ───────────────────
    // GPS is sampled continuously; only the send cadence adapts to motion.
    // sendGpsData() blocks ~15-25 s; the GPS RX buffer absorbs NMEA meanwhile.
    if (catmReady && gpsState == GS_FIX_GOOD) {
        const char* why = nullptr;
        bool stopped = false;
        if (beaconDue(&why, &stopped)) {
            // Commit anchor/timer BEFORE the long blocking send so a failed POST
            // waits a full interval instead of hammering the modem every loop.
            bool grew = stopped && (strcmp(why, "interval") == 0);
            recordAnchor();
            if (grew) {                              // stopped keepalive → decay
                decayInterval *= 2;
                if (decayInterval > DECAY_MAX_MS) decayInterval = DECAY_MAX_MS;
            } else {                                 // any motion → reset decay
                decayInterval = DECAY_START_MS;
            }
            Serial.printf("[CM] beacon (%s, next stopped=%lus)\n",
                          why, (unsigned long)(decayInterval / 1000));
            sendGpsData(true);
        }
    }

    // Small yield — keeps the RTOS watchdog happy and saves a tiny bit of power
    delay(5);
}
