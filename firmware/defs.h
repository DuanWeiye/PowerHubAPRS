// defs.h — 引脚/寄存器/枚举/调参常量/结构体 + 全部函数前置声明
//
// 多 .ino 单编译单元模型：Arduino 把同目录所有 .ino 按「主文件在前、其余字母序」
// 拼成一份再编译。所以本头里的 #define/enum/const/struct 和前置声明对所有 .ino 可见，
// 各 .ino 共享全局作用域、static 函数跨文件可见，无需写 extern。
// 配置差异逻辑整体分到 config_a.ino(#if !GNSS_TIMESHARE) / config_b.ino(#if GNSS_TIMESHARE)，
// 未启用的一份编译成空；setup()/loop() 通过下面的 config* 钩子调用，主流程里没有 #if。
//
// 注意：本头需在 GNSS_TIMESHARE 与 config.h 之后被 include（见 firmware.ino 顶部顺序）。
#pragma once
#include <Arduino.h>
#include "flushlogic.h"   // 配置B 存转日志的共享常量 + flushDue()（无 Arduino 依赖）

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

// ── Store-and-forward ─────────────────────────────────────────────────────────
// 配置A/B 统一用 flashlog.ino 的 LittleFS 段日志做存转（断电不丢）。批量大小/段大小
// 见 flushlogic.h 的 FL_BATCH / FL_SEG_POINTS / FL_BODYLEN。（旧的 RAM 环形队列已弃用。）

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
// Enums
// ═══════════════════════════════════════════════════════════════════════════
enum GpsState : uint8_t {
    GS_DETECTING,   // startup, waiting for any NMEA data
    GS_NO_MODULE,   // timeout — no data received, leave LED off
    GS_INIT_OK,     // first valid sentence — brief blue flash
    GS_SEARCHING,   // module active, no valid fix yet  → yellow
    GS_FIX_GOOD,    // valid fix with HDOP < threshold   → green
    GS_INIT_FAIL    // data received but all checksums bad → red
};

enum CatmState : uint8_t {
    CM_OFF,       // not initialized
    CM_INIT,      // module initializing (yellow)
    CM_READY,     // network ready, idle (blue)
    CM_SENDING,   // HTTP POST in progress (white)
    CM_OK,        // last POST succeeded (green)
    CM_ERR        // last operation failed (red)
};

// ═══════════════════════════════════════════════════════════════════════════
// Structs
// ═══════════════════════════════════════════════════════════════════════════

// 统一位置源：配置A 由 TinyGPS++ 喂，配置B 由 CGNSINF 轮询喂。
// 上层一律通过 fixXxx() 访问器读它，两种来源对上层完全透明。
struct LiveFix {
    bool     valid       = false;   // 有有效定位
    double   lat = 0, lon = 0;
    float    altM = 0, spdKmh = 0, courseDeg = -1, hdop = 25.5f;
    bool     courseValid = false;
    uint8_t  sats        = 0;
    uint32_t tMs         = 0;       // 上次更新 millis（判新鲜度）
};

// Store-and-forward track queue point.  Ring buffer: overwrites oldest when full.
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

// Power log entry (RTC slow memory ring; survives reset, lost on full power-off).
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

#if GNSS_TIMESHARE
// ── 配置B LCD 显示参数（Unit LCD 1.14"）──────────────────────────────────────
static const uint32_t DISPLAY_ON_MS  = 30000;   // 短按/开机亮屏时长（30s）
static const uint32_t LCD_DRAW_MS    = 500;     // 亮屏时状态重绘周期（<1s：秒钟不漏跳）
static const uint8_t  LCD_BRIGHTNESS = 110;     // 亮屏亮度（0-255）
#endif

// ═══════════════════════════════════════════════════════════════════════════
// 前置声明（全部函数）。多 .ino 单 TU 拼接时与定义顺序无关，故在此统一声明，
// 既消除"先用后定义"的隐患，也免去依赖 Arduino 自动原型生成对 static 的处理。
// ═══════════════════════════════════════════════════════════════════════════

// ── powerhub.ino ──
static void     phRd(uint8_t reg, uint8_t* buf, uint8_t n);
static void     phWr(uint8_t reg, const uint8_t* buf, uint8_t n);
static uint8_t  phRd8(uint8_t reg);
static void     phWr8(uint8_t reg, uint8_t v);
static bool     phInit();
static void     phPower(uint8_t port, bool on);
static void     phLedSet(uint8_t idx, uint32_t rgb);
static void     phLedBright(uint8_t idx, uint8_t br);
static uint16_t phVolt(uint8_t mon);
static int16_t  phCurr(uint8_t mon);
static bool     phBtn(uint8_t bitPos);
static int      voltToPercent(uint16_t mv);
static void     readBattery();
static void     refreshGpsLed();
static void     refreshBatLed();
static void     refreshPowerLed();
static void     refreshCatmLed();
static void     refreshUsbLed();

// ── pwrlog.ino ──
static void gnssDiagLine(const char* s);
static void pwrlogInit();
static void pwrlogClear();
static void pwrlogAppend();
static void pwrlogDump();
static void checkSerialCommands();

// ── catm.ino ──
static String catmCmd(const String& cmd, unsigned long timeout);
static bool   catmHasIPv4(const String& cnact);
static void   catmForceIPv4();
static bool   catmInit();
static bool   catmCheckNet();
static void   catmDiagConn();
static bool   catmSyncTime();
static bool   catmSHRecover();
static bool   catmSHOpen(int bodyCap);                 // 开 HTTPS 会话（含撞锁 CFUN=1,1 自愈）
static int    catmSHReq(const char* body, int bodyLen);// 在已开会话上发一个 POST
static void   catmSHClose();                           // 关会话
static int    catmPostBody(const char* body, int bodyLen, int bodyCap);  // 开+发+关（单发）

// ── track.ino ──
static int     fmtPoint(char* buf, int cap, const TrackPoint& p);
static void    buildTrackPoint(TrackPoint& p);
static void    recordAnchor();
static bool    beaconDue(const char** why, bool* stopped);

// ── buttons.ino ──
static void powerSaveShutdown();
static void checkButtons();

// ── diag.ino ──
static void atScan();
static bool gnssWaitFix(uint32_t timeoutMs, int* sats, int* cn0);
static void gnssSwitchTest();

// ── 统一位置访问器（config_a.ino / config_b.ino 各一份实现）──
static bool    fixHasLoc();
static double  fixLat();
static double  fixLon();
static float   fixAltM();
static bool    fixHasSpeed();
static float   fixSpdKmh();
static bool    fixHasHdop();
static float   fixHdop();
static bool    fixHasSats();
static uint8_t fixSats();
static bool    fixHasCourse();
static float   fixCourseDeg();

// ── 配置无关的上层接口（config_a.ino / config_b.ino 各一份实现）──
static void updateGps();
static bool sendGpsData(bool queueOnFail);

// ── 配置钩子：setup()/loop() 调用，A/B 各自实现，使主流程零 #if ──
static void configSetupEarly();             // CatM UART 之后：A=开 GPS 串口+初始化 | B=LCD 初始化+开机屏
static void configSetupPreNet();            // catmInit 之前：B=显示"LTE init..." | A=空
static void configSetupPostNet();           // 对时之后：B=进 GNSS 跟踪+亮屏 | A=空
static void configLoopFeed(uint32_t now);   // loop 顶：A=喂 GPS 解析 | B=轮询 CGNSINF
static void configLoopDisplay(uint32_t now);// B=LCD 超时息屏/重绘 | A=空
static void configLoopPrePwrlog();          // 采样前：B=抓 NMEA 填 CN0/星座 | A=空
static void configLoopSync(uint32_t now);   // A=对时重试+eDRX 回读 | B=空
static void configLoopRecover(uint32_t now);// A=无定位红灯恢复 | B=flush 调度器（切LTE上传积压）
static void configOnTopShortPress();        // 顶部按钮短按：A=请求上传 | B=亮/息屏开关
static void configBeaconAction();           // 到 beacon 点的动作：A=直接发送 | B=记录到 Flash 段日志
static void configForceUpload();            // 长按大按钮：A=bench 强制发一包 | B=强制 flush 积压

// ── flashlog.ino（LittleFS 存转段日志，配置A/B 共用：A 失败兜底 / B 每点记录）──
static void flashLogBegin();
static void flashLogAppend(const TrackPoint& p);
static void flashLogCounts(uint16_t* sealedSegs, uint32_t* totalPoints);
static int  flashLogUpload();

#if GNSS_TIMESHARE
// ── flashlog.ino / config_b.ino 专有 ──
static void flashFlushViaLte(bool forced);  // 切 GNSS→LTE、上传所有封段、切回 GNSS
static void flashLogFillTest(int n);        // 台面实测：灌 n 个假点（绕过 GPS 定位）
static void flashLogClear();                // 台面实测：清空段日志
static bool catmWaitReg(uint32_t timeoutMs);
static void pollGnssIntoLiveFix();
static void gnssSampleNmea();
static void gnssNmeaTest();
static void lcdInit();
static void lcdDrawStatus();
static void lcdBootMsg(const char* msg);
static void displaySetOn(bool on);
#endif
