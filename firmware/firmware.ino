/*
 * M5Stack PowerHub — APRS Tracker
 *
 * 多文件结构（同目录所有 .ino 拼成一个编译单元；详见 defs.h 顶部说明）：
 *   firmware.ino   ← 本文件：includes / 配置开关 / 全局变量定义 / setup() / loop()
 *   defs.h           引脚·寄存器·枚举·调参常量·结构体 + 全部函数前置声明
 *   powerhub.ino     PowerHub I2C、电源/LED、电池换算、各 LED 刷新        〔共用〕
 *   catm.ino         SIM7080G AT 层：cmd/init/checkNet/syncTime/SHRecover/postBody 〔共用〕
 *   track.ino        存转队列 + 自适应 beacon 决策                          〔共用〕
 *   pwrlog.ino       RTC 电量日志 + GNSS 信号解析 + 串口命令台              〔共用〕
 *   buttons.ino      按键状态机 + 省电关机                                  〔共用〕
 *   diag.ino         现场诊断：atScan / gnssWaitFix / gnssSwitchTest        〔共用〕
 *   config_a.ino     #if !GNSS_TIMESHARE：独立 GPS 那套差异逻辑            〔仅配置A〕
 *   config_b.ino     #if  GNSS_TIMESHARE：二合一分时 + LCD 那套差异逻辑    〔仅配置B〕
 *
 * setup()/loop() 通过 config* 钩子调用配置相关逻辑，主流程里没有 #if。
 *
 * LED mapping:
 *   LED_UART_P (idx 2) = PORT.C adjacent LED  → GPS status
 *   LED_PWR_R  (idx 7) = large LED near WiFi  → battery level
 *   LED_USB_A  (idx 1) = USB-A adjacent LED   → USB power state
 *   LED_I2C_P  (idx 4) = PORT.A adjacent LED  → CatM status
 *   LED_BAT_C  (idx 5) = small-button LED     → external power
 *
 * Libraries required: TinyGPS++（配置B 另需 M5UnitLCD / M5GFX）。
 * Board: m5stack:esp32:m5stack_atoms3r（ESP32-S3 + OPI PSRAM，USB-CDC，见 build.sh）。
 */

#include <Wire.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <sys/time.h>    // gettimeofday / settimeofday for ESP32 RTC
#include <string.h>      // strcmp / memcpy
#include <math.h>        // fabsf (heading delta)
#include <Preferences.h> // NVS：GPS 一次性配置标志（让后续开机热启动）

// ── GNSS 来源选择 ────────────────────────────────────────────────────────────
//   1 = 配置B：SIM7080G 二合一(Unit CatM GNSS)内置 GNSS 与 LTE 分时共用 PORT.A，
//             腾出 PORT.C 给 LCD。位置靠 CGNSINF 轮询；发包时 GNSS↔LTE 切换。
//   0 = 配置A：PORT.C 独立 ATGM336H 连续 NMEA + PORT.A 的 SIM7080G 专做 4G。
// 换硬件改这一个数即可；两套逻辑分别在 config_a.ino / config_b.ino，互不影响。
#define GNSS_TIMESHARE 0

// 部署配置（APN / 服务器域名·端口·路径）抽到 config.h，不提交到 git。
// 首次编译前：复制 config.example.h 为 config.h 并填入自己的值。
#include "config.h"

#if GNSS_TIMESHARE
// 配置B：PORT.C 腾出的口接 Unit LCD 1.14"（I2C，ST7789 135x240，板载控制器 0x3E）。
// 由 M5GFX 的 M5UnitLCD 驱动；走 ESP32 I2C 端口1（端口0 被 PowerHub 占用）。
#include <M5UnitLCD.h>
#endif

// 引脚/寄存器/枚举/调参常量/结构体 + 全部前置声明（必须在 GNSS_TIMESHARE 与 config.h 之后）
#include "defs.h"

// ═══════════════════════════════════════════════════════════════════════════
// Global state（全工程唯一一份定义；多 .ino 单编译单元下各文件直接访问）
// ═══════════════════════════════════════════════════════════════════════════

static TinyGPSPlus    gps;
static HardwareSerial gpsSerial(1);   // UART1

// 统一位置源：配置A 由 gps 喂，配置B 由 CGNSINF 轮询喂。上层经 fixXxx() 访问器读它。
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
static CatmState catmState      = CM_OFF;
static bool      catmReady      = false;
static uint8_t   catmFailStreak = 0;       // consecutive sendGpsData failures (0 on success)
static bool      catmTimeSynced = false;   // true after first successful /iot/time sync
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
// 失败的 beacon 点存这里，下次成功发送时批量补发（最旧优先），住 RAM（整机重启即丢，
// 属短期断网保险）。环形缓冲：满了覆盖最旧，永不溢出。
static TrackPoint trackQueue[TRACK_QUEUE_CAP];
static uint16_t   trackHead  = 0;     // next write slot
static uint16_t   trackCount = 0;     // queued points (0..CAP)

// ── Power log (battery voltage/current history) ───────────────────────────────
// 存 RTC 慢速内存，软/掉压/看门狗复位后仍在（整机断电 0xE0 才丢，那本身就是"已关机"信号）。
// 环形缓冲，永不溢出。用 USB 串口 log / logclear 命令拉取/清除。
RTC_NOINIT_ATTR static uint32_t    pwrlogMagic;
RTC_NOINIT_ATTR static uint16_t    pwrlogHead;   // next write slot (0..CAP-1)
RTC_NOINIT_ATTR static uint16_t    pwrlogCount;  // valid entries (0..CAP)
RTC_NOINIT_ATTR static PwrLogEntry pwrlogBuf[PWRLOG_CAP];

// ═══════════════════════════════════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(300);   // let USB-serial settle before first print
    Serial.println("\n=== M5Power APRS Tracker ===");

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
    phPower(PC_UART,    true);    // PORT.C — GPS module (配置B：LCD) power
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

    // 配置相关早期初始化：A=开 GPS 串口+初始化模块 | B=起 LCD 显示进度
    configSetupEarly();

    // ── Initial battery / power read ─────────────────────────────────────────
    readBattery();
    refreshBatLed();
    refreshPowerLed();

    // ── Timestamps ───────────────────────────────────────────────────────────
    tBoot = tLastBatRead = tLastLed = tLastUsbChk = tLastBtn = tLastBlink = tLastGpsLog = millis();
    tLastSend   = millis();   // don't send immediately — wait for GPS fix first
    tLastPwrLog = millis();   // first power-log sample one PWRLOG_MS from now
    Serial.printf("[PWRLOG] %u entries buffered — type 'log' to dump, 'logclear' to erase\n",
                  pwrlogCount);

    configSetupPreNet();      // B：屏显 "LTE init..." | A：空

    // ── CatM init (may take 10-30 s; LED yellow during init) ─────────────────
    catmReady = catmInit();
    Serial.printf("[CM] init %s\n", catmReady ? "OK" : "FAIL");

    // ── Time sync: verify network + set ESP32 RTC ────────────────────────────
    if (catmReady) {
        catmTimeSynced = catmSyncTime();
        Serial.printf("[CM] time sync %s\n", catmTimeSynced ? "OK" : "FAIL (will retry)");
    }
    tLastSyncAttempt = millis();

    configSetupPostNet();     // B：进 GNSS 跟踪 + 亮屏 | A：空

    Serial.println("[BOOT] Setup complete\n");
}

// ═══════════════════════════════════════════════════════════════════════════
// loop
// ═══════════════════════════════════════════════════════════════════════════

void loop() {
    uint32_t now = millis();

    // 喂位置源（最高优先级）：A=喂 GPS 解析器 | B=轮询 CGNSINF
    configLoopFeed(now);

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

    // 配置B：LCD 超时息屏 + 亮屏时定期重绘（A 为空）
    configLoopDisplay(now);

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
        configLoopPrePwrlog();   // B：先抓 NMEA 填 CN0/星座（A 为空，GPS 流已带 GSV）
        pwrlogAppend();
    }

    // ── USB serial command console (log / logclear / sendtest / at... / help) ──
    checkSerialCommands();

    // 配置A：对时重试 + eDRX 回读（B 为空）
    configLoopSync(now);

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

    // 配置A：无定位红灯恢复 | 配置B：红灯处理（均不依赖定位）
    configLoopRecover(now);

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
