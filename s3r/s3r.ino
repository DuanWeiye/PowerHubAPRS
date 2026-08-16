/*
 * S3R — AtomS3R GNSS 协处理器（配置A 提精度硬件链的中间节点）
 *
 * 接线：PowerHub PORT.C(蓝,G1/G2) ↔ S3R 自带 Grove 口(G1/G2)
 *       ABC Base 蓝口 PORT.C(G5/G6) ↔ GPS(ATGM336H)
 *       供电整链来自 PowerHub PORT.C 的 5V（phPower(PC_UART) 可整链断电 = 硬件看门狗）
 *
 * 多文件结构（同目录 .ino 拼成一个编译单元，见 defs.h）：
 *   s3r.ino     ← 本文件：includes / 全局变量 / setup() / loop() / 串口泵与极性探测
 *   defs.h        引脚·常量·枚举 + 全部前置声明
 *   fuse.ino      融合 v1：静止锁存 + 短断档 DR 桥接 + GNSS 逃生门（NMEA 改写转发）
 *   imu.ino       BMI270 100Hz 采样 + 姿态无关静止检测/零偏自校/航向角速率
 *   imulog.ino    IMU/GNSS/事件 LittleFS 环形日志 + $S3R,IMUDUMP 拉取（ESKF 调参原料）
 *   ota.ino       UART-OTA 接收器 + A/B 槽启动确认/回滚 + $S3R 协议分发
 *   parse.ino     NMEA 旁路解析（TinyGPS++ 喂入 + GSV/TXT 星座/CN0/天线诊断）
 *   screen.ino    128×128 自带屏状态显示（M5GFX，按键唤醒，默认超时息屏）
 *   nmea_rw.h     NMEA 解析/改写/重建纯 C 核心（test/ 下主机单测）
 *
 * Board: m5stack:esp32:m5stack_atoms3r（真 AtomS3R；PSRAM=opi, USB-CDC, CPU 80MHz,
 *        PartitionScheme=default_8MB —— app0/app1 双 OTA 槽，UART-OTA 的前提），见 build.sh。
 * Libraries: TinyGPS++ / M5GFX / M5Unified(只用 BMI270_Class+In_I2C，不调 M5.begin)。
 */

#include <TinyGPS++.h>
#include <Preferences.h>
#include "defs.h"

// ═══════════════════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════════════════

static HardwareSerial gpsSerial(1);    // UART1 ← GPS
static HardwareSerial linkSerial(2);   // UART2 ← PowerHub
static TinyGPSPlus    gps;
static Preferences    prefs;           // NVS "s3r"：极性记忆（OTA 状态另用 "s3rota"）

// ── 端口角色/极性（开机 bootProbePorts 自动识别，NVS 记忆，探测失败用默认）──
static int      gpsRxPin     = GPS_RX_DEFAULT;
static int      gpsTxPin     = GPS_TX_DEFAULT;
static int      linkRxPin    = LINK_RX_DEFAULT;
static int      linkTxPin    = LINK_TX_DEFAULT;
static bool     gpsPinLocked = false;  // 收到数据后锁定并存 NVS
static uint32_t tGpsSwap     = 0;      // 上次换脚时刻（兜底极性轮换用）
static uint8_t  gpsSwapCount = 0;      // 已换脚次数（换满仍无数据 → GS_NO_MODULE）

// ── GPS 状态（供屏显）──
static GpsState gpsState     = GS_DETECTING;
static uint32_t gpsChars     = 0;      // 收到的 GPS 字节总数
static uint32_t tLastGpsByte = 0;
static uint32_t tSatsZeroSince = 0;    // 可见星归零起始时刻（ANT? 判据）

// ── GNSS 信号诊断（GSV/TXT 自解析，移植自主固件 pwrlog.ino）──
// 槽位: 0=GPS 1=GLONASS 2=BDS 3=Galileo 4=QZSS 5=SBAS/其它
static uint8_t gnssInView[6] = {0};
static uint8_t gnssCN0[6]    = {0};
static uint8_t gnssAccCN0[6] = {0};
static uint8_t gnssAnt       = 0;      // 天线: 0 未知 / 1 OK / 2 开路 / 3 短路
static char    nmeaLine[100];
static uint8_t nmeaLen       = 0;

// ── 链路/USB 行装配（$S3R 协议拦截；链路侧其余行转发给 GPS）──
static char    linkLine[120];
static uint8_t linkLen       = 0;
static char    usbLine[120];
static uint8_t usbLen        = 0;

// ── 统计 ──
static uint32_t gpsByteAcc   = 0;      // 本统计周期字节数
static uint32_t gpsBps       = 0;      // 上个周期的 GPS 字节速率
static uint32_t tLastStats   = 0;

// ── 屏显/按键 ──
static bool     scrOK        = false;
static bool     displayOn    = false;
static uint32_t tDisplayOff  = 0;
static uint32_t tLastDraw    = 0;
static uint32_t tLastBtn     = 0;
static bool     btnPrev      = false;
static uint32_t tBtnPress    = 0;
static bool     btnOffFired  = false;

// ── OTA ──
static bool     otaPending   = false;  // 本次启动的固件尚待 PING 确认（屏显 PENDING）

// ═══════════════════════════════════════════════════════════════════════════
// setup
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    // 启动确认/回滚守卫必须最先跑：新 OTA 固件若有致命 bug，这里是它唯一必经的代码。
    otaBootGuard();

    // USB-CDC 接收缓冲加大：默认 256B，OTA 的 1KB 数据块会被截丢
    Serial.setRxBufferSize(4096);
    Serial.begin(115200);
    setCpuFrequencyMhz(CPU_MHZ);       // 80 = USB 活着的下限，也是本负载的省电最优点
    delay(300);
    Serial.printf("\n=== S3R GNSS coprocessor v%s ===\n", S3R_FW_VER);
    Serial.printf("[SYS] CPU @ %u MHz, partition=%s%s\n",
                  getCpuFrequencyMhz(), otaRunningPartition(),
                  otaPending ? " (PENDING confirm)" : "");

    // ── 端口角色/极性：NVS 记忆 → 开机实测探测（探到即覆盖并回存）────────────
    prefs.begin("s3r", false);
    {
        int grx = prefs.getUChar("grx", 0), lrx = prefs.getUChar("lrx", 0);
        auto pair = [](int p) { return (p == PORT_OWN_A || p == PORT_OWN_B) ? 0
                                     : (p == PORT_BASE_A || p == PORT_BASE_B) ? 1 : -1; };
        if (pair(grx) >= 0 && pair(lrx) >= 0 && pair(grx) != pair(lrx)) {
            gpsRxPin  = grx;
            gpsTxPin  = (pair(grx) == 0) ? (PORT_OWN_A + PORT_OWN_B - grx)
                                         : (PORT_BASE_A + PORT_BASE_B - grx);
            linkRxPin = lrx;
            linkTxPin = (pair(lrx) == 0) ? (PORT_OWN_A + PORT_OWN_B - lrx)
                                         : (PORT_BASE_A + PORT_BASE_B - lrx);
        }
    }
    bool probed = bootProbePorts();
    if (probed) {
        prefs.putUChar("grx", (uint8_t)gpsRxPin);
        prefs.putUChar("lrx", (uint8_t)linkRxPin);
    }
    Serial.printf("[PRB] %s  GPS RX=G%d TX=G%d | LINK RX=G%d TX=G%d\n",
                  probed ? "detected" : "no stream -> NVS/default",
                  gpsRxPin, gpsTxPin, linkRxPin, linkTxPin);

    // ── 串口 ────────────────────────────────────────────────────────────────
    // 链路 RX 缓冲给大：PowerHub 的 PCAS 行很短，但 OTA 数据块走同一条线。
    // GPS RX 缓冲 4KB：5Hz 四系统 NMEA ~4.5KB/s，透传逐字节即时转发不会积压，
    // 4KB 只为吸收屏幕整帧推送等短暂停顿。
    linkSerial.setRxBufferSize(4096);
    linkUartStart();
    gpsSerial.setRxBufferSize(4096);
    gpsUartStart();
    tGpsSwap = millis();

    // ── 屏幕 ────────────────────────────────────────────────────────────────
    screenInit();
    screenBootMsg("GPS detect...");
    displaySetOn(true);                // 开机亮 1 分钟给状态确认，之后自动息屏省电

    // ── IMU / 融合 / 环形日志 ────────────────────────────────────────────────
    imulogBegin();                     // 先起日志，IMU 失败事件才有处可记
    if (!imuInit()) imulogEvent(EV_IMU_FAIL, 0);   // 失败 → fuse 自动纯透传
    fuseInit();

    tLastStats = tLastBtn = millis();
    Serial.println("[BOOT] setup complete — passthrough+fusion active");
}

// ═══════════════════════════════════════════════════════════════════════════
// loop：三个串口泵 + 状态机 + 屏显
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    uint32_t now = millis();

    // GPS → PowerHub：整行泵（fuse.ino 装配一行 → 按需改写 → 转发；非 GGA/RMC 与
    // 一切坏行原样转发，透传语义不变），同时旁路喂解析
    while (gpsSerial.available()) {
        char c = (char)gpsSerial.read();
        gpsFeedChar(c, now);
        fusePumpChar(c);
    }

    // PowerHub → GPS：整行装配，$S3R 协议行拦截自用，其余（PCAS 配置等）原样转发
    while (linkSerial.available()) linkFeedChar((char)linkSerial.read());

    // USB 控制台：$S3R 协议 + 裸命令（help/info/...）
    while (Serial.available()) usbFeedChar((char)Serial.read());

    imuTick(now);            // BMI270 采样 + 静止检测/零偏/航向角速率
    fuseTick(now);           // $PFUSE 1Hz + 锁存释放兜底
    imulogTick(now);         // 环形日志周期落盘 + UTC 对齐记录

    gpsPolarityWatch(now);   // 无数据自动换脚
    updateGpsState(now);     // 定位状态机（供屏显）
    statsTick(now);

    if (now - tLastBtn >= BTN_SCAN_MS) { tLastBtn = now; buttonScan(now); }
    screenTick(now);

    delay(2);                // 让出 CPU；115200 下 4KB RX 缓冲有 ~350ms 余量
}

// ═══════════════════════════════════════════════════════════════════════════
// 串口泵与极性
// ═══════════════════════════════════════════════════════════════════════════

// 开机端口角色/极性探测。四脚全接内部下拉后采样：
//   持续翻转的脚 = GPS TX（上电即吐 NMEA）→ 我方 GPS RX，其配对脚为 GPS TX；
//   另一对归链路，其中"恒高无翻转"的脚 = 对侧 PowerHub TX 空闲电平 → 我方链路 RX
//   （判不出——PowerHub 未接/未上电——保持 NVS/默认极性，仅修正到正确的口对）。
// 全程只做输入，无总线冲突。返回 true = 扫到了 GPS 数据流。
static bool bootProbePorts() {
    static const int P[4] = {PORT_OWN_A, PORT_OWN_B, PORT_BASE_A, PORT_BASE_B};
    for (int p : P) pinMode(p, INPUT_PULLDOWN);
    delay(20);
    uint32_t deadline = millis() + PROBE_MAX_MS;      // GPS 与 S3R 同链上电，最多等它开流
    while ((int32_t)(deadline - millis()) > 0) {
        uint32_t edges[4] = {0}, highs[4] = {0}, n = 0;
        int prev[4];
        for (int i = 0; i < 4; i++) prev[i] = digitalRead(P[i]);
        uint32_t until = millis() + 300;
        while (millis() < until) {
            for (int i = 0; i < 4; i++) {
                int v = digitalRead(P[i]);
                if (v != prev[i]) { edges[i]++; prev[i] = v; }
                highs[i] += v;
            }
            n++;
        }
        int s = -1;                                    // 数据流最强的脚
        for (int i = 0; i < 4; i++)
            if (edges[i] > 50 && (s < 0 || edges[i] > edges[s])) s = i;
        if (s < 0) continue;                           // 尚无数据流，下一轮
        int gp = (s < 2) ? 0 : 2;                      // GPS 所在对的首索引
        int lp = gp ^ 2;                               // 链路对的首索引
        gpsRxPin = P[s];
        gpsTxPin = P[gp + ((s - gp) ^ 1)];
        bool aHi = n && highs[lp]     * 100 / n > 90 && edges[lp]     < 5;
        bool bHi = n && highs[lp + 1] * 100 / n > 90 && edges[lp + 1] < 5;
        if      (aHi && !bHi) { linkRxPin = P[lp];     linkTxPin = P[lp + 1]; }
        else if (bHi && !aHi) { linkRxPin = P[lp + 1]; linkTxPin = P[lp]; }
        else if (linkRxPin != P[lp] && linkRxPin != P[lp + 1]) {
            linkRxPin = P[lp];                          // NVS/默认在错误的口对上 → 修正
            linkTxPin = P[lp + 1];
        }
        return true;
    }
    return false;
}

static void gpsUartStart() {
    gpsSerial.end();
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, gpsRxPin, gpsTxPin);
    Serial.printf("[GPS] UART1 %d 8N1  RX=G%d TX=G%d\n", GPS_BAUD, gpsRxPin, gpsTxPin);
}

static void linkUartStart() {
    linkSerial.end();
    linkSerial.begin(LINK_BAUD, SERIAL_8N1, linkRxPin, linkTxPin);
    Serial.printf("[LNK] UART2 %d 8N1  RX=G%d TX=G%d\n", LINK_BAUD, linkRxPin, linkTxPin);
}

// GPS 极性兜底看门狗（开机探测没扫到流时才起作用）：GPS 持续吐 NMEA，接对了必有
// 数据。收不到就每 GPS_SWAP_MS 在当前口对内换一次 RX/TX，各试 2 轮仍无数据 →
// 判无模块（屏显 NOGPS，继续低速轮换以便热插）。
static void gpsPolarityWatch(uint32_t now) {
    if (gpsPinLocked) return;
    if (gpsChars > 20) {               // 有真数据 → 锁定并记住
        gpsPinLocked = true;
        prefs.putUChar("grx", (uint8_t)gpsRxPin);
        Serial.printf("[GPS] pins locked (RX=G%d), saved\n", gpsRxPin);
        return;
    }
    if (now - tGpsSwap >= GPS_SWAP_MS) {
        tGpsSwap = now;
        int t = gpsRxPin; gpsRxPin = gpsTxPin; gpsTxPin = t;
        gpsSwapCount++;
        gpsUartStart();
        if (gpsSwapCount >= 4 && gpsState == GS_DETECTING) gpsState = GS_NO_MODULE;
    }
}

// GPS 旁路解析：字节计数 + TinyGPS++ + 整行装配喂 GSV/TXT 诊断
static void gpsFeedChar(char c, uint32_t now) {
    gpsChars++;
    gpsByteAcc++;
    tLastGpsByte = now;
    gps.encode(c);
    if (c == '\n' || c == '\r') {
        if (nmeaLen) { nmeaLine[nmeaLen] = 0; gnssDiagLine(nmeaLine); nmeaLen = 0; }
    } else if (nmeaLen < sizeof(nmeaLine) - 1) {
        nmeaLine[nmeaLen++] = c;
    } else {
        nmeaLen = 0;                   // 行超长，丢弃
    }
}

// 链路（PowerHub 方向）整行装配：$S3R 开头的行是发给我们的协议（PING/OTA/...），
// 其余整行转发给 GPS——今天这条线上只有 PCAS 一次性配置会经过，透传语义不变。
static void linkFeedChar(char c) {
    if (c == '\n' || c == '\r') {
        if (!linkLen) return;
        linkLine[linkLen] = 0;
        linkLen = 0;
        if (strncmp(linkLine, "$S3R", 4) == 0) s3rHandleLine(linkLine, &linkSerial);
        else { gpsSerial.print(linkLine); gpsSerial.print("\r\n"); }
        return;
    }
    if (linkLen < sizeof(linkLine) - 1) linkLine[linkLen++] = c;
    else {                             // 超长（非行式数据？）原样冲给 GPS，保持透传兜底
        linkLine[linkLen] = 0;
        gpsSerial.print(linkLine);
        linkLen = 0;
        linkLine[linkLen++] = c;
    }
}

// USB 控制台：支持与链路相同的 $S3R 协议（台面直测 OTA），及少量裸命令
static void usbFeedChar(char c) {
    if (c == '\n' || c == '\r') {
        if (!usbLen) return;
        usbLine[usbLen] = 0;
        usbLen = 0;
        if (strncmp(usbLine, "$S3R", 4) == 0) s3rHandleLine(usbLine, &Serial);
        else handleBareCmd(usbLine);
        return;
    }
    if (usbLen < sizeof(usbLine) - 1) usbLine[usbLen++] = c;
    else usbLen = 0;
}

static void handleBareCmd(const char* line) {
    if (!strcmp(line, "help")) {
        Serial.println("commands: help | info | ping | imu | fuse [on|off] | imulog | imuclear"
                       " | screen | linkswap | pinscan | reboot");
        Serial.println("protocol: $S3R,PING / $S3R,INFO / $S3R,FUSE,ON|OFF / $S3R,IMUDUMP /"
                       " $S3R,IMULOG,CLEAR / $S3R,OTA,BEGIN,... (see ota_flash.py)");
    } else if (!strcmp(line, "imu")) {
        imuPrintStatus();
    } else if (!strcmp(line, "fuse")) {
        fusePrintStatus();
    } else if (!strcmp(line, "fuse on")) {
        fuseSetEnabled(true);
    } else if (!strcmp(line, "fuse off")) {
        fuseSetEnabled(false);
    } else if (!strcmp(line, "imulog")) {
        uint32_t total; uint16_t segs, sumSegs;
        imulogStats(&total, &segs, &sumSegs);
        Serial.printf("[IL] 全速率 %u/%d 段 + 摘要 %u/%d 段 = %luKB\n",
                      segs, ILOG_SEG_MAX, sumSegs, ILOG_SUM_SEG_MAX,
                      (unsigned long)(total / 1024));
    } else if (!strcmp(line, "imuclear")) {
        imulogClear();
    } else if (!strcmp(line, "info")) {
        s3rHandleLine("$S3R,INFO", &Serial);
    } else if (!strcmp(line, "ping")) {
        s3rHandleLine("$S3R,PING", &Serial);
    } else if (!strcmp(line, "screen")) {
        displaySetOn(!displayOn);
    } else if (!strcmp(line, "linkswap")) {
        // 链路极性手动翻转（PowerHub 未上电时探测不出、默认又不对的兜底），即时生效并存 NVS
        int t = linkRxPin; linkRxPin = linkTxPin; linkTxPin = t;
        prefs.putUChar("lrx", (uint8_t)linkRxPin);
        linkUartStart();
    } else if (!strcmp(line, "pinscan")) {
        // 诊断：扫底部排针 6 脚 + 自带 Grove 口 2 脚，找带 UART 活动的脚
        // （GPS TX 空闲恒高 + 数据翻转）。接线/底座映射存疑时用；扫完恢复两个串口。
        static const int pins[] = {1, 2, 5, 6, 7, 8, 38, 39};
        gpsSerial.end();
        linkSerial.end();
        for (int p : pins) {
            pinMode(p, INPUT_PULLDOWN);
            delay(5);
            uint32_t edges = 0, highs = 0, samples = 0;
            int prev = digitalRead(p);
            uint32_t until = millis() + 300;
            while (millis() < until) {
                int v = digitalRead(p);
                if (v != prev) { edges++; prev = v; }
                highs += v;
                samples++;
            }
            Serial.printf("[SCAN] G%-2d  edges=%-6lu high=%lu%%\n",
                          p, (unsigned long)edges,
                          (unsigned long)(highs * 100 / samples));
        }
        Serial.println("[SCAN] done (edges>0 且 high 高 = UART TX；全 0 = 无驱动/未上电)");
        gpsUartStart();
        linkUartStart();
    } else if (!strcmp(line, "reboot")) {
        Serial.println("rebooting...");
        delay(200);
        ESP.restart();
    } else {
        Serial.printf("unknown cmd: %s (try 'help')\n", line);
    }
}

static void statsTick(uint32_t now) {
    if (now - tLastStats < STATS_MS) return;
    tLastStats = now;
    gpsBps = gpsByteAcc * 1000UL / STATS_MS;
    gpsByteAcc = 0;
}

// 按键（整块屏）：短按 = 亮屏/续 1 分钟；亮屏中长按 1s = 立即息屏
static void buttonScan(uint32_t now) {
    bool down = (digitalRead(BTN_PIN) == LOW);
    if (down && !btnPrev) { tBtnPress = now; btnOffFired = false; }
    if (down && btnPrev && displayOn && !btnOffFired
            && now - tBtnPress >= BTN_OFF_HOLD_MS) {
        btnOffFired = true;
        displaySetOn(false);
    }
    if (!down && btnPrev && !btnOffFired && now - tBtnPress < BTN_OFF_HOLD_MS) {
        displaySetOn(true);            // 短按：亮屏并重置息屏计时
    }
    btnPrev = down;
}
