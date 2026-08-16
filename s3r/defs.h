// defs.h — S3R GNSS 协处理器：引脚/常量/枚举/结构体 + 全部函数前置声明
//
// 多 .ino 单编译单元模型（同主固件 firmware/defs.h）：Arduino 把 s3r/ 下所有 .ino
// 按「主文件在前、其余字母序」拼成一份编译，本头对所有 .ino 可见。
//
// v0.1.0 = 路线第①步「透传」：
//   GPS(ABC Base 蓝口) ↔ PowerHub(自带 Grove 口) 双向透明转发，行为与"GPS 直连
//   PowerHub"严格一致（PowerHub 固件零改动）；旁路解析 NMEA 供屏显；
//   UART-OTA 接收器 + A/B 槽启动确认（以后更新固件全走 PowerHub 透传桥，不拆机）。
// 第②步加 IMU 旁路记录，第③步 ESKF 融合上线（见 s3r/README.md 路线图）。
#pragma once
#include <Arduino.h>
#include <M5GFX.h>      // 须在此（而非 screen.ino）：Arduino 自动原型会被提升到主文件
                        // include 之后、各 .ino 正文之前，LovyanGFX 类型必须在那之前可见
#include <M5Unified.h>  // 只用其 BMI270_Class + In_I2C（不调 M5.begin，屏仍归上面的 M5GFX）
#include <utility/imu/BMI270_Class.hpp>   // 内部驱动头，M5Unified.h 不公开导出
#include <LittleFS.h>   // IMU/GNSS 环形日志（spiffs 分区 1.5MB）
#include "nmea_rw.h"    // NMEA 解析/改写/重建纯 C 核心（主机可单测）

#define S3R_FW_VER "0.2.0"

// ═══════════════════════════════════════════════════════════════════════════
// 引脚
// ═══════════════════════════════════════════════════════════════════════════
// 两个可用 Grove 口：S3R 自带口(G1/G2) 和 ABC Base 蓝口 PORT.C(G5/G6，对照 ATOMS3
// 底部排针：左列 3V3·G5·G6·G7·G8 / 右列 G39·G38·5V·GND，底座 A=G38/G39 B=G7/G8)。
// 一口接 GPS(ATGM336H)、一口接 PowerHub PORT.C——**谁接哪个口开机自动识别**：
// GPS 上电即持续吐 NMEA，扫到持续翻转的脚 = GPS TX = 我方 GPS RX，其配对脚为 GPS TX；
// 另一对归链路，链路 RX 靠"PowerHub TX 空闲恒高"判别（PowerHub 侧固定 G1=TX/G2=RX）。
// 识别结果存 NVS；两口插反、线色对应不明都能工作（2026-08-07 实测 GPS 就插在自带口）。
#define PORT_OWN_A       1   // 自带 Grove 口
#define PORT_OWN_B       2
#define PORT_BASE_A      5   // ABC Base 蓝口
#define PORT_BASE_B      6
// 探测失败（GPS 未上电等）时的默认角色：GPS=自带口 / 链路=底座蓝口（当前实际接法）
#define GPS_RX_DEFAULT   PORT_OWN_A
#define GPS_TX_DEFAULT   PORT_OWN_B
#define LINK_RX_DEFAULT  PORT_BASE_A
#define LINK_TX_DEFAULT  PORT_BASE_B
#define BTN_PIN         41   // AtomS3R 整块屏幕即按键，低有效（板载上拉）

#define LINK_BAUD   115200
#define GPS_BAUD    115200   // ATGM336H 已被一次性配置 115200/5Hz/四系统（存于模块自身 flash）

// ═══════════════════════════════════════════════════════════════════════════
// 调参常量
// ═══════════════════════════════════════════════════════════════════════════
// CPU 下限 80MHz：本板控制台走 USB-Serial/JTAG，48MHz USB 时钟来自 PLL，
// 选 <80 的频率会切裸晶振并关 PLL → USB 断链（同 PowerHub 固件的坑）。
// 透传+解析+屏显在 80MHz 下负载极低，这也是省电最优点（融合上线后再评估 160）。
static const uint8_t  CPU_MHZ            = 80;
static const uint32_t PROBE_MAX_MS       = 8000;     // 开机端口探测窗口（等 GPS 上电开流）
static const uint32_t GPS_SWAP_MS        = 4000;     // 无数据多久换一次 RX/TX 极性
static const uint32_t GPS_STALE_MS       = 5000;     // 定位数据超龄 → 视为丢定位
static const uint32_t GNSS_NOSAT_WARN_MS = 180000UL; // 持续 0 可见星超此时长 → 屏显 ANT?
                                                     // （屋内本就无 GPS 信号，台面上出现属正常）
static const uint32_t DISPLAY_ON_MS      = 60000;    // 亮屏窗口：开机/按键后 1 分钟
static const uint32_t LCD_DRAW_MS        = 500;      // 亮屏时重绘周期
static const uint8_t  LCD_BRIGHTNESS     = 90;       // 亮度(0-255)，小屏低亮度足够
static const uint32_t BTN_SCAN_MS        = 30;       // 按键轮询周期
static const uint32_t BTN_OFF_HOLD_MS    = 1000;     // 亮屏时长按此时长 → 立即息屏
static const uint32_t STATS_MS           = 1000;     // 字节速率统计周期

// ── UART-OTA ────────────────────────────────────────────────────────────────
static const uint32_t OTA_LINE_TMO_MS    = 3000;     // 等一行协议头的超时
static const uint32_t OTA_DATA_TMO_MS    = 3000;     // 等一个数据块体的超时
static const uint32_t OTA_IDLE_ABORT_MS  = 15000;    // 整体无活动 → 放弃本次 OTA
static const uint16_t OTA_CHUNK_MAX      = 1024;     // 单块上限（与 ota_flash.py 一致）
static const uint8_t  OTA_MAX_ATTEMPTS   = 3;        // 待确认固件启动尝试上限，超过 → 回滚

// ── IMU（BMI270，姿态无关算法：不假设任何安装朝向，口袋/包里怎么放都成立）──────
static const float    IMU_ACC_RES  = 8.0f  / 32768.0f * 9.80665f;  // raw→m/s²（±8g 默认量程）
static const float    IMU_GYR_RES  = 2000.0f / 32768.0f;           // raw→dps（±2000dps 默认量程）
static const int      IMU_BLOCK_N        = 25;       // 统计块：25 样本 = 0.25s @100Hz
static const float    IMU_GRAV_ALPHA     = 0.005f;   // 重力向量 LPF 系数（τ≈2s @100Hz）
// 静止检测（进入慢、退出快 + 双阈滞回）。量纲：accStd = 0.25s 块内 |a| 标准差 m/s²；
// gyro = 块内 |ω-bias| 平均 dps。桌面静置 accStd≈0.02-0.05，手持站立 0.1-0.3，步行 1-4。
static const float    IMU_STAT_ACC_ENTER = 0.35f;    // 块 accStd 低于此才算"安静块"
static const float    IMU_STAT_GYR_ENTER = 5.0f;     // 且块陀螺低于此 dps
static const int      IMU_STAT_ENTER_BLK = 10;       // 连续 10 安静块(2.5s) → 进入静止
static const float    IMU_STAT_ACC_EXIT  = 0.60f;    // 单块超此 → 立即退出静止
static const float    IMU_STAT_GYR_EXIT  = 10.0f;    // 或单块陀螺超此 dps
// 陀螺零偏：静止满 2s 起，用 2s 窗均值 EMA 进零偏（首次直接置入）
static const int      IMU_BIAS_BLKS      = 8;        // 零偏窗：8 块 = 2s
static const float    IMU_BIAS_EMA       = 0.3f;     // 后续更新的 EMA 权重
static const int      IMU_LOG_DECIM      = 4;        // 落盘抽取：4 样本平均 → 25Hz

// ── 融合 v1（fuse.ino：静止锁存 + 短断档 DR 桥接 + GNSS 逃生门）────────────────
static const uint32_t FUSE_LATCH_MIN_STAT_MS = 3000;   // 静止满 3s 才锁存
static const int      FUSE_HIST_N        = 15;       // 定位历史环（5Hz×3s，取中位数做锁点）
static const int      FUSE_HIST_MIN      = 5;        // 至少 5 个新鲜历史点才允许锁存
static const uint32_t FUSE_HIST_FRESH_MS = 4000;     // "新鲜"= 4s 内
static const float    FUSE_HIST_HDOP_MAX = 6.0f;     // 历史点收录的 HDOP 上限
static const float    FUSE_ESC_DIST_M    = 35.0f;    // 逃生门：原始定位离锁点超此距离
static const uint32_t FUSE_ESC_MS        = 8000;     // 且持续 8s（HDOP<4）→ 强制解锁
static const float    FUSE_ESC_HDOP      = 4.0f;
static const uint32_t FUSE_DR_START_MS   = 3000;     // 断档前最后好定位 3s 内才允许起 DR
static const uint32_t FUSE_DR_MAX_MS     = 15000;    // DR 最长桥 15s
static const float    FUSE_DR_MAX_DIST_M = 60.0f;    // 或累计 60m，超过即停（诚实丢定位）
static const float    FUSE_DR_MIN_SPD    = 0.5f;     // m/s：低于此不起 DR（步速下限）
static const float    FUSE_DR_SPD_DECAY  = 0.03f;    // DR 速度衰减 3%/s（不确定性递增的保守化）
static const float    FUSE_DR_HDOP_OK    = 4.0f;     // "好定位"参考点的 HDOP 上限
static const uint32_t FUSE_PFUSE_MS      = 1000;     // $PFUSE 诊断句周期

// ── IMU/GNSS 双层环形日志（imulog.ino，LittleFS，P2 重分区后 FS≈4.875MB）─────────
// 全速率层 /il/：25Hz IMU + 5Hz GNSS + 事件——疑难段深挖用，环形只保最近 ~55min。
// 摘要层 /ils/：1Hz GNSS + 1Hz 设备自算特征块——全天回放调参的主粮（~19h 环形）。
// 特征块必须是**设备自算**的（0.25s 块统计的秒级聚合）：回放侧拿 25Hz 日志复算
// 的窗口尺度对不上固件 100Hz 块，阈值语义会漂——这是 P2 记录升级的核心动机之一。
static const uint32_t ILOG_SEG_BYTES     = 96UL * 1024;  // 单段上限（两层通用）
static const int      ILOG_SEG_MAX       = 18;           // 全速率层段数（~1.7MB ≈55min）
static const int      ILOG_SUM_SEG_MAX   = 26;           // 摘要层段数（~2.4MB ≈19h）
static const uint32_t ILOG_FLUSH_MS      = 2000;         // RAM 缓冲落盘周期（掉电最多丢 2s）
static const uint32_t ILOG_TIME_MS       = 60000;        // 'T' 时间对齐记录周期（millis↔UTC）
static const uint32_t ILOG_SUM_GNSS_MS   = 1000;         // 摘要层 GNSS 抽取周期
// 记录类型（[type u8][定长负载]，小端）：
enum : uint8_t {
    ILOG_IMU  = 0x01,   // +16B: tms u32, ax ay az gx gy gz i16(raw)
    ILOG_GNSS = 0x02,   // +21B: tms, lat i32 1e7, lon i32 1e7, alt i16 m, spd u16 0.01m/s,
                        //       crs u16 0.1deg, hdop u8 x10, sats u8, flags u8(b0=valid)
    ILOG_FUSE = 0x03,   // +13B: tms, lat i32, lon i32, mode u8(1=LATCH 2=DR)
    ILOG_EVT  = 0x04,   // + 9B: tms, code u8, val f32
    ILOG_BIAS = 0x05,   // +16B: tms, bx by bz f32 (dps)
    ILOG_TIME = 0x06,   // + 8B: tms, epoch u32 (UTC, 由 RMC 日期时间换算)
    ILOG_FEAT = 0x07,   // +13B: tms, accStdMean u16 mm/s², accStdMax u16 mm/s²,
                        //       gyroMean u16 0.01dps, headRate i16 0.01dps, flags u8(b0=stat)
    ILOG_SGNS = 0x08,   // +21B: 摘要层 1Hz GNSS，负载布局与 ILOG_GNSS 完全相同
};
// 环形段管理（imulog.ino 两层实例共用）。定义必须在 defs.h：imulog.ino 的函数签名
// 用到 IlRing&，Arduino 自动原型会被提升到 .ino 正文之前，类型须先于原型可见（老坑）。
struct IlRing {
    const char* dir;        // "/il" / "/ils"
    const char* seqKey;     // NVS 段序号键
    int         segMax;
    File        f;
    uint32_t    curSize;
    uint8_t     buf[512];
    uint16_t    bufLen;
};
enum : uint8_t {        // ILOG_EVT 事件码
    EV_LATCH_ON = 1, EV_LATCH_OFF = 2, EV_LATCH_ESC = 3,
    EV_DR_ON = 4, EV_DR_OFF = 5 /*val=DR终点与回归定位的误差m*/, EV_DR_ABORT = 6,
    EV_BIAS_SET = 7, EV_IMU_FAIL = 8,
};

// ═══════════════════════════════════════════════════════════════════════════
// 枚举
// ═══════════════════════════════════════════════════════════════════════════
enum GpsState : uint8_t {
    GS_DETECTING,   // 开机探测中（含极性自动换脚）
    GS_NO_MODULE,   // 换遍两种极性仍无数据
    GS_SEARCHING,   // 有 NMEA 流，无有效定位 → 黄
    GS_FIX_GOOD     // 有有效定位 → 绿
};

// ═══════════════════════════════════════════════════════════════════════════
// 前置声明
// ═══════════════════════════════════════════════════════════════════════════

// ── s3r.ino ──
static void statsTick(uint32_t now);
static void buttonScan(uint32_t now);
static bool bootProbePorts();
static void gpsUartStart();
static void linkUartStart();
static void gpsPolarityWatch(uint32_t now);
static void gpsFeedChar(char c, uint32_t now);
static void linkFeedChar(char c);
static void usbFeedChar(char c);
static void handleBareCmd(const char* line);

// ── parse.ino ──
static void     gnssDiagLine(const char* s);
static uint16_t gnssTotalInView();
static uint8_t  gnssMaxCN0();
static void     updateGpsState(uint32_t now);

// ── ota.ino ──
static const char* otaRunningPartition();
static void otaBootGuard();
static void otaConfirmPending(Stream* io);
static void s3rHandleLine(const char* line, Stream* io);
static void otaReceive(Stream* io, uint32_t size, const char* md5hex);
static uint32_t crc32sw(const uint8_t* d, size_t n);           // imulog dump 复用
static bool otaReadLine(Stream* io, char* buf, size_t cap, uint32_t deadline);  // 同上

// ── screen.ino ──
static void scrRender(LovyanGFX* g);
static void screenInit();
static void screenBootMsg(const char* msg);
static void displaySetOn(bool on);
static void screenTick(uint32_t now);
static void screenOtaProgress(uint32_t got, uint32_t total);
static void screenRedraw();

// ── imu.ino ──
static bool  imuInit();
static void  imuTick(uint32_t now);
static bool  imuOk();
static bool  imuIsStationary();
static uint32_t imuStationarySince();   // 进入静止的 millis（非静止时无意义）
static bool  imuBiasKnown();
static float imuHeadingRateDps();       // 罗盘航向角速率，顺时针为正（重力轴投影）
static float imuAccStd();               // 最近统计块 |a| 标准差 m/s²
static float imuGyroMag();              // 最近统计块 |ω-bias| 均值 dps
static void  imuPrintStatus();          // console `imu`

// ── fuse.ino ──
static void  fuseInit();
static void  fusePumpChar(char c);      // GPS→link 整行泵（装配→按需改写→转发）
static void  fuseTick(uint32_t now);    // $PFUSE + 与句流无关的状态维护
static void  fuseSetEnabled(bool on);   // NVS 持久
static bool  fuseEnabledGet();
static bool  fuseIsLatched();
static bool  fuseIsDr();
static void  fusePrintStatus();         // console `fuse`

// ── imulog.ino ──
static void  imulogBegin();
static void  imulogTick(uint32_t now);
static void  imulogImuRaw(uint32_t tms, const int16_t a[3], const int16_t g[3]);
static void  imulogGnss(uint32_t tms, double lat, double lon, float altM,
                        float spdMps, float crsDeg, float hdop, uint8_t sats, bool valid);
static void  imulogFuse(uint32_t tms, double lat, double lon, uint8_t mode);
static void  imulogEvent(uint8_t code, float val);
static void  imulogBias(const float b[3]);
static void  imulogTimeMark(uint32_t epoch);
static void  imulogStats(uint32_t* totalBytes, uint16_t* segs);
static void  imulogClear();
static void  imulogDump(Stream* io);    // $S3R,IMUDUMP 分块 ACK 传输（阻塞）
