// screen.ino — AtomS3R 自带 0.85" 128×128 屏（GC9107，M5GFX 自动检测）
// 版式承袭配置B的 Unit LCD 状态屏（去时间行，按方屏重排）：
//   ┌──────────────┐
//   │ FIX      12  │  定位状态 + 星数（Font4，状态着色）
//   ├──────────────┤
//   │ H1.2    C43  │  HDOP + 最强 CN0（Font4）
//   ├──────────────┤
//   │ SPD 12.3km/h │  定位中=速度；搜星中=各星座可见星（Font2）
//   │ A35m   3h05m │  海拔(无定位=NMEA流速率;OTA待确认=红PEND) + 运行时长（Font2）
//   └──────────────┘
// 默认开机亮 1 分钟后息屏省电；短按屏幕唤醒/续时，亮屏中长按 1s 息屏（见 s3r.ino）。
#include "defs.h"   // M5GFX.h 已在 defs.h 引入（自动原型提升需要，见彼处注释）

static M5GFX    display;
static M5Canvas canvas(&display);
static bool     canvasOK = false;

namespace sfonts = lgfx::fonts;
using lgfx::textdatum_t;

static void screenInit() {
    scrOK = display.init();
    if (!scrOK) { Serial.println("[SCR] init FAIL"); return; }
    display.setRotation(0);
    display.setBrightness(LCD_BRIGHTNESS);
    display.fillScreen(TFT_BLACK);
    canvas.setColorDepth(16);
    canvasOK = (canvas.createSprite(128, 128) != nullptr);   // 32KB，整帧推送无闪烁
    Serial.printf("[SCR] init OK (%s)\n", canvasOK ? "canvas" : "direct");
}

// 定位状态 → 显示词 + 颜色（词义与 PowerHub 的 GPS LED 一致）
static const char* scrGpsWord(int* col) {
    switch (gpsState) {
        case GS_FIX_GOOD:  *col = TFT_GREEN;    return "FIX";
        case GS_SEARCHING: *col = TFT_YELLOW;   return "SRCH";
        case GS_NO_MODULE: *col = TFT_RED;      return "NOGPS";
        default:           *col = TFT_CYAN;     return "DET";
    }
}

static void scrRender(LovyanGFX* g) {
    char line[40];
    int  col;
    g->fillScreen(TFT_BLACK);
    g->setTextWrap(false);

    const int D1 = 44, D2 = 87;
    g->drawFastHLine(2, D1, 124, TFT_DARKGREY);
    g->drawFastHLine(2, D2, 124, TFT_DARKGREY);
    // Font4 字格含下沿留白，大字带垂直中心 +4px 视觉居中（同配置B）
    const int CY_GPS = 24, CY_SIG = 68, CY_R3 = 98, CY_R4 = 118;

    // ── 行1：定位状态 + 星数 ────────────────────────────────────────────────
    // 天线告警只认 SHORT（真短路，模块限流保护中）。OPEN 不是故障：本 GPS 单元用
    // 内置无源天线，模块的天线检测电路测的是有源天线馈电电流，无馈电 → 恒报
    // ANTENNA OPEN（数据手册 §2.8，2026-08-08 与主人确认），属正常态。
    g->setFont(&sfonts::Font4);
    if (gnssAnt == 3) {
        g->setTextColor(TFT_RED, TFT_BLACK);
        g->setTextDatum(textdatum_t::middle_center);
        g->drawString("ANT SHRT", 64, CY_GPS);
    } else if (gpsState == GS_SEARCHING && tSatsZeroSince
               && millis() - tSatsZeroSince >= GNSS_NOSAT_WARN_MS) {
        g->setTextColor(TFT_RED, TFT_BLACK);
        g->setTextDatum(textdatum_t::middle_center);
        g->drawString("ANT?", 64, CY_GPS);       // 屋内无 GPS 信号时属正常现象
    } else {
        const char* gw = scrGpsWord(&col);
        g->setTextColor(col, TFT_BLACK);
        if (gpsState == GS_FIX_GOOD || gpsState == GS_SEARCHING) {
            g->setTextDatum(textdatum_t::middle_left);
            g->drawString(gw, 4, CY_GPS);
            // FIX 显参与解算星数(GGA)；SRCH 显可见星总数(GSV)——搜星进度更直观
            unsigned n = (gpsState == GS_FIX_GOOD && gps.satellites.isValid())
                         ? (unsigned)gps.satellites.value() : (unsigned)gnssTotalInView();
            snprintf(line, sizeof(line), "%u", n);
            g->setTextDatum(textdatum_t::middle_right);
            g->drawString(line, 124, CY_GPS);
        } else {
            g->setTextDatum(textdatum_t::middle_center);
            g->drawString(gw, 64, CY_GPS);
        }
    }

    // ── 行2：HDOP + 最强 CN0 ────────────────────────────────────────────────
    g->setFont(&sfonts::Font4);
    g->setTextColor(TFT_WHITE, TFT_BLACK);
    if (gps.hdop.isValid() && gps.hdop.age() < GPS_STALE_MS)
        snprintf(line, sizeof(line), "H%.1f", gps.hdop.hdop());
    else snprintf(line, sizeof(line), "H-.-");
    g->setTextDatum(textdatum_t::middle_left);
    g->drawString(line, 4, CY_SIG);
    uint8_t c = gnssMaxCN0();
    if (c) snprintf(line, sizeof(line), "C%u", c);
    else   snprintf(line, sizeof(line), "C--");
    g->setTextDatum(textdatum_t::middle_right);
    g->drawString(line, 124, CY_SIG);

    // ── 行3：锁存/DR 状态 > 速度 > 各星座可见星（G/R/B/E/Q）──
    g->setFont(&sfonts::Font2);
    g->setTextColor(TFT_WHITE, TFT_BLACK);
    g->setTextDatum(textdatum_t::middle_center);
    if (fuseIsLatched()) {
        g->setTextColor(TFT_CYAN, TFT_BLACK);      // 静止锁存中（IMU 判静止，坐标已锁）
        snprintf(line, sizeof(line), "LOCK");
    } else if (fuseIsDr()) {
        g->setTextColor(TFT_ORANGE, TFT_BLACK);    // 断档 DR 桥接中
        snprintf(line, sizeof(line), "DR bridge");
    } else if (gpsState == GS_FIX_GOOD && gps.speed.isValid()) {
        snprintf(line, sizeof(line), "SPD %.1f km/h", gps.speed.kmph());
    } else {
        snprintf(line, sizeof(line), "G%u R%u B%u E%u Q%u",
                 gnssInView[0], gnssInView[1], gnssInView[2],
                 gnssInView[3], gnssInView[4]);
    }
    g->drawString(line, 64, CY_R3);

    // ── 行4：左=OTA待确认(红PEND) / 定位中=海拔 / 其它=NMEA流速率；右=运行时长 ──
    // （版本号不占屏幕，看 info/PONG；小屏寸土寸金——主人 2026-08-08 要求）
    g->setTextDatum(textdatum_t::middle_left);
    if (otaPending) {
        g->setTextColor(TFT_RED, TFT_BLACK);
        g->drawString("PEND", 4, CY_R4);
    } else {
        g->setTextColor(TFT_DARKGREY, TFT_BLACK);
        if (gpsState == GS_FIX_GOOD && gps.altitude.isValid())
            snprintf(line, sizeof(line), "A%.0fm", gps.altitude.meters());
        else if (gpsBps >= 1000)
            snprintf(line, sizeof(line), "%.1fk/s", gpsBps / 1000.0f);
        else
            snprintf(line, sizeof(line), "%luB/s", (unsigned long)gpsBps);
        g->drawString(line, 4, CY_R4);
    }
    g->setTextColor(TFT_DARKGREY, TFT_BLACK);
    uint32_t m = millis() / 60000UL;
    snprintf(line, sizeof(line), "%luh%02lum",
             (unsigned long)(m / 60), (unsigned long)(m % 60));
    g->setTextDatum(textdatum_t::middle_right);
    g->drawString(line, 124, CY_R4);
}

static void screenRedraw() {
    if (!scrOK || !displayOn) return;
    if (canvasOK) { scrRender(&canvas); canvas.pushSprite(0, 0); }
    else          { scrRender(&display); }
    tLastDraw = millis();
}

// 引导/提示屏（状态数据未齐时）
static void screenBootMsg(const char* msg) {
    if (!scrOK) return;
    LovyanGFX* g = canvasOK ? (LovyanGFX*)&canvas : (LovyanGFX*)&display;
    g->fillScreen(TFT_BLACK);
    g->setFont(&sfonts::Font2);
    g->setTextDatum(textdatum_t::middle_center);
    g->setTextColor(TFT_CYAN, TFT_BLACK);
    g->drawString("S3R", 64, 52);
    g->setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g->drawString(msg, 64, 76);
    if (canvasOK) canvas.pushSprite(0, 0);
}

// OTA 进度屏（OTA 期间强制亮屏显示）
static void screenOtaProgress(uint32_t got, uint32_t total) {
    if (!scrOK || !total) return;
    if (!displayOn) displaySetOn(true);
    LovyanGFX* g = canvasOK ? (LovyanGFX*)&canvas : (LovyanGFX*)&display;
    int pct = (int)((uint64_t)got * 100 / total);
    char line[16];
    g->fillScreen(TFT_BLACK);
    g->setFont(&sfonts::Font4);
    g->setTextDatum(textdatum_t::middle_center);
    g->setTextColor(TFT_CYAN, TFT_BLACK);
    g->drawString("OTA", 64, 30);
    snprintf(line, sizeof(line), "%d%%", pct);
    g->setTextColor(TFT_WHITE, TFT_BLACK);
    g->drawString(line, 64, 64);
    g->drawRect(14, 92, 100, 12, TFT_DARKGREY);
    g->fillRect(14, 92, pct, 12, TFT_GREEN);
    if (canvasOK) canvas.pushSprite(0, 0);
}

// 开/关屏。开 → 设亮屏窗口 + 立即重绘；关 → 面板睡眠（背光同灭）省电
static void displaySetOn(bool on) {
    if (!scrOK) { displayOn = on; return; }
    if (on) {
        displayOn   = true;
        tDisplayOff = millis() + DISPLAY_ON_MS;
        display.wakeup();
        display.setBrightness(LCD_BRIGHTNESS);
        screenRedraw();
    } else {
        displayOn = false;
        display.sleep();
    }
}

// 息屏超时 + 亮屏时定期重绘
static void screenTick(uint32_t now) {
    if (!displayOn) return;
    if ((int32_t)(now - tDisplayOff) >= 0)       displaySetOn(false);
    else if (now - tLastDraw >= LCD_DRAW_MS)     screenRedraw();
}
