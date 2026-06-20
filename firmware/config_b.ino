// config_b.ino — 配置B 专属逻辑（仅当 GNSS_TIMESHARE==1 编译，否则整文件为空）。
//   配置B = SIM7080G 二合一(Unit CatM GNSS)内置 GNSS 与 LTE 分时共用 PORT.A，
//           腾出 PORT.C 给 Unit LCD 1.14"。位置靠 CGNSINF 轮询；发包时 GNSS↔LTE 切换。
// 这里实现：位置访问器(读 liveFix)、CGNSINF 轮询、NMEA 采样诊断、GPS 状态机、分时发包、
//   LCD 状态屏、catmWaitReg、以及 setup()/loop() 的配置钩子。
#include "defs.h"
#if GNSS_TIMESHARE

// ── 统一位置访问器：读 CGNSINF 轮询填好的 liveFix ────────────────────────────
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

// ── 配置B flush 调度器状态（提前到此声明，供 LCD 渲染与调度器共用）─────────────
static uint32_t flStillSince = 0;   // 进入静止的 millis（0 = 当前非静止）
static uint32_t flNoFixSince = 0;   // 丢定位的 millis（0 = 当前有定位）
static uint32_t flLastUpload = 0;   // 上次成功切 LTE 上传时刻 → LCD "TX" 显示
static bool     flSchedHold  = false; // 台面实测时挂起自动 flush 调度（只用手动 flflush 计时）
static uint32_t tSatsZeroSince = 0;   // 进入"连续 0 可见星"的 millis（0=非零星/未开始）；LCD 据此判 ANT? 告警

// 切回 LTE 后等模组重新附着到网络再判网。
// 二合一是 GNSS/LTE 分时共用射频：GNSS 跟踪期(CGNSPWR=1)LTE 被挂起，CGNSPWR=0 把
// 射频交还 LTE 后，模组要数秒才能重新搜网+附着。切完射频**立刻**查 CEREG/激活 PDP
// 必然看到"没注册"而误判失败——这是配置B 红灯锁死的根因。轮询 CEREG 直到注册
// (stat=1 home / 5 roaming)或超时；已注册时首查即过、零等待。
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
        liveFix.sats  = (uint8_t)fld(14).toInt();   // 搜星中也记录可见星数（LCD 现场判断 + 天线代理）
    }
    // 维护"连续 0 可见星"计时：有星即清零；首次 0 星记起点 → LCD 据此判 ANT? 天线告警。
    if (liveFix.sats > 0) tSatsZeroSince = 0;
    else if (tSatsZeroSince == 0) tSatsZeroSince = millis();
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

// ═══════════════════════════════════════════════════════════════════════════
// GPS state machine update（无 PORT.C 模块；gpsState 由 liveFix 新鲜度决定）
// ═══════════════════════════════════════════════════════════════════════════
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

// 注：配置B 不再有"实时分时发包"——改为 beacon 记录到 Flash 段日志(configBeaconAction)
// + 静止/无定位时整批切 LTE 上传(flashFlushViaLte)。故旧的 sendGpsData(CGNSPWR 切换单
// 点发) 已删除；上层接口 sendGpsData 仅配置A 实现（共用 loop 经 configBeaconAction 调用）。

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
        case GS_INIT_FAIL: *col = RED;      return "ERROR";
        default:           *col = DARKGREY; return "OFF";
    }
}
// 当前网络状态 → 显示词 + 颜色（与 CatM LED 同义）。标签缩成单字母 N，故状态词用全拼。
static const char* netWord(int* col) {
    switch (catmState) {
        case CM_READY:   *col = CYAN;     return "READY";
        case CM_SENDING: *col = WHITE;    return "SEND";
        case CM_OK:      *col = GREEN;    return "OK";
        case CM_ERR:     *col = RED;      return "ERROR";
        case CM_INIT:    *col = YELLOW;   return "INIT";
        default:         *col = DARKGREY; return "OFF";
    }
}

// 把状态画进 g（离屏画布或直接是屏）。135×240 竖屏，分隔线把屏分成几个"带"，
// 每个带里内容带内居中。上 4 行大字(Font4)各占 45px；底部 UP/TX/BUF 一块(无内部分隔线)，
// Font2、标签左 + 数值右、全白。LovyanGFX 无 Font3。
// 注：Font4 字格含下沿留白，大写/数字只占上半，故大字垂直中心 +4px 才视觉居中。
static void lcdRender(LovyanGFX* g) {
    char line[40];
    int  col;
    g->fillScreen(BLACK);
    g->setTextWrap(false);

    // 分隔线 y：时间|电量|定位|网络| 之后底部 UP/TX/BUF 一块（UP 下方不再有分隔线）。
    const int D1 = 45, D2 = 90, D3 = 135, D4 = 180;
    g->drawFastHLine(4, D1, 127, DARKGREY); g->drawFastHLine(4, D2, 127, DARKGREY);
    g->drawFastHLine(4, D3, 127, DARKGREY); g->drawFastHLine(4, D4, 127, DARKGREY);
    // 大字带视觉中心（带几何中心 +4 补字格下沿留白）；底部小字三行
    const int CY_TIME = 26, CY_BAT = 71, CY_GPS = 116, CY_NET = 161;
    const int CY_UP = 192, CY_TX = 212, CY_BUF = 231;

    // ── 时间 HH:MM（Font4，带内居中）。RTC 存 UTC epoch；+9h 出 JST 墙钟，不动全局 TZ。──
    time_t rt = time(nullptr);
    g->setFont(&fonts::Font4);
    g->setTextDatum(textdatum_t::middle_center);
    if (rt > 1735689600L) {
        time_t jst = rt + 9 * 3600;
        struct tm t; gmtime_r(&jst, &t);
        snprintf(line, sizeof(line), "%02d:%02d", t.tm_hour, t.tm_min);
        g->setTextColor(WHITE, BLACK);
    } else {
        snprintf(line, sizeof(line), "--:--");
        g->setTextColor(DARKGREY, BLACK);
    }
    g->drawString(line, 67, CY_TIME);

    // ── 电量：100% 左对齐(按电量着色) + 电压右对齐(白)（Font4），与其余各行对齐 ──
    {
        int bcol = !batValid ? CYAN
                 : batPct < 10 ? RED
                 : batPct < 30 ? ORANGE
                 : batPct < 60 ? YELLOW : GREEN;
        char pct[12], volt[12];
        snprintf(pct, sizeof(pct), "%d%%", batPct);
        if (batMv > 1000) snprintf(volt, sizeof(volt), "%.1fV", batMv / 1000.0);
        else              snprintf(volt, sizeof(volt), "--");
        g->setTextColor(bcol, BLACK);
        g->setTextDatum(textdatum_t::middle_left);  g->drawString(pct,  4,   CY_BAT);
        g->setTextColor(WHITE, BLACK);
        g->setTextDatum(textdatum_t::middle_right); g->drawString(volt, 131, CY_BAT);
    }

    // ── 定位（Font4，带内居中）。定位中：FIX 左 + 卫星数 右；否则居中 SEARCH/INIT/ERROR/OFF ──
    g->setFont(&fonts::Font4);
    { const char* gw = gpsWord(&col);
      g->setTextColor(col, BLACK);
      if (gpsState == GS_FIX_GOOD) {
          g->setTextDatum(textdatum_t::middle_left);
          g->drawString("FIX", 4, CY_GPS);
          g->setTextDatum(textdatum_t::middle_right);
          snprintf(line, sizeof(line), "%u", (unsigned)fixSats());
          g->drawString(line, 131, CY_GPS);
      } else if (gpsState == GS_SEARCHING) {
          // 搜星中显示可见星数；持续 0 星过久 → 极可能天线/接线问题。
          // （模块不给真天线检测命令，ANTENNA NMEA 又不到主串口，只能靠"持续 0 星"反推）
          if (fixSats() == 0 && tSatsZeroSince != 0
                  && millis() - tSatsZeroSince >= GNSS_NOSAT_WARN_MS) {
              g->setTextColor(RED, BLACK);
              g->setTextDatum(textdatum_t::middle_center);
              g->drawString("ANT?", 67, CY_GPS);
          } else {
              g->setTextColor(YELLOW, BLACK);
              g->setTextDatum(textdatum_t::middle_left);
              g->drawString("SRCH", 4, CY_GPS);
              g->setTextDatum(textdatum_t::middle_right);
              snprintf(line, sizeof(line), "%u", (unsigned)fixSats());
              g->drawString(line, 131, CY_GPS);
          }
      } else {
          g->setTextDatum(textdatum_t::middle_center);
          g->drawString(gw, 67, CY_GPS);
      } }

    // ── 网络（Font4，带内居中）：READY/SEND/OK/ERROR/INIT/OFF ──
    { const char* nw = netWord(&col);
      g->setTextDatum(textdatum_t::middle_center);
      g->setTextColor(col, BLACK);
      g->drawString(nw, 67, CY_NET); }

    // ── 底部三行 UP/TX/BUF（Font2，标签左 / 数值右，全白；BUF 失败时转红）──
    g->setFont(&fonts::Font2);
    g->setTextColor(WHITE, BLACK);

    // 运行时长 UP 99h 59m
    {
        uint32_t m = millis() / 60000UL;
        g->setTextDatum(textdatum_t::middle_left);  g->drawString("UP", 4, CY_UP);
        snprintf(line, sizeof(line), "%luh %lum",
                 (unsigned long)(m / 60), (unsigned long)(m % 60));
        g->setTextDatum(textdatum_t::middle_right); g->drawString(line, 131, CY_UP);
    }

    // TX 上次上传距今 1h 29m（仅时+分；从未上传显示 -----）
    g->setTextDatum(textdatum_t::middle_left);  g->drawString("TX", 4, CY_TX);
    if (flLastUpload) {
        uint32_t m = (millis() - flLastUpload) / 60000UL;
        snprintf(line, sizeof(line), "%luh %lum", (unsigned long)(m / 60), (unsigned long)(m % 60));
    } else snprintf(line, sizeof(line), "-----");
    g->setTextDatum(textdatum_t::middle_right); g->drawString(line, 131, CY_TX);

    // BUF 积压：BF <pt> PT / <sg> SG（失败时整行转红）
    {
        uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
        g->setTextColor(catmFailStreak ? RED : WHITE, BLACK);
        g->setTextDatum(textdatum_t::middle_left);  g->drawString("BF", 4, CY_BUF);
        snprintf(line, sizeof(line), "%lu PT / %u SG", (unsigned long)pts, (unsigned)seg);
        g->setTextDatum(textdatum_t::middle_right); g->drawString(line, 131, CY_BUF);
    }
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

// 开/关屏。开 → 唤醒 + 设 1 分钟亮屏窗口 + 立即重绘；关 → 睡眠省电。
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

// ═══════════════════════════════════════════════════════════════════════════
// 配置钩子（被 firmware.ino 的 setup()/loop() 调用）
// ═══════════════════════════════════════════════════════════════════════════

// setup：CatM UART 之后 → 早点起 LCD，整个 init 过程都亮屏显示进度。
static void configSetupEarly() {
    lcdInit();
    displayOn = lcdOK;            // 开机即亮（含整个 init 过程）
    lcdBootMsg("starting...");
    // flashLogBegin() 已挪到共用 setup()（配置A/B 都挂载 LittleFS）。
}

// setup：catmInit 之前 → 屏上提示"LTE init..."。
static void configSetupPreNet() {
    lcdBootMsg("LTE init...");
}

// setup：对时之后 → 进 GNSS 跟踪模式（让出网络给 GNSS），并先亮 1 分钟屏。
// 之后 loop 轮询 CGNSINF，到 beacon 时 sendGpsData() 临时切回 LTE 发包再切回。
static void configSetupPostNet() {
    if (catmReady) {
        lcdBootMsg("GNSS init...");
        // bearer 保持 active（对时时已激活）；进 GNSS 跟踪只开 CGNSPWR，绝不 CNACT=0,0
        // （抽掉 bearer 会让 SH 句柄残留→下次 SHCONN 假锁；实测 CGNSPWR 不影响 bearer）。
        Serial.println("[CM] enter GNSS tracking mode (CGNSPWR=1, bearer 保持 active)");
        catmCmd("AT+CGNSPWR=1", 3000);
        gnssTracking  = true;
        tSatsZeroSince = 0;            // GNSS 刚开，0 星计时从首次轮询起算（给冷启 ~3min 再报 ANT?）
        gpsState      = GS_SEARCHING;
        catmState     = CM_READY;
        refreshCatmLed();
    }
    // 开机先亮 1 分钟（相当于按了一下），随后自动息屏。
    displaySetOn(true);
}

// loop 顶：GNSS 跟踪模式时定期轮询 CGNSINF 更新 liveFix。
static void configLoopFeed(uint32_t now) {
    if (catmReady && gnssTracking && now - tLastGnssPoll >= 1500) {
        tLastGnssPoll = now;
        pollGnssIntoLiveFix();
    }
}

// loop：超时自动息屏 + 亮屏时定期重绘状态。
static void configLoopDisplay(uint32_t now) {
    if (displayOn) {
        if ((int32_t)(now - tDisplayOff) >= 0) displaySetOn(false);
        else if (now - tLastLcdDraw >= LCD_DRAW_MS) lcdDrawStatus();
    }
}

// 采样前：先抓 NMEA 填 CN0/星座（配置B 的 CGNSINF 不带这些）。
static void configLoopPrePwrlog() {
    gnssSampleNmea();
}

// 配置B 跳过对时重试：GNSS 跟踪期跑 SHCONN 会和 GNSS 冲突；开机时已对过一次时。
static void configLoopSync(uint32_t now) { (void)now; }

// 到 beacon 点：把当前定位记进 LittleFS 段日志（不切射频、不发网，开销 ~1ms）。
static void configBeaconAction() {
    TrackPoint p; buildTrackPoint(p);
    flashLogAppend(p);
    uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
    Serial.printf("[FL] 记录点 → 积压 %lu 点 / %u 封段\n", (unsigned long)pts, seg);
}

// flush 专用：在有限预算内把网络弄到"可发 HTTPS"（已注册 + 有 IPv4 PDP）。
// 已 active+IPv4 → 秒过；否则在 FL_REG_WAIT_MS 内等注册 + 激活一次 PDP，仍不行就放弃返回 false。
// 关键：绝不像 catmCheckNet 那样 3 轮干等 ~2-3min（切回 LTE 网络偶尔没及时回来时会冻结+红灯）——
// 存转不丢数据，宁可 ~35-40s 放弃、保留积压、下个时机重试（下次大概率已附着就传成）。
static bool catmFlushNetReady() {
    String r = catmCmd("AT+CNACT?", 5000);
    if (r.indexOf("+CNACT: 0,1,") >= 0 && catmHasIPv4(r)) return true;   // 已就绪
    if (!catmWaitReg(FL_REG_WAIT_MS)) return false;                       // 注册没及时回来 → 放弃
    catmCmd("AT+CNCFG=0,1,\"" CATM_APN "\"", 3000);                       // 激活一次 PDP
    catmCmd("AT+CNACT=0,1", 12000);
    delay(1500);
    return catmHasIPv4(catmCmd("AT+CNACT?", 3000));
}

// 切 GNSS→LTE、上传所有封段（最旧先发、200即删）、再切回 GNSS。
// forced=true 为长按强制（无视静止/无定位条件，连当前半段也封进来一起发）。
static void flashFlushViaLte(bool forced) {
    uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
    if (pts == 0) { if (forced) Serial.println("[FL] 无积压可传"); return; }

    catmState = CM_SENDING; refreshCatmLed();
    Serial.printf("[FL] 切 LTE 上传 %lu 点%s…\n", (unsigned long)pts, forced ? "(强制)" : "");
    catmCmd("AT+CGNSPWR=0", 3000);                 // 出 GNSS，射频回 LTE（bearer 全程保持）
    gnssTracking = false;
    if (catmFailStreak >= CATM_FAIL_REATTACH) { catmForceIPv4(); catmFailStreak = 0; }

    int uploaded = 0;
    if (catmFlushNetReady()) uploaded = flashLogUpload();  // 逐段发、200即删；catmSHOpen 内含 SH 自愈
    else Serial.println("[FL] 网络未及时就绪（~35s内），留积压下轮重试，不长等");

    catmCmd("AT+CGNSPWR=1", 3000);                 // 切回 GNSS 继续跟踪
    gnssTracking = true; tLastGnssPoll = 0; tSatsZeroSince = 0;   // GNSS 重启→0星计时重置，免误报 ANT?

    if (uploaded > 0) {
        catmFailStreak = 0; flLastUpload = millis();
        flStillSince = flNoFixSince = 0;           // 复位计时，避免同一静止/无定位窗口反复触发
        catmState = CM_OK; refreshCatmLed(); delay(600); catmState = CM_READY;
        uint32_t remain; flashLogCounts(nullptr, &remain);
        Serial.printf("[FL] 上传完成 %d 点，剩余 %lu 点\n", uploaded, (unsigned long)remain);
    } else {
        catmFailStreak++; catmState = CM_ERR;
        flStillSince = flNoFixSince = 0;   // ★失败也清计时：统一回退 ~5min 再试，杜绝"无定位+持续失败"
                                            //   时 flushDue 恒真→每 30-75s 贴着重试的死循环(切射频/撞锁/掉电)。
        Serial.println("[FL] 上传未成功，保留积压（计时复位，约 5min 后再试）");
    }
    refreshCatmLed();
}

// loop：配置B 的 flush 调度器（每轮跑，开销忽略）。
// 维护"静止/无定位"计时，满足 flushDue()（攒满≥1段 且 静止≥5min 或 无定位≥5min）即
// 切 LTE 上传积压。空积压时什么都不做——传完即空，空了不再判断，直到攒出新封段。
static void configLoopRecover(uint32_t now) {
    if (!catmReady) return;
    bool hasFix = (gpsState == GS_FIX_GOOD);
    if (hasFix) {
        flNoFixSince = 0;
        float spd = fixHasSpeed() ? fixSpdKmh() : 0.0f;
        if (spd < FL_STILL_KMH) { if (!flStillSince) flStillSince = now; }
        else flStillSince = 0;
    } else {
        flStillSince = 0;
        if (!flNoFixSince) flNoFixSince = now;
    }

    uint16_t seg; uint32_t pts; flashLogCounts(&seg, &pts);
    FlushInputs in;
    in.sealedSegs = seg;
    in.stillMs    = flStillSince ? (now - flStillSince) : 0;
    in.noFixMs    = flNoFixSince ? (now - flNoFixSince) : 0;
    in.forced     = false;
    if (flushDue(in) && !flSchedHold) {
        Serial.printf("[FL] 触发上传：%u 封段，静止 %lus / 无定位 %lus\n",
                      seg, (unsigned long)(in.stillMs / 1000), (unsigned long)(in.noFixMs / 1000));
        flashFlushViaLte(false);
    }
}

// 顶部按钮短按：亮屏/熄屏开关。亮屏 1 分钟后自动熄；亮着时再按立刻熄。
// （手动上传改由长按"强制上传"承担；移动中本就自动 beacon。）
static void configOnTopShortPress() {
    Serial.printf("[BTN] top short press → display %s\n", displayOn ? "OFF" : "ON");
    displaySetOn(!displayOn);
}

// 顶部按钮长按：立即强制 flush 积压（不等静止/无定位，让用户主动选时机上传）。
static void configForceUpload() {
    Serial.println("[FL] 长按 → 强制上传积压");
    displaySetOn(true);            // 长按到点立即亮屏给反馈：看得到 Net=SEND、完后 BUF 归零（解决"以为没触发"）
    flashFlushViaLte(true);
}

#endif  // GNSS_TIMESHARE
