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

// ═══════════════════════════════════════════════════════════════════════════
// 分时发包：当前在 GNSS 跟踪模式 → 切 LTE → 发包 → 切回 GNSS 继续跟踪。
// ═══════════════════════════════════════════════════════════════════════════
static bool sendGpsData(bool queueOnFail) {
    TrackPoint cur;
    buildTrackPoint(cur);                 // 用当前 liveFix
    catmState = CM_SENDING; refreshCatmLed();

    catmCmd("AT+CGNSPWR=0", 3000);        // 出 GNSS，射频交还 LTE
    gnssTracking = false;
    if (catmFailStreak >= CATM_FAIL_REATTACH) {   // 连续失败 → 强制 IPv4 重附着
        Serial.printf("[CM] %u consecutive failures -> IPv4 re-attach\n", catmFailStreak);
        catmForceIPv4();
        catmFailStreak = 0;
    }
    // ★ bearer 全程保持 active（开机/首发已激活）：实测 CGNSPWR 开关 GNSS 完全不影响
    //   PDP（IP 不变）。绝不再 CNACT=0,0/0,1 反复抽建 bearer——那会让 SH(HTTP)应用的
    //   连接句柄随 bearer 被抽掉而残留，下次 SHCONN 撞 "operation not allowed"（SH 假锁，
    //   官方流程也是 bearer 一次性激活后保持、不每请求 toggle）。
    // 实测：CGNSPWR=0 后加长延时(4s)反而锁更多——时序不是触发点，CGNSPWR 射频切换本身
    // 就会扰乱 SH(TLS)栈（这是 SIM7080G GNSS+TLS 分时的固有弱点）。故不加延时，直接判网；
    // 撞锁时由 catmPostBody 内部先轻量复位、不行再 CFUN=1,1。

    bool ok = false;
    if (catmCheckNet()) {                  // 确认 bearer 仍有可用 IPv4（active 时零副作用）
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

    catmCmd("AT+CGNSPWR=1", 3000);        // 切回 GNSS 继续跟踪（bearer 不动，全程 active）
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

// ═══════════════════════════════════════════════════════════════════════════
// 配置钩子（被 firmware.ino 的 setup()/loop() 调用）
// ═══════════════════════════════════════════════════════════════════════════

// setup：CatM UART 之后 → 早点起 LCD，整个 init 过程都亮屏显示进度。
static void configSetupEarly() {
    lcdInit();
    displayOn = lcdOK;            // 开机即亮（含整个 init 过程）
    lcdBootMsg("starting...");
}

// setup：catmInit 之前 → 屏上提示"LTE init..."。
static void configSetupPreNet() {
    lcdBootMsg("LTE init...");
}

// setup：对时之后 → 进 GNSS 跟踪模式（让出网络给 GNSS），并先亮 30s 屏。
// 之后 loop 轮询 CGNSINF，到 beacon 时 sendGpsData() 临时切回 LTE 发包再切回。
static void configSetupPostNet() {
    if (catmReady) {
        lcdBootMsg("GNSS init...");
        // bearer 保持 active（对时时已激活）；进 GNSS 跟踪只开 CGNSPWR，绝不 CNACT=0,0
        // （抽掉 bearer 会让 SH 句柄残留→下次 SHCONN 假锁；实测 CGNSPWR 不影响 bearer）。
        Serial.println("[CM] enter GNSS tracking mode (CGNSPWR=1, bearer 保持 active)");
        catmCmd("AT+CGNSPWR=1", 3000);
        gnssTracking  = true;
        gpsState      = GS_SEARCHING;
        catmState     = CM_READY;
        refreshCatmLed();
    }
    // 开机先亮 30s（相当于按了一下），随后自动息屏。
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

// loop：配置B 红灯处理（不依赖定位）。
// 红灯=发包失败；失败的 beacon 会把点入队(trackEnqueue)，所以红灯≈"有积压待发"。
// 原则(用户 06-19"只做有必要的事")：只有"确有积压要发"时才临时切 LTE 探网；
// 没有待发数据就别折腾网络，安心追 GPS，并把无意义的红灯清回蓝。
static void configLoopRecover(uint32_t now) {
    if (catmReady && gnssTracking && catmState == CM_ERR && gpsState != GS_FIX_GOOD) {
        if (trackCount == 0) {
            // 无积压 → 红灯没有意义（没东西要发）→ 清为蓝，继续追踪，不切网
            catmState = CM_READY;
            refreshCatmLed();
        } else if (now - tLastCatmRecover >= CATM_FAIL_RETRY_MS) {
            tLastCatmRecover = now;
            Serial.printf("[CM] B 红灯恢复：%u 条积压待发，切 LTE 探网…\n", trackCount);
            catmCmd("AT+CGNSPWR=0", 3000);            // 出 GNSS，射频回 LTE（bearer 全程保持）
            gnssTracking = false;
            // bearer 不动（CGNSPWR 不影响 PDP）；只等射频从 GNSS 切回 LTE 重新就绪。
            catmWaitReg(8000);                        // 一直在网，通常秒过
            if (catmFailStreak >= CATM_FAIL_REATTACH) { catmForceIPv4(); catmFailStreak = 0; }
            uint16_t before = trackCount;
            if (catmCheckNet()) trackFlush();         // bearer 仍 active；catmPostBody 内含 SH 自愈
            if (trackCount < before) {                // 只有积压真发出去（队列变短）才清红，绝不假恢复
                Serial.println("[CM] B 恢复：积压补发成功 → 清红");
                catmState = CM_READY; catmFailStreak = 0;
            } else {
                catmFailStreak++;                      // 仍发不出，保持红
            }
            catmCmd("AT+CGNSPWR=1", 3000);            // 切回 GNSS 继续跟踪（bearer 不动）
            gnssTracking  = true;
            tLastGnssPoll = 0;
            refreshCatmLed();
        }
    }
}

// 顶部按钮短按：亮屏/熄屏开关。亮屏 30s 后自动熄；亮着时再按立刻熄。
// （手动上传改由长按"强制上传"承担；移动中本就自动 beacon。）
static void configOnTopShortPress() {
    Serial.printf("[BTN] top short press → display %s\n", displayOn ? "OFF" : "ON");
    displaySetOn(!displayOn);
}

#endif  // GNSS_TIMESHARE
