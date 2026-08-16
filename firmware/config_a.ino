// config_a.ino — 配置A 专属逻辑（仅当 GNSS_TIMESHARE==0 编译，否则整文件为空）。
//   配置A = PORT.C 独立 ATGM336H 连续 NMEA + PORT.A 的 SIM7080G 专做 4G。
// 这里实现：GPS 状态机、发包、以及 setup()/loop() 的配置钩子。
// 位置访问器(fixLat/fixLon/...)已挪到 track.ino，A/B 共用一份实现，读野点过滤+卡尔曼平滑
// 之后的 liveFix；本文件只在 configLoopFeed() 里把 TinyGPS++ 解出的原始定位喂进那套流水线。
#include "defs.h"
#if !GNSS_TIMESHARE

// ═══════════════════════════════════════════════════════════════════════════
// GPS state machine update（PORT.C 连续 NMEA）
// ═══════════════════════════════════════════════════════════════════════════
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
    // 用 liveFix 的新鲜度(而非直接读 gps.location)：fixLat()/fixLon() 现在读的是野点过滤+
    // 卡尔曼平滑之后的 liveFix（见 track.ino gnssFeedLiveFix），状态机也该跟着同一份数据走，
    // 否则会出现"状态机说有定位，但实际 fixLat() 还停在上一个野点过滤前的旧值"的不一致。
    // liveFix.tMs 由 gnssFeedLiveFix() 在每次真正接受(非野点)的新定位时刷新，语义等价于
    // 原来的 gps.location.age()<5000。
    if (gpsState == GS_SEARCHING || gpsState == GS_FIX_GOOD) {
        bool good = liveFix.valid && (millis() - liveFix.tMs < 5000);

        if (good && gpsState != GS_FIX_GOOD) {
            Serial.printf("[GPS] Fix  lat=%.6f lon=%.6f hdop=%.1f sat=%u\n",
                liveFix.lat, liveFix.lon, liveFix.hdop, liveFix.sats);
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

// ═══════════════════════════════════════════════════════════════════════════
// 长阻塞操作(发包/补发/开机初始化)结束后调用：把阻塞期间积压的 NMEA 整段丢弃。
//
// 为什么必须丢：RX 缓冲 4096B 只够吸收 ~5-10s 的三系统 NMEA(GSV 全开约 400-900B/s)，
// 而发包阻塞 15-25s、恢复补发更久——溢出几乎必然。ESP32 HardwareSerial 溢出时**丢最新、
// 留最旧**，事后排空解析出的"最后一个定位"其实是阻塞初期的旧位置；若配上当前 millis 喂进
// 滤波流水线，就是"旧位置新时间戳"：步行时注入 ~20m 假点(恰是在治的漂移量级)，电车速度下
// 假点与 1s 后真定位的隐含速度会超 GLITCH_MAX_MPS → 野点门连拒→强制放行→KF 重置，每次
// 发包后都白白折腾一轮。旧数据本无价值(下一个新鲜定位 1s 内就到)，整段丢弃最干净；丢弃
// 造成的 dt(~25s) 走 KF 大方差预测路径，天然安全（>30s 则由 GLITCH_RESYNC 重建，也安全）。
// 丢弃可能截断半句 NMEA，TinyGPS++ 在下一个 '$' 自动重新同步，无需处理。
// ═══════════════════════════════════════════════════════════════════════════
static void gpsDrainStale() {
    uint32_t n = 0;
    while (gpsSerial.available()) { gpsSerial.read(); n++; }
    if (n) {
        tLastNmeaByte = millis();   // 丢的也是流——链路看门狗只关心"有没有流"
        Serial.printf("[GPS] drained %lu stale NMEA bytes after blocking op\n",
                      (unsigned long)n);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// POST the current GPS fix to the home server via HTTPS. On failure the point is
// queued for later (store-and-forward); on success any backlog is flushed too.
// queueOnFail=false for the bench/forced path, where coords may be stale or 0,0.
// ═══════════════════════════════════════════════════════════════════════════
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
        if (queueOnFail) flashLogAppend(cur);   // 发失败(无信号) → 落 Flash 段日志(断电不丢)
        gpsDrainStale();   // checkNet 重试也阻塞了几十秒 → 丢弃积压的旧 NMEA
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
        flashLogUpload();         // 网络恢复 → 立刻把 Flash 积压全部补发（A 独立蜂窝，不怕 GPS 失明）
    } else {
        catmFailStreak++;
        catmState = CM_ERR;       // red — stays until the next attempt
        refreshCatmLed();
        if (queueOnFail) flashLogAppend(cur);   // 发失败(无信号) → 落 Flash 段日志(断电不丢)
    }
    gpsDrainStale();   // 发包阻塞 15-25s 必溢出 RX 缓冲 → 丢弃旧 NMEA，只吃新鲜定位
    return ok;
}

// ═══════════════════════════════════════════════════════════════════════════
// 配置钩子（被 firmware.ino 的 setup()/loop() 调用）
// ═══════════════════════════════════════════════════════════════════════════

// setup：CatM UART 之后 → 开 PORT.C 的 GPS 串口 + 一次性配置 GPS 模块。
static void configSetupEarly() {
    // ── GPS serial ───────────────────────────────────────────────────────────
    // RX 缓冲 4096B 只够吸收 ~5-10s 的三系统 NMEA(约 400-900B/s)，撑不过 15-25s 的发包
    // 阻塞——溢出后缓冲里剩的是"阻塞初期"的旧数据(ESP32 丢最新留最旧)。所以不指望它保数据：
    // 各阻塞操作结束后一律 gpsDrainStale() 整段丢弃，绝不把旧定位当新鲜数据喂滤波。
    gpsSerial.setRxBufferSize(4096);
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
    // PCAS02,200 = 定位更新间隔 200ms(5Hz)：采样多 5 倍给 KF 平均增益，拐弯响应更细。
    //   流量涨到 ~4.5KB/s(GSV 也 5Hz)，115200 波特(11.5KB/s)吃得下；阻塞期旧数据本就由
    //   gpsDrainStale 丢弃，野点门 dt 也已做 1s 下限解耦——5Hz 的前置条件都已就位。
    {
        const uint8_t GPS_CFG_VER = 3;     // 改配置时 +1，强制下次开机重配一次
        Preferences gpsPrefs;
        gpsPrefs.begin("gps", false);
        if (gpsPrefs.getUChar("cfgver", 0) != GPS_CFG_VER) {
            delay(200);
            gpsSerial.print("$PCAS10,9*15\r\n");   // 一次性出厂启动 + 使能串口&射频
            delay(500);                            //  (清星历→本次冷启动一回，之后热启动)
            gpsSerial.print("$PCAS04,15*2D\r\n");  // GPS(+QZSS)+BDS+GLONASS+Galileo 全开
            delay(100);
            gpsSerial.print("$PCAS02,200*1D\r\n"); // 5Hz 定位更新
            delay(100);
            gpsSerial.print("$PCAS00*01\r\n");     // 存入模块 flash，掉电不丢
            delay(200);
            gpsPrefs.putUChar("cfgver", GPS_CFG_VER);
            Serial.println("[GPS] one-time config: RF + 4-GNSS + 5Hz, saved");
        } else {
            Serial.println("[GPS] config persisted -> hot start (no re-config this boot)");
        }
        gpsPrefs.end();
    }

    // S3R 链路计时基准（看门狗 3min ≫ setup 阻塞 ~25s，从这里起算即可）
    tLastNmeaByte = tLastS3rPing = millis();
}

// 配置A 在 PreNet 时机无事可做（LCD / GNSS 跟踪是配置B 专属）。
static void configSetupPreNet()  {}
// setup 里 catmInit+对时阻塞了 10-30s，期间 GPS 串口已开、NMEA 已在积压——同发包后一样
// 丢弃旧数据，避免开机第一个定位就是"旧位置新时间戳"（开机就在电车上时会触发连拒）。
static void configSetupPostNet() { gpsDrainStale(); }

// loop 顶：喂 GPS 解析器（每轮都跑，最高优先级；RX 缓冲在发包阻塞期吸收 NMEA）。
static void configLoopFeed(uint32_t now) {
    bool gotBytes = false;
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gotBytes = true;
        gps.encode(c);
        // Raw NMEA dump for the first GPS_RAW_DUMP_MS — helps diagnose
        // whether the module sees any satellites (look for $GNGSV SNR values).
        if (now - tBoot < GPS_RAW_DUMP_MS)
            Serial.write(c);
        // 装配整行 → $S3R 协议行给链路处理，其余喂 GNSS 信号诊断（CN0/星座/天线）
        if (c == '\n' || c == '\r') {
            if (nmeaLen) {
                nmeaLine[nmeaLen] = 0;
                if (!strncmp(nmeaLine, "$S3R,", 5)) s3rLinkLine(nmeaLine);
                else                                gnssDiagLine(nmeaLine);
                nmeaLen = 0;
            }
        } else if (nmeaLen < sizeof(nmeaLine) - 1) {
            nmeaLine[nmeaLen++] = c;
        } else {
            nmeaLen = 0;   // 行超长，丢弃
        }
    }
    if (gotBytes) tLastNmeaByte = now;   // 链路看门狗的"有流"时间戳
    // TinyGPS++ 刚解出一个新定位 → 喂野点过滤+卡尔曼平滑，写共享 liveFix(见 track.ino
    // gnssFeedLiveFix)。isUpdated() 读一次即自清，和 isValid()/age() 是各自独立的标志，
    // 不影响别处（比如下面 updateGps 的搜星诊断日志）继续读 gps.location 的原始状态。
    if (gps.location.isUpdated() && gps.location.isValid()) {
        gnssFeedLiveFix(gps.location.lat(), gps.location.lng(), now,
                        gps.altitude.isValid() ? gps.altitude.meters() : 0.0f,
                        gps.speed.isValid(), gps.speed.isValid() ? gps.speed.kmph() : 0.0f,
                        gps.course.isValid(), gps.course.isValid() ? gps.course.deg() : -1.0f,
                        gps.hdop.isValid(), gps.hdop.isValid() ? gps.hdop.hdop() : 25.5f,
                        gps.satellites.isValid(), gps.satellites.isValid() ? (uint8_t)gps.satellites.value() : 0);
    }
    s3rLinkTick(now);   // S3R 心跳 + NMEA 断流看门狗
}

// 配置A 无 LCD、采样前无需抓 NMEA（GPS 流里已带 GSV）。
static void configLoopDisplay(uint32_t now)  { (void)now; }
static void configLoopPrePwrlog()            {}

// loop：对时重试（开机若 PDP 没就绪会失败，这里每 60s 重试）+ eDRX 授权周期一次性回读。
static void configLoopSync(uint32_t now) {
    // catmSyncTime() may fail at boot if PDP isn't ready yet; retry here.
    if (catmReady && !catmTimeSynced && now - tLastSyncAttempt >= 60000) {
        tLastSyncAttempt = now;
        catmTimeSynced = catmSyncTime();
    }
    // A successful time sync means attached + PDP active, so the network has
    // finished negotiating eDRX — only now does CEDRXRDP report the real granted
    // cycle (the 3rd field), not 0. Logged once for diagnostics.
    if (catmReady && catmTimeSynced && !edrxChecked) {
        edrxChecked = true;
        Serial.printf("[CM] eDRX granted: %s\n", catmCmd("AT+CEDRXRDP", 3000).c_str());
    }
}

// loop：红灯但 GPS 没有定位时的恢复（不依赖定位）。
// 下面的 beacon 发送整块被 GS_FIX_GOOD 门控，因此一次失败留下的红灯只能在"有定位"
// 的前提下重试 / 重附着 / 补发积压点。一旦红灯期间又丢了定位（进楼、城市峡谷、地下、
// 回到室内），恢复逻辑就永远不触发，红灯无限锁死——2026-06-13 实测：盲区丢定位后红灯
// 卡死 30 分钟。这里把恢复做成不依赖定位：哪怕没定位也让模组有机会恢复、并把积压队列
// 发出去。有定位的情况完全不变（仍由 beaconDue 的 "retry" 处理），两条路径不会重复发送。
static void configLoopRecover(uint32_t now) {
    if (catmReady && catmState == CM_ERR && gpsState != GS_FIX_GOOD
            && now - tLastCatmRecover >= CATM_FAIL_RETRY_MS) {
        tLastCatmRecover = now;
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
        uint32_t pts; flashLogCounts(nullptr, &pts);
        if (pts > 0) {
            if (flashLogUpload() > 0) {   // catmSHOpen 内含 SH 锁 CFUN=1,1 自愈；只有真发出去才清红
                Serial.println("[CM] recover (no fix): 积压补发成功 -> 清红");
                catmState = CM_READY;
                refreshCatmLed();
            } else {
                catmFailStreak++;     // 仍发不出，保持红，下个周期再试
            }
        }
        gpsDrainStale();   // 重附着/补发都长阻塞 → 丢弃期间积压的旧 NMEA
    }
}

// 顶部按钮短按：请求上传当前 GPS 位置（移动中本就自动 beacon，这是手动补一发）。
static void configOnTopShortPress() {
    Serial.println("[BTN] top button short press → request GPS upload");
    manualSendReq = true;
}

// 到 beacon 点的动作（配置A：实时直发，与改造前完全一致）。
static void configBeaconAction() { sendGpsData(true); }

// 长按大按钮（配置A：bench 诊断——忽略定位强发一包，原行为）。
static void configForceUpload() {
    Serial.println("[CM] === FORCED bench upload (long-press, ignoring GPS fix) ===");
    sendGpsData(false);          // bench/diagnostic：不污染轨迹队列
    recordAnchor();
    decayInterval = DECAY_START_MS;
}

// ═══════════════════════════════════════════════════════════════════════════
// S3R 协处理器链路：透传桥 / 心跳 / 链路看门狗（见 s3r/HANDOFF.md 待办1）
// GPS 可能直连 PORT.C 也可能经 AtomS3R 转接，三件对两种接法都成立。
// ═══════════════════════════════════════════════════════════════════════════

// 链路上收到 $S3R 行（心跳应答 PONG / OTA 确认 CONFIRMED / INFO 应答…）→ 记存活+打日志。
// 行频最高 1 条/分钟（心跳应答），直接全量进日志，联测/排障都用得上。
static void s3rLinkLine(const char* s) {
    tS3rLastSeen = millis();
    Serial.printf("[S3R] %s\n", s);
}

// 每 ~60s 心跳 + NMEA 断流看门狗（configLoopFeed 末尾每轮调用）
static void s3rLinkTick(uint32_t now) {
    // 心跳：S3R 收到即确认待确认固件（OTA 防变砖闭环）并回 PONG；GPS 直连时
    // ATGM336H 忽略未知语句，无害。
    if (now - tLastS3rPing >= S3R_PING_MS) {
        tLastS3rPing = now;
        gpsSerial.print("$S3R,PING\r\n");
    }
    // 链路看门狗：NMEA 断流超阈值 → PORT.C 断电重启整链（S3R+GPS 或裸 GPS 同样适用）。
    // 长阻塞操作（发包 15-25s、恢复补发）后 gpsDrainStale 会刷新时间戳，不会误触发。
    if (now - tLastNmeaByte >= S3R_LINK_WDT_MS) {
        Serial.printf("[S3R] link watchdog: NMEA 断流 %lus → PORT.C 断电 %lums 重启整链\n",
                      (unsigned long)((now - tLastNmeaByte) / 1000),
                      (unsigned long)S3R_WDT_OFF_MS);
        phPower(PC_UART, false);
        delay(S3R_WDT_OFF_MS);
        phPower(PC_UART, true);
        gpsDrainStale();
        tLastNmeaByte = millis();
        // 卡在 NO_MODULE 的状态机不再参与检测；断电重启后给一次重新检测的机会。
        // （INIT_FAIL 不重置：TinyGPS 校验计数累计，坏流未修好会立即再判失败，无意义。）
        if (gpsState == GS_NO_MODULE) {
            gpsState = GS_DETECTING;
            tBoot = millis();   // 检测窗口重新起算（raw dump 起点复用同一基准，无碍）
        }
    }
}

// USB↔PORT.C 透传桥：给装机后的 S3R 做 UART-OTA。ota_flash.py 开口后先发一行
// "s3rbridge" 进入本模式，之后字节原样双向转发（协议见 s3r/ota_flash.py 头注释）。
// 阻塞独占 loop：期间 LED/按键/beacon/喂流全部暂停——维护操作，可接受。
// 空闲退出而非命令退出：刷完后不再有 USB 数据，60s 后自动恢复正常运行。
static void s3rBridgeMode() {
    Serial.printf("[S3R] bridge: USB<->PORT.C 透传开始（%lus 无 USB 数据自动退出）\n",
                  (unsigned long)(S3R_BRIDGE_IDLE_MS / 1000));
    gpsDrainStale();               // 只转发新鲜字节
    uint8_t buf[256];
    uint32_t tUsb = millis();
    for (;;) {
        // USB → 链路（256B 一批；UART 115200 写满阻塞 ~22ms，反压由 USB-CDC NAK 兜住）
        int n = 0;
        while (n < (int)sizeof(buf) && Serial.available()) buf[n++] = (uint8_t)Serial.read();
        if (n) { gpsSerial.write(buf, n); tUsb = millis(); }
        // 链路 → USB（NMEA 噪声混在其中，ota_flash.py 侧按 $S3R 前缀过滤）
        int m = 0;
        while (m < (int)sizeof(buf) && gpsSerial.available()) buf[m++] = (uint8_t)gpsSerial.read();
        if (m) Serial.write(buf, m);
        if (millis() - tUsb >= S3R_BRIDGE_IDLE_MS) break;
        if (!n && !m) delay(1);    // 空转让出 CPU，喂 RTOS 看门狗
    }
    Serial.println("[S3R] bridge: 空闲超时退出，恢复正常运行");
    gpsDrainStale();
    tLastNmeaByte = millis();      // 桥内没喂解析器 → 重置看门狗基准，避免退出即误触发
}

#endif  // !GNSS_TIMESHARE
