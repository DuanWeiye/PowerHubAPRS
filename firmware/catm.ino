// catm.ino — SIM7080G (Unit CatM) AT 命令层 + 网络/TLS/HTTPS POST + SH 锁死自愈
// 配置A/B 共用。部署参数（APN/服务器域名·端口·路径）来自 config.h（在 firmware.ino 顶部 include）。
#include "defs.h"

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
            // 必须匹配带冒号的结果 URC "+SHREQ:"（如 +SHREQ: "POST",200,22）；不能用
            // 裸 "+SHREQ"——命令回显 "AT+SHREQ=..." 含该子串，回显未关时会被误命中而
            // 28ms 就早退，漏读 ~100ms 后才到的真状态码（把 HTTP 200 错读成 0）。
            if (ret.indexOf("+SHREQ:") >= 0 && ret.endsWith("|")) break;
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

// SIM7080G 的 SH(HTTPS)应用会退化进「每个射频周期只放行一次 SHCONN、之后一律
// +CME ERROR: operation not allowed」的锁死态（与 GNSS/信号/PDP/注册都无关——
// 2026-06-19 台面 + 外场复现：CNACT/CEREG/IPv4 全正常，仍连不上）。实测恢复手段：
//   · CFUN=0/1（射频重启，catmForceIPv4 用的）→ 清不掉，只换来「再一次」会话又锁；
//   · CFUN=1,1（整模块软重启）→ 真正解锁，恢复连续多会话（实测连发 8 次全 200）。
// 故 SH 锁死必须靠 CFUN=1,1。本函数整模块重启并重建到「可发 HTTPS」的最小状态。
static bool catmSHRecover() {
    Serial.println("[CM] === SH 锁死恢复：AT+CFUN=1,1 整模块软重启 ===");
    catmCmd("AT+CFUN=1,1", 20000);    // 等模块重启到 RDY
    // CFUN=1,1 返回 RDY 后 UART 仍要十几秒才真正应答 AT（实测 ATE0/CMEE/CSCLK 全是
    // 空响应超时）。此时回显默认仍 ON、ATE0 丢进空气 → 后续命令回显残留，触发上面
    // catmCmd 的早退误判、漏读 HTTP 状态码。故先轮询 AT 真就绪，再做配置。
    bool atReady = false;
    for (int i = 0; i < 30; i++) {    // 最多 ~15s 等 UART 恢复
        if (catmCmd("AT", 1000).indexOf("|OK|") >= 0) { atReady = true; break; }
        delay(500);
    }
    Serial.printf("[CM] SH 恢复：模块 AT %s\n", atReady ? "就绪" : "仍无应答(仍继续配置)");
    catmCmd("ATE0", 2000);            // 此刻已就绪，关回显才能真正生效
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

// 打开一个 HTTPS(SH)会话：SHCONF/SSL 配置 + SHCONN 建链。若首次 SHCONN 命中 SH
// 锁死（operation not allowed），CFUN=1,1 整模块重启后把整段重做一遍再连（见
// catmSHRecover）。成功（SHSTATE=1）返回 true。
// ★会话复用：打开一次后可连发多次 catmSHReq，最后 catmSHClose 一次——这样上传一批
//   积压只做一次 TLS 握手，而不是每 8 个点重建一次（实测每次握手 0.8~2.4s，是大批量
//   上传的主要耗时；改造前 200 点要 30 次握手 ~75s）。bodyCap = SHCONF BODYLEN（≥单条
//   body；SIM7080G 上限 1024），整个会话设一次即可。
static bool catmSHOpen(int bodyCap) {
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
    if (!connected) return false;
    if (catmCmd("AT+SHSTATE?", 5000).indexOf("+SHSTATE: 1") == -1) {
        Serial.println("[CM] SHSTATE not 1");
        catmCmd("AT+SHDISC", 3000);
        return false;
    }
    return true;
}

// 在【已打开的 SH 会话】上发一个 POST 请求（body 到 PATH_APRS）。返回 HTTP 状态码
// （解析失败=0）。不建链、不断链——会话由 catmSHOpen/catmSHClose 管理，故连发多请求
// 只一次 TLS 握手。Headers 必须 SHCHEAD 后 SHAHEAD 前，否则 +CME ERROR → 无
// Content-Type → 服务器解析失败 → 400。
static int catmSHReq(const char* body, int bodyLen) {
    catmCmd("AT+SHCHEAD", 3000);
    catmCmd("AT+SHAHEAD=\"Content-Type\",\"application/json\"", 3000);
    String ret = catmCmd("AT+SHBOD=" + String(bodyLen) + ",3000", 5000);
    if (ret.indexOf(">") >= 0) {
        Serial2.write((const uint8_t*)body, bodyLen);
        Serial2.flush();                 // 阻塞到 UART 字节发完（115200 下每 KB ~87ms）
        delay(150);                      // 再留点余量让模组把 body 收进 SHBOD（保守 150ms）
    }
    ret = catmCmd("AT+SHREQ=\"" PATH_APRS "\",3", 15000);   // 30s→15s：瞬时丢包少干等
    int ci = ret.indexOf("\"POST\",");
    int code = 0;
    if (ci >= 0) {
        String s = ret.substring(ci + 7);
        code = s.substring(0, s.indexOf(",")).toInt();
    }
    return code;
}

// 关闭 SH 会话。
static void catmSHClose() { catmCmd("AT+SHDISC", 3000); }

// 单发包：开会话 → 发一个 body → 关会话。配置A 的单点上传走这里，行为与改造前一致；
// 返回 HTTP 状态码或 -1（建链失败）。批量上传请直接用 catmSHOpen + 多次 catmSHReq。
static int catmPostBody(const char* body, int bodyLen, int bodyCap) {
    if (!catmSHOpen(bodyCap)) return -1;
    int code = catmSHReq(body, bodyLen);
    catmSHClose();
    return code;
}
