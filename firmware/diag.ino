// diag.ino — 现场诊断命令（串口触发）：PORT.A 波特率扫描 / GNSS<->LTE 分时切换测速。
// 配置A/B 共用（gnsstest / atscan 命令在两配置下都可用，用来评估二合一模块/排线）。
#include "defs.h"

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

// ═══════════════════════════════════════════════════════════════════════════
// GNSS↔LTE 分时切换测速（评估 SIM7080G 二合一模块是否可行；串口命令 gnsstest 触发）
// 前提：PORT.A 插的是 Unit CatM GNSS（带 GNSS 天线，天线见天）。复用 catmCmd / 已就绪的模块。
// 测的关键数：切回 GNSS 后到再次 fix=1 的秒数（"秒切回来"成立与否）。
// ═══════════════════════════════════════════════════════════════════════════
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
