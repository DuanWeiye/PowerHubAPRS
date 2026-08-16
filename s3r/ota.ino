// ota.ino — UART-OTA 接收器 + A/B 槽启动确认/回滚 + $S3R 协议分发
//
// 更新链路：DGX ota_flash.py → (台面: S3R 自身 USB | 装机后: PowerHub USB 透传桥)
//           → $S3R,OTA 协议 → Update 库写入另一个 app 槽 → 重启 → PING 确认。
//
// 防变砖：esp32 Arduino 核未开 bootloader 级回滚，这里用 NVS 自管状态机——
//   OTA 写完 → pend=1 重启；新固件每次启动 attempts++（otaBootGuard，setup 最先跑）；
//   收到 PING → pend=0（确认合格）；attempts 超限仍未确认 → 切回旧槽重启。
//   崩溃循环/僵死固件靠 PowerHub phPower(PC_UART) 断电重启来累计 attempts 触发回滚。
//   刷完必 PING 是 ota_flash.py 的固定流程，正常情况几秒内即确认。
//   （残余风险：坏固件在 setup 最顶前就崩（静态构造函数）→ 守卫跑不到，只能拆机 USB
//    兜底。守卫之前不放任何自定义静态构造，风险压到最低。）
#include "defs.h"
#include <Update.h>
#include "esp_ota_ops.h"

static const char* otaRunningPartition() {
    const esp_partition_t* p = esp_ota_get_running_partition();
    return p ? p->label : "?";
}

// setup() 最先调用：待确认固件的启动计数与回滚
static void otaBootGuard() {
    Preferences op;
    op.begin("s3rota", false);
    if (op.getUChar("pend", 0)) {
        uint8_t att = op.getUChar("att", 0) + 1;
        op.putUChar("att", att);
        otaPending = true;
        if (att > OTA_MAX_ATTEMPTS) {
            // 多次启动都没等到确认 → 认定新固件坏死，切回旧槽
            op.putUChar("pend", 0);
            op.putUChar("att", 0);
            op.end();
            if (Update.canRollBack() && Update.rollBack()) ESP.restart();
            // 旧槽不可用（不应发生）：只能带着 pend 清零继续跑当前槽
            otaPending = false;
            return;
        }
    }
    op.end();
}

// 收到 PING → 确认当前固件合格
static void otaConfirmPending(Stream* io) {
    if (!otaPending) return;
    Preferences op;
    op.begin("s3rota", false);
    op.putUChar("pend", 0);
    op.putUChar("att", 0);
    op.end();
    otaPending = false;
    io->println("$S3R,CONFIRMED");
    Serial.println("[OTA] firmware confirmed by ping");
}

// ═══════════════════════════════════════════════════════════════════════════
// $S3R 协议分发（来自链路或 USB，io 指向来源串口，应答原路返回）
// ═══════════════════════════════════════════════════════════════════════════
static void s3rHandleLine(const char* line, Stream* io) {
    if (strncmp(line, "$S3R,PING", 9) == 0) {
        otaConfirmPending(io);
        io->printf("$S3R,PONG,%s,%s\r\n", S3R_FW_VER, otaRunningPartition());
        return;
    }
    if (strncmp(line, "$S3R,INFO", 9) == 0) {
        uint32_t ilTotal; uint16_t ilSegs, ilSumSegs;
        imulogStats(&ilTotal, &ilSegs, &ilSumSegs);
        io->printf("$S3R,INFO,ver=%s,part=%s,pend=%d,up=%lus,heap=%lu,"
                   "gpsRX=G%d,linkRX=G%d,bps=%lu,csOK=%lu,csFail=%lu,"
                   "sats=%u,hdop=%.1f,ant=%u,imu=%d,fuse=%d,mode=%c,log=%luK/%u\r\n",
                   S3R_FW_VER, otaRunningPartition(), otaPending ? 1 : 0,
                   (unsigned long)(millis() / 1000), (unsigned long)ESP.getFreeHeap(),
                   gpsRxPin, linkRxPin, (unsigned long)gpsBps,
                   (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum(),
                   gps.satellites.isValid() ? (unsigned)gps.satellites.value() : 0,
                   gps.hdop.isValid() ? gps.hdop.hdop() : -1.0f, gnssAnt,
                   imuOk() ? 1 : 0, fuseEnabledGet() ? 1 : 0,
                   fuseIsLatched() ? 'L' : (fuseIsDr() ? 'D' : 'P'),
                   (unsigned long)(ilTotal / 1024), ilSegs);
        return;
    }
    if (strncmp(line, "$S3R,FUSE,", 10) == 0) {
        fuseSetEnabled(strncmp(line + 10, "ON", 2) == 0);
        io->printf("$S3R,FUSE,%s\r\n", fuseEnabledGet() ? "ON" : "OFF");
        return;
    }
    if (strncmp(line, "$S3R,IMUDUMP", 12) == 0) {
        imulogDump(io);
        return;
    }
    if (strncmp(line, "$S3R,IMULOG,CLEAR", 17) == 0) {
        imulogClear();
        io->println("$S3R,IMULOG,CLEARED");
        return;
    }
    if (strncmp(line, "$S3R,IMU", 8) == 0) {   // 注意须排在 IMUDUMP/IMULOG 之后（前缀）
        io->printf("$S3R,IMU,ok=%d,i2c=%d,who=%02X/%02X,stat=%d,since=%lus,acc=%.3f,"
                   "gyro=%.2f,hr=%.2f,bias=%d,fuse=%d,mode=%c\r\n",
                   imuOk() ? 1 : 0, imuI2cOk ? 1 : 0, imuWho69, imuWho68,
                   imuIsStationary() ? 1 : 0,
                   imuIsStationary() ? (unsigned long)((millis() - imuStationarySince()) / 1000) : 0UL,
                   imuAccStd(), imuGyroMag(), imuHeadingRateDps(),
                   imuBiasKnown() ? 1 : 0, fuseEnabledGet() ? 1 : 0,
                   fuseIsLatched() ? 'L' : (fuseIsDr() ? 'D' : 'P'));
        return;
    }
    if (strncmp(line, "$S3R,OTA,BEGIN,", 15) == 0) {
        // $S3R,OTA,BEGIN,<size>,<md5hex32>
        uint32_t size = strtoul(line + 15, nullptr, 10);
        const char* comma = strchr(line + 15, ',');
        if (!size || !comma || strlen(comma + 1) != 32) {
            io->println("$S3R,OTA,ERR,begin-args");
            return;
        }
        otaReceive(io, size, comma + 1);
        return;
    }
    io->printf("$S3R,ERR,unknown:%s\r\n", line);
}

// ═══════════════════════════════════════════════════════════════════════════
// OTA 传输（BEGIN 之后阻塞直到完成/失败；期间透传暂停，GPS 流丢弃防陈旧）
// ═══════════════════════════════════════════════════════════════════════════

// zlib 兼容 CRC32（与 ota_flash.py 的 zlib.crc32 一致），逐块校验早发现线路误码
static uint32_t crc32sw(const uint8_t* d, size_t n) {
    uint32_t c = 0xFFFFFFFFUL;
    while (n--) {
        c ^= *d++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (~((c & 1) - 1)));
    }
    return ~c;
}

// 带截止时刻的读一行（丢弃空行；返回 false = 超时）
static bool otaReadLine(Stream* io, char* buf, size_t cap, uint32_t deadline) {
    size_t n = 0;
    while ((int32_t)(deadline - millis()) > 0) {
        while (gpsSerial.available()) gpsSerial.read();   // OTA 期间 GPS 流直接丢弃
        int c = io->read();
        if (c < 0) { delay(1); continue; }
        if (c == '\n' || c == '\r') {
            if (!n) continue;
            buf[n] = 0;
            return true;
        }
        if (n < cap - 1) buf[n++] = (char)c;
    }
    return false;
}

// 读满 len 字节块体
static bool otaReadBody(Stream* io, uint8_t* buf, size_t len, uint32_t deadline) {
    size_t n = 0;
    while (n < len && (int32_t)(deadline - millis()) > 0) {
        while (gpsSerial.available()) gpsSerial.read();
        int avail = io->available();
        if (avail <= 0) { delay(1); continue; }
        size_t want = len - n;
        if ((size_t)avail < want) want = avail;
        n += io->readBytes(buf + n, want);
    }
    return n == len;
}

static void otaReceive(Stream* io, uint32_t size, const char* md5hex) {
    Serial.printf("[OTA] begin size=%lu md5=%s\n", (unsigned long)size, md5hex);
    if (!Update.begin(size)) {
        io->printf("$S3R,OTA,ERR,begin:%s\r\n", Update.errorString());
        return;
    }
    Update.setMD5(md5hex);
    io->println("$S3R,OTA,READY");
    screenOtaProgress(0, size);

    static uint8_t body[OTA_CHUNK_MAX];
    char     hl[64];
    uint32_t expSeq = 0, got = 0;
    uint32_t tActive = millis();

    for (;;) {
        if (millis() - tActive > OTA_IDLE_ABORT_MS) {
            Update.abort();
            io->println("$S3R,OTA,ERR,idle-timeout");
            Serial.println("[OTA] idle timeout, aborted");
            screenRedraw();
            return;
        }
        if (!otaReadLine(io, hl, sizeof(hl), millis() + OTA_LINE_TMO_MS)) continue;
        tActive = millis();

        if (strncmp(hl, "$S3R,OTA,DATA,", 14) == 0) {
            // $S3R,OTA,DATA,<seq>,<len>,<crc32hex>\n + <len> 字节原始数据
            char* end;
            uint32_t seq = strtoul(hl + 14, &end, 10);
            if (*end != ',') { io->printf("$S3R,OTA,NAK,%lu,hdr\r\n", (unsigned long)seq); continue; }
            uint32_t len = strtoul(end + 1, &end, 10);
            if (*end != ',' || !len || len > OTA_CHUNK_MAX) {
                io->printf("$S3R,OTA,NAK,%lu,len\r\n", (unsigned long)seq); continue;
            }
            uint32_t crc = strtoul(end + 1, nullptr, 16);
            if (!otaReadBody(io, body, len, millis() + OTA_DATA_TMO_MS)) {
                io->printf("$S3R,OTA,NAK,%lu,body\r\n", (unsigned long)seq); continue;
            }
            if (crc32sw(body, len) != crc) {
                io->printf("$S3R,OTA,NAK,%lu,crc\r\n", (unsigned long)seq); continue;
            }
            if (seq + 1 == expSeq) {                    // 上一块的重发（ACK 丢了）→ 幂等重 ACK
                io->printf("$S3R,OTA,ACK,%lu\r\n", (unsigned long)seq); continue;
            }
            if (seq != expSeq) {                        // 乱序（不应发生）
                io->printf("$S3R,OTA,NAK,%lu\r\n", (unsigned long)seq); continue;
            }
            if (Update.write(body, len) != len) {
                Update.abort();
                io->printf("$S3R,OTA,ERR,write:%s\r\n", Update.errorString());
                screenRedraw();
                return;
            }
            expSeq++;
            got += len;
            io->printf("$S3R,OTA,ACK,%lu\r\n", (unsigned long)seq);
            if ((expSeq & 15) == 0 || got == size) screenOtaProgress(got, size);
            continue;
        }
        if (strncmp(hl, "$S3R,OTA,END", 12) == 0) {
            if (Update.end(true)) {                     // true = 校验 MD5 + 完整性
                Preferences op;
                op.begin("s3rota", false);
                op.putUChar("pend", 1);                 // 新固件待确认，重启后守卫接管
                op.putUChar("att", 0);
                op.end();
                io->println("$S3R,OTA,OK");
                Serial.println("[OTA] flash OK, rebooting into new firmware");
                screenBootMsg("OTA OK, reboot");
                delay(300);
                ESP.restart();
            }
            io->printf("$S3R,OTA,ERR,end:%s\r\n", Update.errorString());
            Serial.printf("[OTA] end FAIL: %s\n", Update.errorString());
            screenRedraw();
            return;
        }
        if (strncmp(hl, "$S3R,OTA,ABORT", 14) == 0) {
            Update.abort();
            io->println("$S3R,OTA,ERR,aborted");
            Serial.println("[OTA] aborted by host");
            screenRedraw();
            return;
        }
        // 其它行（NMEA 回声等）忽略
    }
}
