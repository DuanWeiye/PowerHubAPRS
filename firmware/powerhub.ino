// powerhub.ino — PowerHub I2C 读写、电源/LED 控制、电池电压换算、各 LED 刷新
// 配置A/B 共用。所有全局状态定义在 firmware.ino，多 .ino 单编译单元下可直接访问。
#include "defs.h"

// ═══════════════════════════════════════════════════════════════════════════
// PowerHub I2C helpers
// ═══════════════════════════════════════════════════════════════════════════

static void phRd(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(PH_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)PH_ADDR, n);
    for (uint8_t i = 0; i < n; i++)
        buf[i] = Wire.available() ? Wire.read() : 0;
}
static void phWr(uint8_t reg, const uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(PH_ADDR);
    Wire.write(reg);
    Wire.write(buf, n);
    Wire.endTransmission();
}
static uint8_t phRd8(uint8_t reg)            { uint8_t v; phRd(reg,&v,1); return v; }
static void    phWr8(uint8_t reg, uint8_t v) { phWr(reg,&v,1); }

static bool phInit() {
    Wire.end();                   // reset any previous state
    Wire.begin(SYS_SDA, SYS_SCL);
    Wire.setClock(100000);        // start at 100 kHz — more forgiving during boot

    // STM32 on PowerHub needs ~500 ms after power-on to initialise its I2C.
    // (The official IDF firmware has vTaskDelay(500) here for the same reason.)
    delay(700);

    for (int attempt = 0; attempt < 8; attempt++) {
        Wire.beginTransmission(PH_ADDR);
        if (Wire.endTransmission() == 0) {
            Wire.setClock(400000);    // switch to 400 kHz once confirmed
            Serial.printf("[PH] Found at 0x%02X (attempt %d)\n", PH_ADDR, attempt + 1);
            return true;
        }
        delay(150);
    }

    // Not found — scan the entire bus so the user can see what is actually there
    Serial.println("[PH] Scanning I2C bus for diagnostics:");
    bool any = false;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  found: 0x%02X\n", a);
            any = true;
        }
    }
    if (!any) Serial.println("  (no devices found — check SDA/SCL wiring)");
    return false;
}

static void phPower(uint8_t port, bool on) {
    phWr8(REG_PWR + port, on ? 1 : 0);
}

// Only issues I2C write when color actually changes (cache-guarded)
static void phLedSet(uint8_t idx, uint32_t rgb) {
    if (idx >= 8 || ledCache[idx] == rgb) return;
    ledCache[idx] = rgb;
    uint8_t b[3] = {(uint8_t)rgb, (uint8_t)(rgb>>8), (uint8_t)(rgb>>16)};
    phWr(LED_CR[idx], b, 3);
}

static void phLedBright(uint8_t idx, uint8_t br) {
    if (idx < 8) phWr8(LED_BR[idx], br);
}

static uint16_t phVolt(uint8_t mon) {
    uint8_t b[2]; phRd(VA_VR[mon],b,2); return (uint16_t)(b[1]<<8)|b[0];
}
static int16_t phCurr(uint8_t mon) {
    uint8_t b[2]; phRd(VA_IR[mon],b,2); return (int16_t)((b[1]<<8)|b[0]);
}
static bool phBtn(uint8_t bitPos) {
    return (phRd8(REG_BTN) >> bitPos) & 1;
}

// ═══════════════════════════════════════════════════════════════════════════
// Battery: 2S LiPo voltage → percentage
// ═══════════════════════════════════════════════════════════════════════════

static int voltToPercent(uint16_t mv) {
    // 2S LiPo discharge curve. This PowerHub board's charge IC terminates at
    // ~8.1V (≈4.05V/cell) by design — it never reaches the 8.4V cell maximum,
    // to extend battery life. So we treat 8100mV and above as 100% and rescale
    // the rest of the curve accordingly. 6.8V (≈3.4V/cell) is the cut-off (0%).
    static const uint16_t V[] = {6800,7000,7200,7400,7600,7800,8000,8100};
    static const int      P[] = {   0,  10,  20,  35,  55,  75,  92, 100};
    const int N = 8;
    if (mv <= V[0])   return 0;
    if (mv >= V[N-1]) return 100;
    for (int i = 1; i < N; i++) {
        if (mv <= V[i])
            return P[i-1] + (int)((int32_t)(mv-V[i-1])*(P[i]-P[i-1])/(V[i]-V[i-1]));
    }
    return 100;
}

static void readBattery() {
    uint16_t mv  = phVolt(VM_BAT);
    uint8_t  chg = phRd8(REG_CHG);
    uint8_t  pwr = phRd8(REG_PSUP);
    batMv = mv;                 // 供 LCD 显示（INA226 就绪前为 0）
    // INA226 needs ~200 ms after PC_VAMETER is enabled to produce the first
    // reading; before that phVolt() returns 0.  Only latch valid when mv>1V.
    if (mv > 1000) {
        batPct   = voltToPercent(mv);
        batValid = true;
    }
    hasExtPower = (pwr != 0);
    Serial.printf("[BAT] %u mV  %d%%  chg=0x%02X pwr=0x%02X extPwr=%d valid=%d\n",
                  mv, batPct, chg, pwr, hasExtPower, batValid);
}

// ═══════════════════════════════════════════════════════════════════════════
// LED refresh functions
// ═══════════════════════════════════════════════════════════════════════════

static void refreshGpsLed() {
    uint32_t c;
    switch (gpsState) {
        case GS_DETECTING:  c = C_OFF;    break;
        case GS_NO_MODULE:  c = C_OFF;    break;
        case GS_INIT_OK:    c = C_BLUE;   break;
        case GS_SEARCHING:  c = C_YELLOW; break;
        case GS_FIX_GOOD:   c = C_GREEN;  break;
        case GS_INIT_FAIL:  c = C_RED;    break;
        default:            c = C_OFF;
    }
    phLedSet(GPS_LED, c);
}

// Battery LED — always shows battery level.  Blue = reading not yet available.
static void refreshBatLed() {
    uint32_t c;
    if (!batValid) {
        c = C_BLUE;                        // INA226 not ready yet at startup
    } else if (batPct < 10) {
        c = blinkPhase ? C_RED : C_OFF;   // critical blink
    } else if (batPct < 30) {
        c = C_RED;
    } else if (batPct < 60) {
        c = C_YELLOW;
    } else {
        c = C_GREEN;
    }
    phLedSet(BAT_LED, c);
}

// External-power indicator — LED adjacent to small round button.
// Blue = any external power present (charging OR data cable).  Off = on battery.
static void refreshPowerLed() {
    phLedSet(CHG_LED, hasExtPower ? C_BLUE : C_OFF);
}

// CatM status LED (PORT.A adjacent)
static void refreshCatmLed() {
    uint32_t c;
    switch (catmState) {
        case CM_OFF:     c = C_OFF;    break;
        case CM_INIT:    c = C_YELLOW; break;
        case CM_READY:   c = C_BLUE;   break;
        case CM_SENDING: c = C_WHITE;  break;
        case CM_OK:      c = C_GREEN;  break;
        case CM_ERR:     c = C_RED;    break;
        default:         c = C_OFF;
    }
    phLedSet(CATM_LED, c);
}

// USB output LEDs — side USB-C (LED_USB_C) and USB-A (LED_USB_A) are paired outputs.
// Both follow the same double-click toggle and share one current sensor (VM_USB).
// Blue = output enabled, no load.  Green = load detected on either port.
static void refreshUsbLed() {
    if (!usbEnabled) {
        phLedSet(LED_USB_C, C_OFF);
        phLedSet(USB_LED,   C_OFF);
        return;
    }
    int16_t cur = phCurr(VM_USB);
    uint32_t c = (cur > USB_LOAD_MA || cur < -USB_LOAD_MA) ? C_GREEN : C_BLUE;
    phLedSet(LED_USB_C, c);
    phLedSet(USB_LED,   c);
}
