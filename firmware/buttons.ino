// buttons.ino — 按键状态机（顶部 BTN_OK / 黄色圆钮 GPIO11）+ 省电关机
// 配置A/B 共用。顶部按钮短按的语义两配置不同（A=上传 / B=亮息屏），抽到
// configOnTopShortPress() 钩子里，本文件保持配置无关。
#include "defs.h"

// ═══════════════════════════════════════════════════════════════════════════
// Power-saving shutdown
// Cuts every peripheral power rail (so the coprocessor's standby current is
// minimal — the built-in BtnPWR double-click leaves these on, which drains the
// battery), then writes the coprocessor power-off command (REG 0xE0).  Charging
// is left enabled so it can still charge while off.  If power isn't actually cut
// (e.g. external power present), we reboot to restore a known-good state.
// Powers the device off, so it normally never returns.  Press BtnPWR to wake.
// ═══════════════════════════════════════════════════════════════════════════
static void powerSaveShutdown() {
    Serial.println("[PWR] long-press → power-saving shutdown");
    phPower(PC_USB,     false);
    phPower(PC_UART,    false);   // PORT.C — GPS module
    phPower(PC_I2C,     false);   // PORT.A — CatM module
    phPower(PC_VAMETER, false);   // INA226 current/voltage monitors
    phPower(PC_BUS,     false);   // RS485/CAN
    phPower(PC_LED,     false);   // all LEDs
    // PC_CHARGE left ON so the battery still charges while powered off.
    delay(50);
    Serial.println("[PWR] writing power-off (REG 0xE0)");
    Serial.flush();
    phWr8(REG_OFF, 1);            // coprocessor cuts power
    delay(2500);
    // Still alive → power-off was blocked (VIN present?). Reboot to restore.
    Serial.println("[PWR] still powered — rebooting to restore rails");
    ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════
// Buttons (all active-low: idle reads 1, pressed reads 0):
//   Big top button (BTN_OK, REG_BTN bit0)  → short press   → configOnTopShortPress()
//                                           → long-press 3s → FORCE CatM upload, ignore
//                                                             GPS fix (bench diagnostic)
//   Yellow round button (GPIO11/BTN_SELECT) → double-click  → toggle USB-A power
//                                           → long-press 3s → power-saving shutdown
// ═══════════════════════════════════════════════════════════════════════════

static void checkButtons() {
    uint32_t now = millis();

    // ── Big top button (BTN_OK) ──────────────────────────────────────────────
    //   short press   → 配置相关（见 configOnTopShortPress：A=上传 | B=亮/息屏）
    //   long-press 3s → force a CatM upload regardless of GPS fix, so the POST
    //                   path can be exercised/diagnosed on the bench (indoors,
    //                   where there is no fix and uploads otherwise never fire).
    // REG_BTN is active-low, so a pressed button reads 0 → invert to get "down".
    bool okNow = !phBtn(BTN_OK_BIT);
    if (okNow && !prevBtnOK) {                   // press begins
        tOkPress    = now;
        okLongFired = false;
    }
    if (okNow && !okLongFired && now - tOkPress >= LONGPRESS_MS) {
        okLongFired  = true;                     // fire once, mid-hold
        forceSendReq = true;
        Serial.println("[BTN] top button long-press → FORCE CatM upload (ignore fix)");
    }
    if (!okNow && prevBtnOK) {                    // release
        if (!okLongFired)                         // short press → 配置相关动作（钩子）
            configOnTopShortPress();
    }
    prevBtnOK = okNow;

    // ── Yellow round button (GPIO11) ─────────────────────────────────────────
    //   double-click  → toggle USB-A power
    //   long-press 3s → power-saving shutdown
    // Also active-low: pressed = LOW, so invert digitalRead (this was reversed
    // before, which caused a phantom click on every boot).
    bool selNow = !(bool)digitalRead(BTN_SELECT_GPIO);

    if (selNow && !prevBtnSel) {                 // press begins
        tSelPress    = now;
        selLongFired = false;
    }
    if (selNow && !selLongFired && now - tSelPress >= LONGPRESS_MS) {
        selLongFired = true;                     // fire once, mid-hold
        powerSaveShutdown();                     // powers off — does not return
    }
    if (!selNow && prevBtnSel) {                 // release
        if (!selLongFired) {                     // short press → double-click FSM
            if (clickCount == 0) {
                clickCount  = 1;
                tFirstClick = now;
            } else {
                if (now - tFirstClick <= DBLCLICK_MS) {
                    // ── Double-click confirmed ────────────────────────────────
                    usbEnabled = !usbEnabled;
                    phPower(PC_USB, usbEnabled);
                    refreshUsbLed();
                    Serial.printf("[USB] toggled → %s\n", usbEnabled ? "ON" : "OFF");
                }
                clickCount = 0;
            }
        }
    }
    if (clickCount == 1 && now - tFirstClick > DBLCLICK_MS)
        clickCount = 0;          // lone single click = no action

    prevBtnSel = selNow;
}
