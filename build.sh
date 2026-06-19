#!/bin/bash
# build.sh — M5Stack PowerHub firmware build & flash
#
# Usage:
#   ./build.sh              # build only
#   ./build.sh -f /dev/ttyACM0   # build + flash
#
# Board note:
#   M5Stack arduino-esp32 2.1.4 has no dedicated PowerHub profile.
#   We use m5stack_cores3 (CoreS3) as it shares the same silicon:
#     ESP32-S3 / 16 MB Flash (QIO) / 8 MB OPI PSRAM / HW-CDC USB
#   AtomS3R (atoms3r) matches best: ESP32-S3 + OPI PSRAM.
#   Flash declared as 8MB (atoms3r only offers this); PowerHub's extra 8MB
#   is simply not addressed. Upgrade FQBN if a PowerHub profile is added.

set -e

# 工具链路径可用环境变量覆盖（默认值适配标准 Arduino 1.8.19 安装）
ARDUINO="${ARDUINO:-$HOME/Downloads/arduino-1.8.19/arduino}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/m5stack/tools/esptool_py/4.5.1/esptool.py}"

# 仓库内路径相对脚本所在目录，便于他人 clone 后直接用
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$SCRIPT_DIR/firmware/firmware.ino"

FQBN="m5stack:esp32:m5stack_atoms3r:\
JTAGAdapter=default,\
PSRAM=opi,\
FlashMode=qio,\
FlashSize=8M,\
LoopCore=1,\
EventsCore=1,\
USBMode=hwcdc,\
CDCOnBoot=cdc,\
MSCOnBoot=default,\
DFUOnBoot=default,\
UploadMode=default,\
PartitionScheme=default_8MB,\
CPUFreq=240,\
UploadSpeed=921600,\
DebugLevel=none,\
EraseFlash=none"

BUILD_DIR="/tmp/m5power_build"
OUT_DIR="$SCRIPT_DIR"
BIN_NAME="m5power.bin"

FLASH_PORT=""

# AtomS3R 稳定设备 ID（ESP32-S3 内置 USB-JTAG，MAC 固定 → by-id 路径不随枚举顺序变）。
# -f 不带参数时用它，避免误刷到 ttyACM0/ttyUSB* 上的其它设备（如 M5Paper）。
ATOMS3R_DEV="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_1C:DB:D4:A8:27:C4-if00"

# ── Parse arguments ────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            # 端口可选：不给（或下一个又是选项）则用写死的 AtomS3R ID
            if [[ -n "$2" && "$2" != -* ]]; then
                FLASH_PORT="$2"; shift 2
            else
                FLASH_PORT="$ATOMS3R_DEV"; shift 1
            fi
            ;;
        -h|--help)
            echo "Usage: $0 [-f [port]]"
            echo "  (no args)   Build only"
            echo "  -f          Build then flash to the fixed AtomS3R device ID"
            echo "  -f <port>   Build then flash to an explicit port"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Run '$0 --help' for usage."
            exit 1
            ;;
    esac
done

mkdir -p "$BUILD_DIR"

# ── Build ──────────────────────────────────────────────────────────────────
echo "=== Building M5Power firmware ==="
echo "    FQBN: $FQBN"
echo "    Sketch: $SKETCH"

"$ARDUINO" \
    --board "$FQBN" \
    --pref "build.path=$BUILD_DIR" \
    --verify \
    "$SKETCH"

# ── Copy output ───────────────────────────────────────────────────────────
BIN_SRC="$BUILD_DIR/firmware.ino.bin"
if [[ ! -f "$BIN_SRC" ]]; then
    echo "ERROR: expected binary not found: $BIN_SRC"
    exit 1
fi

cp "$BIN_SRC" "$OUT_DIR/$BIN_NAME"
SIZE=$(stat -c%s "$OUT_DIR/$BIN_NAME")
echo "=== Build done: $OUT_DIR/$BIN_NAME  (${SIZE} bytes) ==="

# ── Flash (optional) ──────────────────────────────────────────────────────
if [[ -n "$FLASH_PORT" ]]; then
    echo ""
    echo "=== Flashing to $FLASH_PORT ==="

    BOOTLOADER="$BUILD_DIR/firmware.ino.bootloader.bin"
    PARTITIONS="$BUILD_DIR/firmware.ino.partitions.bin"
    BOOT_APP="$HOME/.arduino15/packages/m5stack/hardware/esp32/2.1.4/tools/partitions/boot_app0.bin"

    for f in "$BOOTLOADER" "$PARTITIONS" "$BOOT_APP"; do
        if [[ ! -f "$f" ]]; then
            echo "ERROR: required file missing: $f"
            exit 1
        fi
    done

    python3 "$ESPTOOL" \
        --chip    esp32s3 \
        --port    "$FLASH_PORT" \
        --baud    921600 \
        --before  default_reset \
        --after   hard_reset \
        write_flash \
        -z \
        --flash_mode  dio \
        --flash_freq  80m \
        --flash_size  8MB \
        0x00000 "$BOOTLOADER" \
        0x08000 "$PARTITIONS" \
        0x0e000 "$BOOT_APP" \
        0x10000 "$OUT_DIR/$BIN_NAME"

    echo "=== Flash done ==="
fi
