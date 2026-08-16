#!/bin/bash
# s3r/build.sh — AtomS3R GNSS 协处理器固件 build & flash
#
# Usage:
#   ./build.sh                    # 只编译
#   ./build.sh -f                 # 编译 + USB 完整刷写（首刷/救砖用）
#   ./build.sh -w                 # 只刷已有 ./s3r.bin（app @0x10000）
#   ./build.sh -o [--port <dev>]  # 编译 + UART-OTA（日常更新：装机后经 PowerHub 透传桥，
#                                 #   台面直测时 --port 省略即用 S3R 自身 USB 口）
#
# 刷机目标写死为 S3R_DEV（真 AtomS3R 的 by-id 路径）。注意主仓 build.sh 写死的是
# PowerHub（MAC 1C:DB:D4:A8:27:C4）——两台都是 ESP32-S3 USB-JTAG，靠 MAC 区分，别混用脚本。

set -e

ARDUINO="${ARDUINO:-$HOME/Downloads/arduino-1.8.19/arduino}"
ESPTOOL="${ESPTOOL:-$HOME/.arduino15/packages/m5stack/tools/esptool_py/4.5.1/esptool.py}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="$SCRIPT_DIR/s3r.ino"

# 真 AtomS3R：PSRAM=opi / 8MB Flash / USB-Serial-JTAG。
# CPUFreq=80：透传+屏显负载极低，80 是 USB 存活下限 = 省电最优点。
# PartitionScheme=default_8MB：app0/app1 双 OTA 槽（UART-OTA 的前提，首刷后不可改）。
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
CPUFreq=80,\
UploadSpeed=921600,\
DebugLevel=none,\
EraseFlash=none"

BUILD_DIR="/tmp/s3r_build"
OUT_DIR="$SCRIPT_DIR"
BIN_NAME="s3r.bin"

# 真 AtomS3R 的稳定设备 ID（内置 USB-JTAG，MAC 固定）
S3R_DEV="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:D5:B8:64-if00"

DO_BUILD=1
FLASH_PORT=""
DO_OTA=0
OTA_PORT="$S3R_DEV"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -f) FLASH_PORT="$S3R_DEV"; shift ;;
        -w) DO_BUILD=0; FLASH_PORT="$S3R_DEV"; shift ;;
        -o) DO_OTA=1; shift ;;
        --port)
            [[ -z "$2" || "$2" == -* ]] && { echo "ERROR: --port needs a device"; exit 1; }
            OTA_PORT="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [-f] | [-w] | [-o [--port <dev>]]"
            echo "  (no args)  只编译"
            echo "  -f         编译 + USB 完整刷写（首刷/救砖）"
            echo "  -w         只刷已有 ./s3r.bin（app @0x10000）"
            echo "  -o         编译 + UART-OTA（--port 指定串口，默认 S3R 自身 USB；"
            echo "             装机后给 PowerHub 的 USB 口 + 透传桥模式）"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

mkdir -p "$BUILD_DIR"

if [[ $DO_BUILD -eq 1 ]]; then
    echo "=== Building S3R firmware ==="
    "$ARDUINO" --board "$FQBN" --pref "build.path=$BUILD_DIR" --verify "$SKETCH"
    cp "$BUILD_DIR/s3r.ino.bin" "$OUT_DIR/$BIN_NAME"
    echo "=== Build done: $OUT_DIR/$BIN_NAME ($(stat -c%s "$OUT_DIR/$BIN_NAME") bytes) ==="
fi

if [[ -n "$FLASH_PORT" ]]; then
    if [[ $DO_BUILD -eq 1 ]]; then
        # 分区表用自制 partitions_s3r.csv（app 1.5MB×2 + spiffs 4.875MB），不是 IDE
        # 生成的 default_8MB——运行时布局以片上分区表为准，见 csv 头注释。
        python3 "$HOME/.arduino15/packages/m5stack/hardware/esp32/2.1.4/tools/gen_esp32part.py" \
            "$SCRIPT_DIR/partitions_s3r.csv" "$BUILD_DIR/partitions_s3r.bin"
        WRITE_ARGS=(
            0x00000 "$BUILD_DIR/s3r.ino.bootloader.bin"
            0x08000 "$BUILD_DIR/partitions_s3r.bin"
            0x0e000 "$HOME/.arduino15/packages/m5stack/hardware/esp32/2.1.4/tools/partitions/boot_app0.bin"
            0x10000 "$OUT_DIR/$BIN_NAME"
        )
        echo "=== Flashing FULL image (custom partitions) to $FLASH_PORT ==="
    else
        WRITE_ARGS=( 0x10000 "$OUT_DIR/$BIN_NAME" )
        echo "=== Flashing APP ONLY to $FLASH_PORT ==="
    fi
    python3 "$ESPTOOL" --chip esp32s3 --port "$FLASH_PORT" --baud 921600 \
        --before default_reset --after hard_reset \
        write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB \
        "${WRITE_ARGS[@]}"
    echo "=== Flash done ==="
fi

if [[ $DO_OTA -eq 1 ]]; then
    echo "=== UART-OTA via $OTA_PORT ==="
    python3 "$SCRIPT_DIR/ota_flash.py" --port "$OTA_PORT" --bin "$OUT_DIR/$BIN_NAME"
fi
