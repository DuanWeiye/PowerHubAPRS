#!/bin/bash
# build.sh — M5Stack PowerHub firmware build & flash
#
# Usage:
#   ./build.sh                    # build only
#   ./build.sh -f                 # build + flash
#   ./build.sh -w                 # flash an existing .bin only (no build)
#   ./build.sh -w --bin <file>    # flash a specific .bin (no build)
#
# 刷机目标设备写死为下面的 ATOMS3R_DEV（by-id 路径，稳定不随枚举变），不接受端口
# 参数，从根上避免误刷到 ttyACM0/ttyUSB* 上的其它板子。多带的端口参数会被忽略。
#
# Flash-only (-w) does NOT rebuild — it writes an already-compiled .bin straight
# to the device. Default bin is ./m5power.bin (app partition only @0x10000, for
# re-flashing the same partition scheme). If the chosen .bin sits next to a full
# image set (bootloader.bin + partitions.bin + boot_app0.bin, as in
# releases/configA/), -w writes the whole set instead — a full restore.
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
DO_BUILD=1        # -w 关掉它：跳过编译，直接刷已有 .bin
BIN_OVERRIDE=""   # --bin 指定要刷的 .bin（默认 $OUT_DIR/$BIN_NAME）

# AtomS3R 稳定设备 ID（ESP32-S3 内置 USB-JTAG，MAC 固定 → by-id 路径不随枚举顺序变）。
# 刷机目标写死为它：-f/-w 都只刷这一台，避免误刷到 ttyACM0/ttyUSB* 上的其它设备（如 M5Paper）。
ATOMS3R_DEV="/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_1C:DB:D4:A8:27:C4-if00"

# ── Parse arguments ────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            FLASH_PORT="$ATOMS3R_DEV"; shift 1     # 目标写死，不取端口参数
            ;;
        -w)
            DO_BUILD=0                              # flash-only：跳过编译，直接刷已有 .bin
            FLASH_PORT="$ATOMS3R_DEV"; shift 1     # 目标写死，不取端口参数
            ;;
        --bin)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "ERROR: --bin needs a file path"; exit 1
            fi
            BIN_OVERRIDE="$2"; shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [-f] | [-w [--bin <file>]]"
            echo "  (no args)        Build only"
            echo "  -f               Build then flash (full image) to the hardcoded AtomS3R"
            echo "  -w               Flash existing ./m5power.bin only (no build, app @0x10000)"
            echo "  -w --bin <file>  Flash a specific .bin (no build); full restore if a"
            echo "                   complete image set sits next to it (see releases/configA/)"
            echo "  注：刷机目标设备已写死，不接受端口参数"
            exit 0
            ;;
        *)
            # 设备 ID 写死后端口参数已无意义：误带的端口（不以 - 开头）直接忽略；
            # 但拼错的选项（以 - 开头）仍报错，免得静默刷错 bin。
            if [[ "$1" == -* ]]; then
                echo "Unknown option: $1"
                echo "Run '$0 --help' for usage."
                exit 1
            fi
            echo "Note: 忽略多余参数 '$1'（刷机设备已写死，无需端口）"
            shift 1
            ;;
    esac
done

mkdir -p "$BUILD_DIR"

# ── Build (skipped with -w) ─────────────────────────────────────────────────
if [[ $DO_BUILD -eq 1 ]]; then
    echo "=== Building M5Power firmware ==="
    echo "    FQBN: $FQBN"
    echo "    Sketch: $SKETCH"

    "$ARDUINO" \
        --board "$FQBN" \
        --pref "build.path=$BUILD_DIR" \
        --verify \
        "$SKETCH"

    # ── Copy output ─────────────────────────────────────────────────────────
    BIN_SRC="$BUILD_DIR/firmware.ino.bin"
    if [[ ! -f "$BIN_SRC" ]]; then
        echo "ERROR: expected binary not found: $BIN_SRC"
        exit 1
    fi

    cp "$BIN_SRC" "$OUT_DIR/$BIN_NAME"
    SIZE=$(stat -c%s "$OUT_DIR/$BIN_NAME")
    echo "=== Build done: $OUT_DIR/$BIN_NAME  (${SIZE} bytes) ==="
fi

# ── Flash (optional) ──────────────────────────────────────────────────────
if [[ -n "$FLASH_PORT" ]]; then
    # 要刷的 app .bin：--bin 优先，否则仓库根目录的 m5power.bin
    APP_BIN="${BIN_OVERRIDE:-$OUT_DIR/$BIN_NAME}"
    if [[ ! -f "$APP_BIN" ]]; then
        echo "ERROR: app binary not found: $APP_BIN"
        exit 1
    fi

    # 找配套的 bootloader/分区/boot_app0：
    #   -f（刚编译）：用 /tmp 构建产物
    #   -w（不编译）：找 app .bin 同目录的整套镜像（releases/configA/ 风格）
    BOOT_APP="$HOME/.arduino15/packages/m5stack/hardware/esp32/2.1.4/tools/partitions/boot_app0.bin"
    BOOTLOADER=""; PARTITIONS=""
    if [[ $DO_BUILD -eq 1 ]]; then
        BOOTLOADER="$BUILD_DIR/firmware.ino.bootloader.bin"
        PARTITIONS="$BUILD_DIR/firmware.ino.partitions.bin"
    else
        BIN_DIR="$(cd "$(dirname "$APP_BIN")" && pwd)"
        if [[ -f "$BIN_DIR/bootloader.bin" && -f "$BIN_DIR/partitions.bin" ]]; then
            BOOTLOADER="$BIN_DIR/bootloader.bin"
            PARTITIONS="$BIN_DIR/partitions.bin"
            [[ -f "$BIN_DIR/boot_app0.bin" ]] && BOOT_APP="$BIN_DIR/boot_app0.bin"
        fi
    fi

    # 有整套镜像就完整刷；否则只覆盖 app 分区（分区方案不变时够用）
    if [[ -n "$BOOTLOADER" && -f "$BOOTLOADER" && -n "$PARTITIONS" && -f "$PARTITIONS" && -f "$BOOT_APP" ]]; then
        echo ""
        echo "=== Flashing FULL image to $FLASH_PORT ==="
        echo "    app: $APP_BIN"
        WRITE_ARGS=(
            0x00000 "$BOOTLOADER"
            0x08000 "$PARTITIONS"
            0x0e000 "$BOOT_APP"
            0x10000 "$APP_BIN"
        )
    else
        echo ""
        echo "=== Flashing APP ONLY to $FLASH_PORT ==="
        echo "    app: $APP_BIN  (@0x10000，假定分区方案未变)"
        WRITE_ARGS=( 0x10000 "$APP_BIN" )
    fi

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
        "${WRITE_ARGS[@]}"

    echo "=== Flash done ==="
fi
