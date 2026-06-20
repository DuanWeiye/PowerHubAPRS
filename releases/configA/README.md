# 配置A 固件快照（已验证可用）

冻结于 **2026-06-19**，对应 git commit **c77278e**
（"修复 SIM7080G SH(HTTPS)栈锁死 + 假恢复;新增 sendtest/AT 透传调试命令"）。

## 这是什么
**配置A** = 独立 ATGM336H GPS + 独立 Unit CatM(SIM7080G)，`GNSS_TIMESHARE 0`。
外场端到端验证通过：开机对时 + `sendtest` 均 HTTP 200，服务器 `aprs_track.db`
收到带定位的真实 beacon。含 SH 栈锁死自愈(`CFUN=1,1`)与假恢复修复。

留这份快照的目的：以后回去试**配置B**(二合一+LCD，会改 `firmware.ino`)时，
随时能把这套已知可用的配置A 原样刷回来，不必重编译。

## 内容（自包含整套刷机镜像）
| 文件 | 偏移 | 说明 |
|---|---|---|
| `firmware.ino`   | —       | 源码快照（与 commit c77278e 的 `firmware/firmware.ino` 一致） |
| `bootloader.bin` | 0x00000 | bootloader |
| `partitions.bin` | 0x08000 | 分区表（default_8MB） |
| `boot_app0.bin`  | 0x0e000 | OTA 选择扇区 |
| `m5power.bin`    | 0x10000 | 应用固件（app） |

## 怎么刷回配置A
脚本会自动识别本目录里带了整套支持镜像 → 走**完整刷写**：

```bash
# 完整刷回配置A（bootloader+分区+app），刷到写死的 AtomS3R 设备 ID
./build.sh -w --bin releases/configA/m5power.bin

# 指定端口
./build.sh -w --bin releases/configA/m5power.bin /dev/ttyACM0
```

只想快速覆盖 app 分区（分区方案不变时最常用，不重编译）：

```bash
./build.sh -w            # 刷仓库根目录的 m5power.bin（仅 app @0x10000）
```
