# M5Power APRS Tracker

基于 **M5Stack AtomS3R + PowerHub** 的低功耗 GPS 轨迹追踪器。通过 Cat-M (LTE-M)
蜂窝网络把定位点以 HTTPS POST 上报到自建服务器。支持两种硬件配置（编译期切换），
断网积压**落 Flash 持久化、断电不丢**。

## 两种配置（`firmware.ino` 顶部 `GNSS_TIMESHARE` 一个宏切换）

| | **配置A**（`GNSS_TIMESHARE 0`，默认） | **配置B**（`GNSS_TIMESHARE 1`） |
|---|---|---|
| GPS 来源 | PORT.C **独立 GPS 模块**（ATGM336H，连续 NMEA） | PORT.A 的 **SIM7080G 二合一**内置 GNSS |
| 蜂窝 | PORT.A 的 SIM7080G **专做 4G** | 同一颗 SIM7080G **GNSS/LTE 分时**共用射频 |
| PORT.C | GPS 模块 | 腾出来接 **Unit LCD 1.14"**（状态屏） |
| 实时性 | **实时**：每个 beacon 立即直发 | 非实时：点先记 Flash，择机批量上传 |
| 上传时机 | 发包失败才落盘，**网络一恢复立刻全部补发** | 静止≥5min / 无定位≥5min / 长按 时切 LTE 整批发 |
| 取舍 | 实时、不撞分时锁（推荐） | 省一个 GPS 模块，但有 SH 分时锁（见下） |

> 两配置的差异逻辑分别在 `config_a.ino` / `config_b.ino`，未启用的一份编译成空。
> **存储、上传、撞锁自愈三层逻辑两配置完全共用**，只在"何时落盘/何时上传"按硬件特性分流。

## 硬件接线

| 部件 | 接口 | 引脚 |
|---|---|---|
| AtomS3R（ESP32-S3 + 8MB PSRAM + 16MB Flash） | — | 主控 |
| PowerHub | 内部 I2C 0x50（G45/G48） | 供电 / 电量计 / 按钮 / LED |
| GPS 模块〔仅配置A〕 | PORT.C | G2←GPS TX, G1→GPS RX（UART 115200） |
| Unit LCD 1.14"〔仅配置B〕 | PORT.C | I2C 0x3E（SDA=G2, SCL=G1） |
| SIM7080G（Unit CatM） | PORT.A | G16←模块 TX, G15→模块 RX（UART 115200） |

> 引脚 / 寄存器细节见 `firmware/defs.h`。

## 工作原理

### 存转（store-and-forward）—— LittleFS 段日志（`flashlog.ino`）

断网积压统一存到板载 Flash 的 LittleFS（挂现成的 ~1.5MB 数据分区），**断电/重启都不丢**：

- 点按**段**存：每条 20 字节（packed），满 `FL_SEG_POINTS`(20) 点封段、滚下一段；每点写完即
  `flush()` 落盘。RAM 只缓存计数（O(1)），开机扫一次盘重建。
- 上传：开**一次** SH(HTTPS) 会话，把封好的段**最旧先发、逐段 200 即删**，最后关一次会话。
- 崩溃 / 中途断网：整段保留、下次重发 → **服务端按时间戳幂等去重**，不重不漏。
- 磁盘写满（极端长断网）：删最旧段，保最新（记录仪语义）。

### SIM7080G 的 SH(HTTPS) 分时锁 + 自愈（`catm.ino`）

二合一模组（配置B）GNSS↔LTE 切换射频时，SH 应用偶发退化进
`+CME ERROR: operation not allowed` 的锁死态（与信号/PDP/注册无关）。**唯一可靠恢复是
`AT+CFUN=1,1` 整模组软重启**——已封装在 `catmSHOpen()` 里：首次 `SHCONN` 撞锁 → 自愈 → 重连，
对配置A/B 都生效（配置A 偶发、配置B ~50%）。配置A 用独立蜂窝、不切射频，稳态基本不锁。

### 上传优化

- **SH 会话复用**：`catmPostBody` 拆成 `catmSHOpen`/`catmSHReq`/`catmSHClose`，一批积压只做
  一次 TLS 握手（而非每 8 点重建一次）——大批量上传提速数倍。
- **批量**：`FL_BATCH`(20) + SHCONF `BODYLEN` 4096（实测上限），一个段一次 SHREQ 发完。
- 自适应 beacon（SmartBeaconing + 停车衰减 + 转角加点），见 `track.ino`。

## 文件结构（同目录所有 `.ino` 拼成单编译单元）

```
firmware.ino   includes / 配置开关 / 全局变量 / setup() / loop()
defs.h         引脚·寄存器·枚举·调参常量·结构体 + 全部前置声明
flushlogic.h   存转决策纯逻辑(flushDue)+调参常量（固件与 PC 仿真共用）
catm.ino       SIM7080G AT 层：cmd/init/checkNet/syncTime/SH 会话+撞锁自愈   〔共用〕
flashlog.ino   LittleFS 段日志：append/upload/磁盘满处理/计数               〔共用〕
track.ino      点格式化 + 自适应 beacon 决策                                〔共用〕
powerhub.ino   PowerHub I2C、电源/LED、电池换算                            〔共用〕
pwrlog.ino     RTC 电量日志 + GNSS 信号解析 + 串口命令台                    〔共用〕
buttons.ino    按键状态机 + 省电关机                                        〔共用〕
diag.ino       现场诊断：atScan / gnssWaitFix / gnssSwitchTest             〔共用〕
config_a.ino   #if !GNSS_TIMESHARE：独立 GPS 那套                          〔仅配置A〕
config_b.ino   #if  GNSS_TIMESHARE：二合一分时 + LCD 那套                  〔仅配置B〕
```

## 配置（首次编译前必做）

敏感信息（服务器域名 / 端口 / 路径、APN 等）抽到 `firmware/config.h`，**该文件不在仓库中**，
仅以宏名 `SERVER_HOST` / `SERVER_BASE` / `SERVER_PORT` / `PATH_APRS` / `CATM_APN` 被引用。

```bash
cp firmware/config.example.h firmware/config.h
# 编辑 firmware/config.h 填入你自己的值
```

## 依赖

- Arduino IDE 1.8.19（或 arduino-cli）+ M5Stack ESP32 板包（含 LittleFS）
- **TinyGPS++**（配置A 需要）
- **M5GFX / M5UnitLCD**（配置B 需要）

## 编译与烧录

切换配置：改 `firmware/firmware.ino` 顶部的 `#define GNSS_TIMESHARE`（0=A / 1=B）。

```bash
./build.sh        # 仅编译
./build.sh -f     # 编译并烧录（目标设备已写死，见 build.sh）
./build.sh -w     # 只把已编译的 m5power.bin 刷进去（不重编）
```

工具链路径可用环境变量覆盖：`ARDUINO=... ESPTOOL=... ./build.sh`。

## 串口命令台（USB CDC，115200）

`log` / `logclear`（电量日志） · `sendtest`（强制发一包） · `at<原文>`（AT 透传）·
`gnsstest` / `atscan`（诊断）。配置B 另有段日志台面测试命令：
`flfill <n>` / `flflush` / `flstat` / `flclear` / `flhold`。

## 辅助工具

- `pwrlog.py` — 从设备读取 / 绘制电量曲线（RTC 环形缓冲日志）
- `log.sh` — 串口监视

## 服务器侧

设备 POST 到 `PATH_APRS`，后端把点位写入数据库供地图展示。**重发会带重复点，服务端需按
时间戳幂等去重**（如 `UNIQUE(ts,lat,lon)` + `INSERT OR IGNORE`）。本仓库只含设备端固件。

## 网络要点（Cat-M / SIM7080G）

- SIM7080G 仅支持 **TLS 1.2**，服务器证书需为 **RSA**（ECDSA 会握手失败）。
- 运营商常下发 **IPv6-only PDP**，服务器若只有 IPv4 会连接失败；固件强制 IPv4-only PDP，
  失败时重附着 + 暂存补发。
- 二合一（配置B）的 SH 分时锁见上"工作原理"；配置A 用独立蜂窝可规避。

## 许可

[MIT](LICENSE)
