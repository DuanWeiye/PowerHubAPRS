# M5Power APRS Tracker

基于 **M5Stack AtomS3R + PowerHub + GPS Unit v1.1 + Unit CatM (SIM7080G)** 的
低功耗 GPS 轨迹追踪器。通过 Cat-M (LTE-M) 蜂窝网络，把定位点以 HTTPS POST 上报到自建服务器。

## 硬件

| 部件 | 接口 | 说明 |
|---|---|---|
| AtomS3R | — | 主控（ESP32-S3 + OPI PSRAM） |
| PowerHub | — | 供电 / 电量计 / 按钮 / LED |
| GPS Unit v1.1 | PORT.C | G1→GPS RX, G2←GPS TX |
| Unit CatM (SIM7080G) | PORT.A | G15→模块 RX, G16←模块 TX |

> 引脚与寄存器细节见 `firmware/firmware.ino` 顶部注释。

## 配置（首次编译前必做）

敏感信息（服务器域名、APN 等）抽到了 `firmware/config.h`，**该文件不在仓库中**。
克隆后复制模板并填入自己的值：

```bash
cp firmware/config.example.h firmware/config.h
# 编辑 firmware/config.h，填入你的 APN / 服务器域名 / 端口 / 路径
```

## 依赖

- Arduino IDE 1.8.19（或 arduino-cli）+ M5Stack ESP32 板包
- **TinyGPS++**（库管理器安装，或自行放入工程）

## 编译与烧录

```bash
./build.sh                    # 仅编译
./build.sh -f /dev/ttyACM0    # 编译并烧录
```

工具链路径可用环境变量覆盖：`ARDUINO=... ESPTOOL=... ./build.sh`。

## 辅助工具

- `pwrlog.py` — 从设备读取/绘制电量曲线（RTC 环形缓冲日志）
- `log.sh` — 串口监视

## 服务器侧

设备 POST 到 `PATH_APRS`（默认 `/iot/aprs`），后端把点位写入数据库供地图展示。
本仓库只含设备端固件，服务器端实现不在此处。

## 许可

[MIT](LICENSE)

## 网络要点（Cat-M / SIM7080G）

- SIM7080G 仅支持 **TLS 1.2**，服务器证书需为 **RSA**（ECDSA 会握手失败）。
- 运营商常下发 **IPv6-only PDP**，而服务器若只有 IPv4（无 AAAA）会导致连接失败；
  固件会强制 IPv4-only PDP 并在失败时重附着、暂存补发。详见固件内注释。
