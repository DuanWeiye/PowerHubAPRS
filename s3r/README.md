# S3R — AtomS3R GNSS 协处理器

配置A 提精度硬件链的中间节点：把 AtomS3R（+Atomic PortABC Base）插在 PowerHub 和
GPS(ATGM336H) 之间，最终目标是 GNSS+IMU(BMI270) 融合后以标准 NMEA 回喂 PowerHub
（PowerHub 固件零改动就吃到更准的"GPS"）。

```
PowerHub PORT.C(蓝, G1/G2) ←Grove→ AtomS3R 自带口(G1/G2)
                                    AtomS3R ⊕ ABC Base
                                    Base 蓝口 PORT.C(G5/G6) ←Grove→ GPS ATGM336H
供电：PowerHub PORT.C 5V 带整链；phPower(PC_UART) 断电重启 = 下游硬件看门狗
```

## 路线图

| 步 | 内容 | 状态 |
|---|---|---|
| ① | 透传：GPS↔PowerHub 双向转发（行为与直连严格一致）+ 屏显 + UART-OTA | v0.1.1 完成 |
| ② | IMU 旁路记录：BMI270+NMEA 记 LittleFS 环形日志，`imu_fetch.py`/`imu_decode.py` 拉取解码，外场数据离线调融合参数 | **v0.2.0 完成** |
| ③ | 融合上线：输出融合 NMEA + `$PFUSE`；PowerHub 旁路自家 KF 直接消费 + 上传遥测 | **v0.3.0 完成（2026-08-22，nav_core v2）**；v1 三开关已删；完整 ESKF 已砍（包内姿态未知，加计积分是伪能力） |

融合 v2（默认开，`fuse off`/PowerHub `s3rfuse off` 可关，NVS 持久）：唯一估计器
`nav_core.h`（2D 匀速 KF + 连续停留信念/ZUPT + 诚实 coast，参数经 0808/0816/0822 三份
外场数据回放定版）。每拍 GGA 驱动估计器（RMC 先暂存、同拍一起放），IMU 提供 1s 滑动
accStd（"被携带"证据）与陀螺航向角速率（coast 转向）。输出：跟踪点换坐标/速度/航向；
推算点 GGA quality=6 / RMC 模式 E，σ>40m 即停发；被野点门拒的拍标 q=0/V；其余原样透传。
`$PFUSE,2,…` 1Hz 心跳让 PowerHub 旁路自家野点门/KF（单估计器原则），上传报文带
nav/sig/still/cn0/rej 遥测。台面验证：`test/inject_test.py`（经 PowerHub `gpsin` 注入合成
轨迹，不出门跑通静止/步行/断档/恢复）。

## 构建 / 刷写

```bash
./build.sh          # 只编译
./build.sh -f       # 编译 + USB 完整刷写（首刷/救砖；目标写死真 AtomS3R 的 by-id）
./build.sh -o       # 编译 + UART-OTA（日常更新，见下）
```

⚠️ 主仓 `build.sh` 写死的是 PowerHub（MAC `1C:DB:D4:A8:27:C4`），本目录脚本写死
S3R（MAC `14:C1:9F:D5:B8:64`）——两台都是 ESP32-S3 USB-JTAG，别混用。

### UART-OTA（装机后不拆机更新）

esptool 无法走 Grove（ROM 下载模式只在未引出的 UART0/USB），所以固件自带 OTA
接收器（`$S3R,OTA` 协议，`ota_flash.py` 配对）：

- 台面直测：`./build.sh -o`（走 S3R 自身 USB）
- 装机之后：`./build.sh -o --port <PowerHub的by-id>`——ota_flash.py 自动发
  `s3rbridge` 把 PowerHub 带进透传桥（已实现并实测），DGX 只插 PowerHub 即可更新 S3R

防变砖：default_8MB 双 app 槽；新固件启动后必须收到 PING 才算确认（脚本刷完自动
PING），未确认重启 3 次自动回滚旧槽。链路僵死时 PowerHub 断 PORT.C 电即触发重启。
兜底：拆下来 `./build.sh -f` 走 USB。

## 串口命令（USB 控制台 115200）

`help` / `info`（状态一览）/ `ping` / `imu`（IMU/静止/零偏）/ `fuse [on|off]`
（融合开关/状态）/ `imulog`（日志量）/ `imuclear` / `screen` / `linkswap`（链路
极性翻转兜底）/ `pinscan` / `reboot`

装机后经 PowerHub 控制台远程：`s3rinfo` / `s3rimu` / `s3rfuse on|off` / `s3rping`
（应答走 `[S3R]` 日志行）；日志拉取 `python3 imu_fetch.py [--clear]`。

## 端口 / 屏幕 / 省电

- **端口角色自动识别**：可用两口 = S3R 自带 Grove 口(G1/G2) 和 Base 蓝口(G5/G6)，
  一口 GPS 一口 PowerHub，**怎么插都行**——开机扫描：持续吐 NMEA 的对 = GPS（顺带
  定出 RX/TX），另一对归链路（RX 靠 PowerHub TX 空闲恒高判别）。结果存 NVS。
  2026-08-07 实测 GPS 插在自带口、蓝口留给 PowerHub，工作正常。Base 的红口
  (G38/G39)/黑口(G7/G8) 不在扫描范围，留给以后扩展。
- **链路极性兜底**：PowerHub 未上电时判不出链路 RX/TX，接上后若 PCAS/OTA 不通，
  USB 控制台敲 `linkswap` 一次即可（即时生效并记忆）。`pinscan` 可扫全部脚找信号。
- **屏幕** 128×128：定位状态+星数 / HDOP+最强CN0 / 速度(或各星座可见星) / 海拔或
  NMEA流速率+运行时长。开机亮 1 分钟自动息屏；短按屏幕唤醒，亮屏中长按 1s 息屏。
  屋内无 GPS 信号，台面上 3 分钟后显示 ANT? 属正常。天线告警只认 ANTENNA SHORT
  （真短路）；本 GPS 单元用内置无源天线，模块的有源天线馈电检测恒报 ANTENNA
  OPEN——**正常态，不是故障**（数据手册 §2.8）。
- **省电**：CPU 80MHz（USB 存活下限，再低断 USB——同 PowerHub 的坑）、WiFi/BT 不
  初始化、屏幕默认息屏。IMU 自 v0.2.0 起 100Hz 常开（BMI270 全功耗 ~1mA 级，
  相对整链 ~150mA 可忽略）。

## 双 5V 注意

S3R 接 DGX USB 的同时若 PowerHub Grove 也供电，两路 5V 并联无隔离。联调时让
PowerHub 先 `phPower(PC_UART, false)` 关掉那路（信号线共地照常通信）。
