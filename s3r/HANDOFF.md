# S3R 交接文档（2026-08-08 更新，供新对话续做）

> 背景：配置A 提精度——AtomS3R(+PortABC Base) 插在 PowerHub 与 GPS(ATGM336H) 之间做
> GNSS+IMU 融合协处理器。路线三步：①透传 ②IMU 旁路记录 ③ESKF 融合（见 README.md）。
> **第①步已完成；PowerHub 三件（桥/心跳/看门狗）与整链联测也已完成（见下）**。

## 当前状态

- `s3r/` 固件 v0.1.1 已刷入真 AtomS3R 并运行：GPS↔链路双向透传、NMEA 旁路解析、
  128×128 屏显、UART-OTA、端口角色自动识别。全部台面验证通过。
- **2026-08-08 整链装机联测通过**（PowerHub↔S3R↔GPS 全接好、PowerHub 在 DGX 上）：
  - PowerHub 固件已补齐三件（见下方「已完成」），主仓 firmware/ 改动同样**未提交**。
  - 透传链 NMEA 正常：`[GPS] Module OK`、csFail=0；实际接线这次 GPS 在**蓝口**(gpsRX=G6)、
    链路在自带口(linkRX=G2)——与台面时相反，自动角色识别正确处理，怎么插都行坐实。
  - 真 OTA 经 PowerHub 桥全流程成功：434KB @ 8.9KB/s(UART 115200 上限)，app0→app1，
    PING 确认 pend=0；桥 60s 空闲自动退出恢复正常运行。
  - 心跳 60s 整拍收 `$S3R,PONG`，`[S3R]` 行进 PowerHub 日志。
- **git 未提交**：`s3r/` 目录 + 主仓 firmware/ 改动都无 commit（提交前照例做隐私扫描，
  s3r/ 目录应无服务器信息）。

## 已验证的关键事实（都是实测/文档坐实，不要重新怀疑）

1. **设备识别**（两台都是 ESP32-S3 USB-JTAG，靠 MAC 区分）：
   - S3R = `/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:D5:B8:64-if00`
   - PowerHub = `...1C:DB:D4:A8:27:C4...`（主仓 build.sh 写死；s3r/build.sh 写死 S3R）
2. **端口/引脚**：S3R 自带 Grove 口=G1/G2；Base 蓝口 PORT.C=G5/G6（红=G38/G39、
   黑=G7/G8 未用）。**实际接线 GPS 在自带口(G1=GPS TX)**、蓝口留 PowerHub——固件开机
   自动识别角色（持续 NMEA 流的对=GPS，另一对=链路），存 NVS，怎么插都行。
   PowerHub 侧固定：G1=TX/G2=RX（firmware/defs.h GPS_RX_PIN=2/GPS_TX_PIN=1）。
3. **GPS**：同一颗 ATGM336H-6N，已被 PowerHub 一次性配置 115200/5Hz/四系统（存模块
   flash），台面实测校验零失败。**`ANTENNA OPEN` 是正常态不是故障**：单元用内置无源
   天线，模块的天线检测电路测有源天线馈电电流（数据手册 §2.8）；只有 SHORT 算真告警。
4. **UART-OTA 已全流程通过**：434KB / 42KB/s / ~10s，app0→app1，PING 确认。协议见
   ota_flash.py 头注释。防变砖=NVS pend/att 状态机（Arduino 核无 bootloader 回滚），
   3 次启动未确认→Update.rollBack()。**坑：HWCDC USB-CDC 默认 RX 缓冲 256B 会丢 1KB
   块，必须 setRxBufferSize(4096)（已做，PowerHub 桥若走 USB 同理）**。
5. **电源**（原理图坐实）：USB VBUS 经 PPTC 直连 VIN_5V=底部 5V=Grove 5V，无二极管——
   USB 供电时 GPS/底座有电（台面可测）；**双 5V 并联无隔离**，联调时 PowerHub 先
   `phPower(PC_UART,false)`。PowerHub PORT.C 断电重启=对 S3R+GPS 整链的硬件看门狗。
6. **省电**：CPU 80MHz 下限（<80 关 PLL 断 USB，两板同坑）；IMU 上电默认 suspend。
7. Arduino 多 .ino 坑：自动原型被提到主文件 include 后 → 用到的类型（M5GFX 等）必须
   在 defs.h 里 include（见 defs.h 顶部注释）。

## 已完成（2026-08-08，原待办 1+2）

### PowerHub 固件三件（主仓 firmware/，均只在配置A 编译，`#if !GNSS_TIMESHARE`）
- **透传桥** `s3rbridge`（pwrlog.ino 命令 → config_a.ino s3rBridgeMode）：USB↔gpsSerial
  原样双向转发，60s 无 USB 数据自动退出，退出时 gpsDrainStale+看门狗基准重置。
  ota_flash.py 开口后自动先发一行 `s3rbridge` 进桥（直连 S3R 时无害），BEGIN 等待
  放宽到 35s（PowerHub 可能正阻塞在发包里）。`build.sh -o --port <PowerHub口>` 即用。
- **心跳**（config_a.ino s3rLinkTick）：每 60s 发 `$S3R,PING`；链路上的 `$S3R,*` 行由
  configLoopFeed 行装配处分流到 s3rLinkLine 打日志+记存活（tS3rLastSeen）。
- **链路看门狗**（同 s3rLinkTick）：NMEA 断流 >3min → PORT.C 断电 2s 重启整链；
  长阻塞后 gpsDrainStale 刷新"有流"时间戳防误触发；GS_NO_MODULE 时给重新检测机会。
- 配套修的坑：①PowerHub USB HWCDC RX 缓冲 256B→4096（否则桥转发丢 OTA 1KB 块，
  与 S3R 同坑）；②nmeaLine 装配缓冲 100→192（$S3R,INFO 应答 ~150 字符被超长丢弃）；
  ③控制台 `atscan` 一直被 AT 透传分支吞掉（以"at"开头），已加例外——顺手修的老 bug。
- 新控制台命令：`s3rbridge` / `s3rping` / `s3rinfo`（后两个应答异步走 `[S3R]` 日志行）。

### 整链联测结果
- 透传链 NMEA 正常（`[GPS] Module OK`、csFail=0）；链路极性/角色开机自动探测成功。
- 真 OTA 经桥：434KB @ 8.9KB/s，app0→app1，PING 确认，桥空闲自动退出。
- 心跳 PONG 60s 整拍到达；`s3rinfo` 回 INFO 全量行（pend/heap/bps/csOK 等）。
- **未演练**：看门狗断流实测（拔 S3R 上 GPS 口线，3min 后应见 PORT.C 断电重启日志
  `[S3R] link watchdog`）——纯物理操作，主人有空可顺手验一次。
- 屏显联动（可选，第③步一起也行）：PowerHub 推 CatM 状态句（如 `$S3R,STAT,...`）给
  S3R 屏显示上传状态。

## 已完成（2026-08-08 深夜追加：v0.2.0 = 第②步全部 + 第③步的融合 v1）

> 背景：主人要求出门实测前把 IMU 提精度逻辑整机装好。完整 ESKF 特意**不**在此版上线——
> 不能在零外场数据的情况下盲调（原计划②在③前正是这个原因）；v0.2.0 的环形日志
> 就是在采集它的调参原料。融合 v1 只做"证据充分才出手"的三件事，全部可台面验证。

### 融合 v1（fuse.ino，默认开，NVS 持久，`s3rfuse on/off` 可远程开关）
- **静止锁存**：IMU 判静止(2.5s 进入/0.25s 退出，姿态无关)满 3s 且有新鲜定位史 →
  坐标锁到最近 3s 定位的**分量中位数**，RMC 速度置 0。消停留漂移云。
  定位丢失中若仍静止 → 用锁点续发（GGA quality=6 / RMC mode=E 诚实标注）——
  高架下等红灯不再断档。
- **GNSS 逃生门**：锁存期间原始定位持续 8s 离锁点 >35m(HDOP<4) → 强制解锁。
  IMU 误判（电梯/平稳起步电车）永远锁不死轨迹，GNSS 说了算。
- **短断档 DR**：移动中丢定位且 3s 内有好定位 → 最后速度(3%/s 衰减)+陀螺重力轴
  航向角速率续点，≤15s/≤60m，超限即停如实丢定位。quality=6 标注。
- 改写只对校验和合法的 GGA/RMC 做；其余句、坏行、fuse off、IMU 故障 → 全部原样
  透传（透传语义兜底）。核心解析/重建在 nmea_rw.h，test/ 主机单测 37 项全过。

### IMU 管线（imu.ino）
- BMI270 @0x68（**实测这台在 0x68，M5Unified 默认 0x69 探不到**——双地址 WhoAmI
  探测已做）；In_I2C(I2C_NUM_0, SDA=G45, SCL=G0)，不调 M5.begin，屏仍归 M5GFX。
- 100Hz 采样；0.25s 统计块（|a| 标准差 + |ω| 均值，姿态无关）双阈滞回静止检测；
  静止时陀螺零偏自动校准(EMA)；重力轴投影 → 罗盘航向角速率（顺时针正）。
- 台面实测：桌面 accStd≈0.013 m/s²、gyro≈0.10dps、stat=1、零偏 6s 内自校成功。

### 环形日志（imulog.ino，第②步完成）
- LittleFS /il/，12 段×96KB 环形（≈35min）：IMU 25Hz(4 样本平均) + GNSS 5Hz +
  融合事件/零偏/UTC 对齐。深度静止(锁存>2min)自动降流量防"在家坐着挤掉移动段"。
- 拉取：`python3 imu_fetch.py`（自动进 PowerHub 桥,~9.4KB/s,CRC 校验,--clear 可清）
  → `python3 imu_decode.py <bin>` → 4 个 CSV。**全链路台面实测通过**（165KB，
  bad_sync=0，|a|均值 9.94≈g）。这是第③步 ESKF 的调参数据管线。

### 观测手段
- PowerHub 控制台新命令：`s3rimu`（IMU/静止/零偏/诊断）、`s3rfuse on|off`；
  `$PFUSE` 1Hz 诊断句在链路上流动（PowerHub 忽略，S3R 日志可查）。
- S3R 屏行3：锁存=青色 LOCK / DR=橙色 "DR bridge"。
- 修过的坑：①BMI270 地址 0x68；②TinyGPS 无定位时 date 给垃圾值且 month=0 曾致
  数组越界（TIMEMARK 写出 2046 年，已加范围校验）；③$S3R,INFO 应答 ~170 字符，
  PowerHub nmeaLine 已扩到 192（100 会被超长丢弃）。
- 已知无害现象：LittleFS 写盘瞬间非 IRAM 中断关闭 → GPS 字节偶掉，S3R 侧
  csFail ≈1%（坏行原样转发、两侧 TinyGPS 自弃，5Hz 定位流无感）。

## ★2026-08-08 外场结果：融合 v1 翻车 → v2 返工中（读这段，别再动 v1）

外场实测 v1 严重劣化轨迹（飞点+大偏移）。根因三重（定量见记忆/主对话）：
① 静止检测在真实携带场景语义反转——平稳电车 accStd 中位 0.19（85% 窗口<0.35 阈值
→ 误判静止），人真静立反而 ~1.5；振动幅度特征对"地理静止"不可分，调参救不了。
② 逃生门 8s 等待 × 车速 = 每次误锁白送 160m 漂移（HDOP 门不背锅）。
③ 逃生门解锁后同 tick 立即重锁（无冷却）→ ESC/ON 同毫秒死循环，轨迹成 8s 阶梯。
电量：97mA / 23h 续航（比无 S3R 多 ~17mA）。

**v2 设计原则**（教训直接写进结构）：GNSS 在场永远权威；无模式（锁存/DR/逃生门
全部删除，换成单个 2D 匀速 KF 的连续协方差机制）；诚实输出（推算点 σ 超界停发）；
一条链路只能有一个估计器（上机时 PowerHub 检测到 $PFUSE 心跳要旁路自家野点门+KF）。

### P1 已完成（回放框架，纯主机）：
- `nav_core.h` —— 纯 C 估计器核心（野点门+NIS two-strike 沿袭 track.ino 语义；
  停留信念连续量→过程噪声 log 内插+ZUPT 伪量测；断档 coast=速度衰减+陀螺航向转动，
  σ>40m 停发；"被携带"IMU 高活动量守卫；GNSS 创新持续超界→信念清零的权威释放通道）。
- `test/replay.cpp` —— 主机回放器（--base = PowerHub 管线忠实复刻作基线）；
  `test/metrics.py` —— A~E 指标评分 + 轨迹对比图（验收用同一把尺子）。
- 0808 电车段回放：v1 失败签名 0；良好段偏差与基线持平（中位 10.1 vs 9.7m）；
  停留散布 4.9 vs 5.5m 反超；桥接 743 拍（基线 0）最长 51s；地图可见传送 1:1（原始
  鬼影段，两版同承受）。桥接终点修正 max 113m 是 P2 调参对象（coastDecay/σmax）。

## ★2026-08-16 首份外场数据：P2 验证点 1~3 全过 + 调参定版（读这段再动参数）

当天：纯步行 6.7h（14:47–21:29 JST），全速率层覆盖最后 ~62min（回家路+到家），
摘要层 16h（含 0808 夜在家 11h 段；8/9–8/15 设备未开机）。电量 102mA/21.5h 与上次一致。

### 验证点结果
1. ✔ `feat=5054(device)` 生效。可分性（设备特征 accStdMax）：步行 p50=3.36/p10=1.13，
   在家静置 p50=0.011——1.0 阈值两态干净可分；**电车巡航侧仍待下次坐车数据**。
2. ✔ 摘要层完整：今天段 21618 行 6.74h 无 >30s 空洞；TIMEMARK 对齐 UTC（漂移 <1.5s）；
   真损坏行仅 0.01% 量级（bad_sync 残渣，loader 通用判据已滤）。
3. ✔ 1Hz 全天回放通（把"今天段"从多开机段 CSV 里切出来喂——loader 的 ms 单调滤波
   会把混段数据吃掉，见 scratch 做法：从尾部找最后一个 ms 回退点切片）。
   1Hz 下 KF 行为与 5Hz 一致（A p90 稍松 20.5 vs 16.5，无 runaway）。

### 0816 失效解剖（进楼过渡，本次调参的靶子）
21:17–21:19 JST 进楼：sats 14→3、hdop 0.8→12.8;**ATGM336H 在 sats≤5 时输出坐标
冻结的保持解（step=0、crs 恒定，却标 valid=1）**，随后 hdop 6 的 30m 级多径鬼影步进
把 KF 速度打到 4m/s 且位置拖偏 ~100m（NIS two-strike 放行，基线同样中招），断档后
coast 以幻影速度+陀螺转向再补 ~90m → 191.6m 传送。逐桥统计证实：**坏桥（0816 191.6m、
0808 113.2m）断档前 hdop 均 ≥6；好桥（电车 28/37s、停留保持）均 ≤1.7**。

### 调参定版（nav_core.h navParamsDefault 已写入；replay 新增 --set 名=值 扫参口）
- **coastHdopMax=2.5（新机制）**：断档前 hdop EMA(τ8s) 超此 → 本段 coast 整段禁发
  （外推资格门）；停留保持(still≥zuptOn，v=0 钉住)不是外推，豁免。
- **accStdCoastCarried=1.0（新机制）**：被携带(步行)中 coast 用更大过程噪声，σ 加速
  越诚实界（人转身/停步不可预测；电车滑行 IMU 安静不受影响）。
- coastHdopRef（初速折减）实现了但**默认关**：门限已覆盖病灶，折减会给好桥引入低速偏差。
- **accStdMove 0.3→0.5**：0808 电车段 A med/p90 = 6.5/15.5 **反超基线** 9.7/19.9；
  B 双数据集改善（0816 停留散布 3.9 vs 基线 7.1）；代价=原始鬼影跳变不再被过度平滑
  吞掉（C 与基线 1:1 同承受）。注意 0.5+posSigma=3 组合会出 E=1，别动 posSigmaM。
- 定版指标（vs 基线）：0816 C传送 0/0、E 0/0、B 3.9/7.1；0808 A 6.5/9.7、
  D2max 19.9m（原 113）、C 1/1、E 0/0、电车桥保留（最长 26s）。
- metrics.py E 指标加了 hdop<2 质量门（室内多径拍 spd 虚报>3 不是 v1 签名，
  基线同样中招证明是 raw 现象——0816 店内 260 拍全是这类）。
- 已知残留：到家后室内鬼影 fix 被接受导致的红线游走与基线等价（非退化）；
  P3 上机后 PowerHub 消费 $PFUSE still 标志可收敛，暂不动。

### 其它当天动作
- PowerHub `gnssDiagLine()` 加 NMEA 校验和门（pwrlog.ino，坏行曾把方位角当 CN0 记进
  电量日志出现 cn0=213/sats=111），**已刷机**，启动验证正常。
- imu_fetch.py --clear 改为明确等 CLEARED+3 次重发（桥上单次 5s 应答易丢）；
  当天环形已确认清空。
- 双数据集扫参脚本样例在会话 scratchpad（scan.py），复用时照抄思路即可。

## 待办（按序）

### P2 数据积累+离线调参 —— 设备侧已完成（0808 深夜），剩外场采数+调参：
  ✔ 重分区：`partitions_s3r.csv`（app 1.5MB×2 + spiffs 4.875MB），build.sh -f 时经
    gen_esp32part.py 写入——编译仍用 default_8MB（只管构建期尺寸检查），**片上表才是
    真相**；已刷入，free=4944KB 实证。NVS 未动（fuse=off/端口角色保留）。
  ✔ 双层环形（imulog.ino 重构成 IlRing×2，struct 在 defs.h——自动原型坑）：
    全速率 /il/ 18 段(~55min) + 摘要 /ils/ 26 段(~19h)=1Hz SGNS + 1Hz FEAT（设备
    自算秒级特征：accStd mean/max、gyro、headRate、stat——消除回放复算窗口尺度差）。
  ✔ fuse 默认改 OFF（NVS 全擦后也安全）；imu_decode.py 出 _feat/_sgnss；replay.cpp
    优先吃 _feat.csv（feat=device），无 _gnss.csv 时退 _sgnss.csv 做全天回放。
  ✔ 端到端实测：直连拉取→解码→feat 回放全通；双环形清零，等外场数据。
  ☐ 外场正常携带数天（纯透传），每晚 `imu_fetch.py --clear` 拉日志。
  ✔ 首份新数据验证点 1~3 全过（0816，见上方新段）。
  ✔ 调参第一轮定版（0816 步行段 + 0808 电车段互验，见上方新段）：
    D2max 113→19.9；A p90 21.6→15.5 反超基线；B 双改善；E=0。
  ☐ **剩余验证（下批数据）**：
    - 电车巡航侧 accStdActive=1.0 可分性（本批无乘车段，用设备 _feat 补画分布）；
    - 定版参数在新一天数据上的 E 必须仍=0（0.5+posSigma=3 曾出 E=1，参数在边缘）；
    - 红绿灯/店内真实停留段看 stillEnterSec/zuptSigma（本批停留全在信号差区，没调）。
### P3 上机（调参在多天数据上稳定后）：
  - fuse.ino 删三开关（锁存/DR/逃生门整体删除，不保留），接 nav_core：GGA 驱动
    navGnss/navCoast，RMC 复用同拍输出改写；navImu 按 0.25s 块直喂（imu.ino）。
  - PowerHub：收到 $PFUSE 心跳（1Hz，nav 版换新字段：mode/σ/still）→ 旁路自家
    野点门+KF 直接消费；心跳消失 >N 秒自动回落现有管线（透传/摘除 S3R 兜底）。
  - PowerHub 加 `pcoff`/`pcon` 控制台命令（phPower(PC_UART) 手控）——以后 S3R 接
    USB 前先软件断 PORT.C，避免双 5V 并联（本次是拔 PowerHub USB 换线，撞过一次）。
  - 上传报文加融合诊断遥测字段（mode/σ/still/NIS 拒点计数/CN0，几十字节搭现有
    会话，服务端 iotService 配合）——解决"外场无法实时观测"；验收统计也靠它。
  - 上机后台面验证：室内无定位→coast 停发行为、假点注入（gnsstest 类似手段）、
    fuse on/off 切换透传语义不破。
### P4 外场 A/B 验收：同路线 s3rfuse on/off；标准=停留成点、断档≤15s 桥接、良好段
  无偏移、飞点为零、全段类不输基线 → 定最终版；调不出净收益则 S3R 永久退守
  透传+看门狗+记录仪。
### （原第③步"完整 ESKF"已被 v2 取代——包内姿态未知，加计位置积分是伪能力，
  砍掉；IMU 角色=陀螺航向+活动量证据+调参数据。）

## 速查

```bash
cd s3r && ./build.sh          # 编译
./build.sh -f                 # USB 完整刷写（救砖/重分区——写 partitions_s3r.csv）
# v2 回放调参（P1 框架）：
cd test && g++ -O2 -o replay replay.cpp
./replay ../imulog_<日期> > nav.csv            # v2；--base = PowerHub 基线复刻
python3 metrics.py nav.csv base.csv --plot cmp.png   # A~E 指标 + 轨迹对比图
./build.sh -o --port /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_1C:DB:D4:A8:27:C4-if00
                              # OTA 经 PowerHub 桥（装机常态；台面省略 --port 走 S3R USB）
python3 imu_fetch.py --clear  # 拉 IMU/GNSS 日志（经桥）并清空
python3 imu_decode.py imulog_*.bin   # → _imu/_gnss/_fuse/_events 四个 CSV
# S3R USB 控制台：help/info/ping/imu/fuse [on|off]/imulog/imuclear/screen/linkswap/pinscan/reboot
# PowerHub 控制台：s3rbridge / s3rping / s3rinfo / s3rimu / s3rfuse on|off
```

屏幕：短按亮 1 分钟，亮屏中长按 1s 熄。行4 左侧=海拔(定位)/NMEA流速率(无定位)/
红 PEND(OTA 待确认)。版本号不上屏（info/PONG 里看）。
