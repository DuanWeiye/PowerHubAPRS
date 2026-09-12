// imu.ino — BMI270 采样 + 姿态无关的运动状态估计
//
// 设计约束：设备装在口袋/包里，安装朝向完全未知且非刚性 → 一切算法只用
// 姿态无关量：|a| 的块内标准差、|ω| 模长、重力方向（低通加速度）投影。
//   - 静止检测：双阈滞回，进入慢(2.5s)退出快(0.25s)（v1 锁存判据遗留；v2 只做零偏校准窗）
//   - 陀螺零偏：静止时自动校准（EMA），无需任何手工步骤
//   - 航向角速率：ω 在重力轴上的投影 → 罗盘航向变化率（顺时针为正），供 coast 转动速度矢量
// 硬件：M5Unified BMI270_Class + In_I2C（内部 I2C：SDA=G45 SCL=G0，addr 0x69）。
// 不调 M5.begin()——屏幕仍归 screen.ino 的 M5GFX 管，互不相扰。
#include "defs.h"

static m5::BMI270_Class* imuDev = nullptr;   // 地址探测(0x69/0x68)后 new
static bool     imuReady    = false;
// 远程诊断（装机后 S3R USB 不可达，经 $S3R,IMU 回读）
static bool     imuI2cOk    = false;
static uint8_t  imuWho69    = 0, imuWho68 = 0;

// ── 每样本状态 ──
static float    gravLp[3]   = {0};   // 重力向量 LPF（设备系，m/s²）
static bool     gravInit    = false;
static float    gyroBias[3] = {0};   // 陀螺零偏（dps）
static bool     biasSet     = false;
static float    headRate    = 0;     // 罗盘航向角速率（dps，顺时针正），EMA 平滑

// ── 统计块（0.25s = 25 样本）──
static float    blkSumMag   = 0, blkSumMagSq = 0, blkSumGyr = 0;
static int      blkN        = 0;
static float    lastAccStd  = 0, lastGyroMag = 0;

// ── 静止检测 ──
static int      quietBlocks = 0;
static bool     stationary  = false;
static uint32_t tStatSince  = 0;

// ── 零偏校准窗（静止时累计原始 gyro 均值）──
static float    biasSum[3]  = {0};
static int      biasBlkCnt  = 0;     // 已累计的完整块数
static int      biasSampCnt = 0;

// ── 落盘抽取（4 样本平均 → 25Hz）──
static int32_t  decSumA[3]  = {0}, decSumG[3] = {0};
static int      decN        = 0;

static bool imuInit() {
    // In_I2C 尚未 begin（我们不走 M5.begin）——显式起内部 I2C 总线（AtomS3R: SDA=G45 SCL=G0）
    imuI2cOk = m5::In_I2C.begin(I2C_NUM_0, 45, 0);
    // WhoAmI 双地址探测（BMI270=0x24；AtomS3R 常规 0x69，兜底 0x68）
    imuWho69 = m5::In_I2C.readRegister8(0x69, 0x00, 400000);
    imuWho68 = m5::In_I2C.readRegister8(0x68, 0x00, 400000);
    uint8_t addr = (imuWho69 == 0x24) ? 0x69 : (imuWho68 == 0x24) ? 0x68 : 0;
    if (!addr) {
        Serial.printf("[IMU] BMI270 not found (i2c=%d who69=0x%02X who68=0x%02X)\n",
                      imuI2cOk, imuWho69, imuWho68);
        return false;
    }
    imuDev = new m5::BMI270_Class(addr);
    m5::IMU_Base::imu_spec_t spec = m5::IMU_Base::imu_spec_none;
    for (int attempt = 0; attempt < 2; attempt++) {      // 配置文件上传偶发失败 → 重试一次
        spec = imuDev->begin();
        if ((spec & m5::IMU_Base::imu_spec_accel) && (spec & m5::IMU_Base::imu_spec_gyro)) break;
    }
    if (!(spec & m5::IMU_Base::imu_spec_accel) || !(spec & m5::IMU_Base::imu_spec_gyro)) {
        Serial.printf("[IMU] BMI270@0x%02X begin FAIL (spec=%d)\n", addr, (int)spec);
        return false;
    }
    // begin() 只在检出 BMM150 的分支里开 PWR_CTRL——兜底确保 acc/gyr/temp 使能
    uint8_t pwr = imuDev->readRegister8(0x7D);
    if ((pwr & 0x0E) != 0x0E) imuDev->writeRegister8(0x7D, pwr | 0x0E);
    imuDev->writeRegister8(0x40, 0xA8);   // ACC_CONF: ODR 100Hz, normal, perf 滤波
    imuDev->writeRegister8(0x42, 0xA8);   // GYR_CONF: ODR 100Hz, normal, perf 滤波
    imuReady = true;
    Serial.printf("[IMU] BMI270@0x%02X OK (spec=%d)\n", addr, (int)spec);
    return true;
}

static bool imuOk()              { return imuReady; }
static bool imuIsStationary()    { return stationary; }
static uint32_t imuStationarySince() { return tStatSince; }
static bool imuBiasKnown()       { return biasSet; }
static float imuHeadingRateDps() { return biasSet ? headRate : 0.0f; }
static float imuAccStd()         { return lastAccStd; }
static float imuGyroMag()        { return lastGyroMag; }

// ── 秒级特征聚合（4 块 → 1 条 ILOG_FEAT，摘要层全天记录，P2 调参主粮）──
static float    featSumStd  = 0, featMaxStd = 0, featSumGyr = 0;
static int      featBlks    = 0;

// 一个统计块结束：更新静止检测 + 零偏校准 + 秒级特征落盘
static void imuBlockDone(float accStd, float gyroMag, uint32_t now) {
    lastAccStd  = accStd;
    lastGyroMag = gyroMag;
    fuseImuBlock(accStd, headRate, now);   // 估计器：1s 滑动最大 accStd + 航向角速率

    featSumStd += accStd;
    if (accStd > featMaxStd) featMaxStd = accStd;
    featSumGyr += gyroMag;
    if (++featBlks >= 4) {
        imulogFeat(now, featSumStd / featBlks, featMaxStd, featSumGyr / featBlks,
                   headRate, stationary);
        featSumStd = featMaxStd = featSumGyr = 0;
        featBlks = 0;
    }

    bool quiet = (accStd < IMU_STAT_ACC_ENTER && gyroMag < IMU_STAT_GYR_ENTER);
    bool loud  = (accStd > IMU_STAT_ACC_EXIT  || gyroMag > IMU_STAT_GYR_EXIT);

    if (stationary) {
        if (loud) {                        // 快退出：0.25s 内响应起步
            stationary  = false;
            quietBlocks = 0;
            biasBlkCnt = biasSampCnt = 0;
            biasSum[0] = biasSum[1] = biasSum[2] = 0;
        }
    } else {
        if (quiet) {
            if (++quietBlocks >= IMU_STAT_ENTER_BLK) {   // 慢进入：连续 2.5s 安静
                stationary = true;
                tStatSince = now;
            }
        } else {
            quietBlocks = 0;               // 中间带/吵闹都重置进入计数
        }
    }

    // 零偏校准：静止时以 2s 窗均值更新（首次直接置入，之后 EMA）
    if (stationary) {
        if (++biasBlkCnt >= IMU_BIAS_BLKS && biasSampCnt > 0) {
            float m[3] = { biasSum[0] / biasSampCnt,
                           biasSum[1] / biasSampCnt,
                           biasSum[2] / biasSampCnt };
            if (!biasSet) {
                gyroBias[0] = m[0]; gyroBias[1] = m[1]; gyroBias[2] = m[2];
                biasSet = true;
                Serial.printf("[IMU] gyro bias set: %.2f %.2f %.2f dps\n", m[0], m[1], m[2]);
                imulogEvent(EV_BIAS_SET, 0);
            } else {
                for (int i = 0; i < 3; i++)
                    gyroBias[i] += IMU_BIAS_EMA * (m[i] - gyroBias[i]);
            }
            // 只在零偏实际动了(>0.02dps)才落盘，别让静置刷屏
            static float loggedBias[3] = {1e9f, 1e9f, 1e9f};
            if (fabsf(gyroBias[0]-loggedBias[0]) > 0.02f ||
                fabsf(gyroBias[1]-loggedBias[1]) > 0.02f ||
                fabsf(gyroBias[2]-loggedBias[2]) > 0.02f) {
                memcpy(loggedBias, gyroBias, sizeof(loggedBias));
                imulogBias(gyroBias);
            }
            biasBlkCnt = biasSampCnt = 0;
            biasSum[0] = biasSum[1] = biasSum[2] = 0;
        }
    }
}

static void imuTick(uint32_t now) {
    if (!imuReady) return;
    m5::IMU_Base::imu_raw_data_t raw;
    auto got = imuDev->getImuRawData(&raw);          // 数据未就绪时返回 none
    if (!(got & m5::IMU_Base::imu_spec_accel)) return;

    float a[3] = { raw.accel.x * IMU_ACC_RES, raw.accel.y * IMU_ACC_RES,
                   raw.accel.z * IMU_ACC_RES };
    float g[3] = { raw.gyro.x * IMU_GYR_RES, raw.gyro.y * IMU_GYR_RES,
                   raw.gyro.z * IMU_GYR_RES };

    // 重力向量 LPF（τ≈2s；步行中的摆动被平均掉，留缓变的重力方向）
    if (!gravInit) { gravInit = true; for (int i = 0; i < 3; i++) gravLp[i] = a[i]; }
    else for (int i = 0; i < 3; i++) gravLp[i] += IMU_GRAV_ALPHA * (a[i] - gravLp[i]);

    // 航向角速率：ω-bias 在重力"上轴"(â=归一化 gravLp，静止时指向天)上的投影取负
    // = 绕"下轴"角速率 = 罗盘航向变化率（顺时针为正）。台面验证：平放屏朝上、
    // 俯视顺时针旋转 → headRate > 0。
    float gn = sqrtf(gravLp[0]*gravLp[0] + gravLp[1]*gravLp[1] + gravLp[2]*gravLp[2]);
    if (biasSet && gn > 1.0f) {
        float hr = -((g[0]-gyroBias[0]) * gravLp[0] +
                     (g[1]-gyroBias[1]) * gravLp[1] +
                     (g[2]-gyroBias[2]) * gravLp[2]) / gn;
        headRate += 0.2f * (hr - headRate);
    }

    // 统计块累计（姿态无关量）
    float amag = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    float gmag = sqrtf((g[0]-gyroBias[0])*(g[0]-gyroBias[0]) +
                       (g[1]-gyroBias[1])*(g[1]-gyroBias[1]) +
                       (g[2]-gyroBias[2])*(g[2]-gyroBias[2]));
    blkSumMag += amag; blkSumMagSq += amag * amag; blkSumGyr += gmag;
    if (stationary) {          // 零偏窗只在静止时累计（原始值，含当前零偏）
        biasSum[0] += g[0]; biasSum[1] += g[1]; biasSum[2] += g[2];
        biasSampCnt++;
    }
    if (++blkN >= IMU_BLOCK_N) {
        float mean = blkSumMag / blkN;
        float var  = blkSumMagSq / blkN - mean * mean;
        imuBlockDone(sqrtf(var > 0 ? var : 0), blkSumGyr / blkN, now);
        blkSumMag = blkSumMagSq = blkSumGyr = 0;
        blkN = 0;
    }

    // 落盘抽取：4 样本平均 → 25Hz（简易抗混叠）
    decSumA[0] += raw.accel.x; decSumA[1] += raw.accel.y; decSumA[2] += raw.accel.z;
    decSumG[0] += raw.gyro.x;  decSumG[1] += raw.gyro.y;  decSumG[2] += raw.gyro.z;
    if (++decN >= IMU_LOG_DECIM) {
        int16_t la[3] = { (int16_t)(decSumA[0]/decN), (int16_t)(decSumA[1]/decN),
                          (int16_t)(decSumA[2]/decN) };
        int16_t lg[3] = { (int16_t)(decSumG[0]/decN), (int16_t)(decSumG[1]/decN),
                          (int16_t)(decSumG[2]/decN) };
        imulogImuRaw(now, la, lg);
        decSumA[0]=decSumA[1]=decSumA[2]=0;
        decSumG[0]=decSumG[1]=decSumG[2]=0;
        decN = 0;
    }
}

// console `imu`：一屏当前状态
static void imuPrintStatus() {
    Serial.printf("[IMU] %s  stationary=%d(%.0fs)  accStd=%.3f m/s2  gyro=%.2f dps\n",
                  imuReady ? "OK" : "FAIL", stationary,
                  stationary ? (millis() - tStatSince) / 1000.0f : 0.0f,
                  lastAccStd, lastGyroMag);
    Serial.printf("[IMU] bias %s: %.2f %.2f %.2f dps   headRate=%.2f dps/s(CW+)\n",
                  biasSet ? "set" : "unset", gyroBias[0], gyroBias[1], gyroBias[2], headRate);
    Serial.printf("[IMU] grav: %.2f %.2f %.2f (|g|=%.2f m/s2)\n",
                  gravLp[0], gravLp[1], gravLp[2],
                  sqrtf(gravLp[0]*gravLp[0]+gravLp[1]*gravLp[1]+gravLp[2]*gravLp[2]));
}
