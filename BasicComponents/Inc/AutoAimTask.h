/**
  ******************************************************************************
  * @FileName			    AutoAimTask.cpp
  * @Description            Commmunicate with MiniPC and control the gimbal
  *                         and shoot automatically
  * @author                 Steve Young & Huang Shuhang
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/
#ifndef __AUTOAIMTASK_H__
#define __AUTOAIMTASK_H__

#include "includes.h"
#define AIM_INTERPOLATION_CNT (30)


typedef struct GMAngle_t {
    float yaw;
    float pitch;
    float yaw_v;
    float pitch_v;
} GMAngle_t;

typedef struct MCURecvData_t {
    uint8_t start;
    float yaw;                // 单位 °
    float pitch;
    float yaw_v;              // 单位 °/s
    float pitch_v;            // 角度环得到值 + yaw_v
    uint16_t period;             // 陀螺周期
    uint16_t period_offset;
    float predict_dist_3d;
    uint8_t shoot_flag  : 1;  // 射击标志位
    uint8_t engegy_flag : 7;  // 能量机关标志位
    uint8_t check;
    uint8_t end;
} __packed MCURecvData_t;
// shoot_flag : 射击标志位   0:瞄 1:开炮


// enemy_color  : 0 - BLUE
//              : 1 - RED
// program_mode : 0 - autoaim , 1 - energy 能量机关
// energy_mode :  0 - HIT_SMALL 击打小能量机关
//                1 - HIT_BIG   击打大能量机关
//                2 - DISTURB_SMALL 扰乱敌方小能量机关
//                3 - DISTURB_BIG   扰乱敌方大能量机关
typedef struct MCUSendData_t {
    uint8_t start;
    float bulletSpeed;
    uint8_t enemyColor: 1;
    uint8_t program_mode: 1;  // 普通自瞄 program_mode:0 && energy_mode:0; 陀螺: program_mode:0 && energy_mode:5
    uint8_t energy_mode:  6;
    uint8_t crc;
    uint8_t end;
} __packed MCUSendData_t;


enum AutoAimMode_e {     // 自瞄模式 4档
    manual = 0,          // 手动
    predict,             // 预测 简单自瞄模式
    antiTop,             // 反陀螺模式
    energy               // 能量机关模式
};

enum EnergyModeSend_e {  // 击打能量机关的具体模式
    HIT_SMALL  = 0,      // 0 击打小能量机关
    HIT_BIG,             // 1 击打大能量机关
    DISTURB_SMALL,       // 2 扰乱敌方小能量机关
    DISTURB_BIG          // 3 扰乱敌方大能量机关
};

enum EnergyModeRecv_e { // 能量机关模式下 视觉发送回的flag 对应 energy_shoot
    ENERGY_FOLLOW = 0,   // 能量机关跟随模式
    ENERGY_SHOOT         // 开炮
};
// 从AGX得到的数据
// buffer[0]              : 's'
// buffer[1]  - buffer[4]  : float1 -- data.yaw
// buffer[5]  - buffer[8]  : float2 -- data.pitch
// buffer[9]  - buffer[12] : float3 -- data.yaw_v
// buffer[13] - buffer[16] : float4 -- data.pitch_v
// buffer[17] - buffer[20] : float5 -- data.period  陀螺周期
// buffer[21]              : data.flag 底盘锁定与云台锁定的判断依据
// buffer[22]              : 校验位 为 buffer[1] 至 buffer[17] 之和
// buffer[23]              : 'e'
// length ： 1 + 4 + 4 + 4 + 4 + 1 + 1 + 1

class AutoAimRecv {
private:
    static constexpr int BUFFER_SIZE = 200;
    uint8_t AutoAimBuffer[BUFFER_SIZE];
protected:
    float      lastYaw;
    bool       recvFinish;
    uint8_t    Check;
    uint32_t   sendCnt;
    uint32_t   recvivedCnt;
    MCURecvData_t data;
public:
    GMAngle_t  output{};
    void RecvStart();
    bool autoShoot;
    bool autoControl;

    AutoAimRecv();

    inline void ParaInit();

    void UartRxCpltCallback();

    MCURecvData_t getData() { return data;}

};


class AutoAim : public AutoAimRecv {
private:
    bool gimbalFreeze;
    bool chassisFreeze;
    MCUSendData_t sendData{};
    float worldYaw;

    float PitchRealAngle[80];
    float YawRealAngle[80];
    uint8_t posPtr;

    uint16_t antitop_period;
    uint16_t antitop_period_offset;

    void AutoAimParaInit();

    void RestoreRealAngle();

    void DataProcess();

    void Control();

    void UartTxInfo();

public:
    bool speedCalc_pitch;
    bool speedCalc_yaw;
    AutoAimMode_e autoaimMode;
    EnergyModeRecv_e energyShoot;
    uint8_t energyMode;
    uint32_t autoShootCounter;

    bool  V_Predict;
    float K_V_Predict;

    static AutoAim autoAim;

    AutoAim();

    void Reset();

    void AutoShoot();

    void Handle();
};

extern AutoAim autoAim;

#endif
