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
#include "AutoAimTask.h"
#include "includes.h"

#ifdef INFANTRY_3
const float YAW_OFFSET = 0.;
const float PIT_OFFSET = 0.;
const float PIT_SENTRY_OFFSET = 0.;
const float YAW_PREDICT_K_LEFT  = 1.3;
const float YAW_PREDICT_K_RIGHT = 1.3;
const float PIT_SENTRY_OFFSET_LEFT = 0.0f;
const float PIT_SENTRY_OFFSET_RIGHT = 0.0f;
#elif defined INFANTRY_4
//const float YAW_OFFSET = -0.35f;
//const float PIT_OFFSET = -0.50f;
//const float YAW_OFFSET = -0.4f;
//const float PIT_OFFSET = -0.55f;
//const float PIT_SENTRY_OFFSET_LEFT = -1.0f;
//const float PIT_SENTRY_OFFSET_RIGHT = -0.3f;
//const float YAW_PREDICT_K_LEFT  = 1.55;
//const float YAW_PREDICT_K_RIGHT = 1.95;
const float YAW_OFFSET = 0.0f;
const float PIT_OFFSET = -0.35f;
const float PIT_SENTRY_OFFSET_LEFT = -0.73f;
const float PIT_SENTRY_OFFSET_RIGHT = -0.73f;
const float YAW_PREDICT_K_LEFT  = 1.3;
const float YAW_PREDICT_K_RIGHT = 1.3;
#elif defined HERO
const float YAW_OFFSET = 0.;
const float PIT_OFFSET = 0.;z
const float PIT_SENTRY_OFFSET = 0.;
const float YAW_PREDICT_K_LEFT  = 1.;
const float YAW_PREDICT_K_RIGHT = 1.;
#endif

// functions in AutoAimRecv
inline void AutoAimRecv::ParaInit() {
    Check        = 0;
    data.yaw     = 0.0;
    data.pitch   = 0.0;
    data.yaw_v   = 0.0;
    data.pitch_v = 0.0;
    recvivedCnt  = 0;
    sendCnt      = 0;
    lastYaw      = 0.0;
    autoControl  = false;
    autoShoot    = false;
    recvFinish   = false;
}


AutoAimRecv::AutoAimRecv() {
    ParaInit();
}


void AutoAimRecv::RecvStart(){
    HAL_UART_Receive_DMA(&AUTOAIM_UART, AutoAimBuffer, BUFFER_SIZE);
}


void AutoAimRecv::UartRxCpltCallback() {
    if(RECV_LEN(AUTOAIM_UART) % sizeof(data) == 0) {
        memcpy(&data, AutoAimBuffer, sizeof(data));
        recvFinish = true;
    }
    HAL_UART_Receive_DMA(&AUTOAIM_UART, AutoAimBuffer, BUFFER_SIZE);
}

// AutoAim类的函数
void AutoAim::AutoAimParaInit() {
    sendData.start = 's';
    sendData.end   = 'e';

    energyShoot      = ENERGY_FOLLOW;
    autoaimMode      = manual;
    energyMode       = HIT_SMALL;
    worldYaw         = 0.0f;
    V_Predict        = true;
    K_V_Predict      = 1;

    autoShootCounter = 0;
    speedCalc_yaw = false;
    speedCalc_pitch = false;

    posPtr = 0;
}

AutoAim::AutoAim(){
    AutoAimParaInit();
}

void AutoAim::Reset(){
    AutoAimParaInit();
    RecvStart();
}


inline void AutoAim::DataProcess() {
//    if (autoaimMode != manual && recvFinish) {
    if (recvFinish) {
        recvFinish = false;
        Check = 0;
        for (uint8_t *ptr = (uint8_t *) &(data.yaw); ptr < (uint8_t *) &(data.check); ++ptr)
            Check += *ptr;

        if (data.start == 's' && data.end == 'e' && !isnan(data.yaw) && !isnan(data.pitch) && Check == data.check) {
            recvivedCnt++;
            // 打乱此次数据，下次不会复用
            data.check += 1;
//            output.yaw = data.yaw + YAW_OFFSET;
//            output.pitch = data.pitch + PIT_OFFSET;

            // 敌方静止时 相信视觉发的数据是对的！
            if (fabs(data.yaw_v / data.predict_dist_3d) < 0.10f) {
                output.yaw = data.yaw + YAW_OFFSET;
                output.pitch = data.pitch + PIT_OFFSET;
            } else if (data.yaw_v > 0.) {
            // 敌方运动时 坚信视觉发的数据是偏慢的！
                // 在四号车中yaw为正，云台向左，pit为正，云台向上 故向左走乘以YAW_PREDICT_K_LEFT
                output.yaw = YAW_PREDICT_K_LEFT * (data.yaw + YAW_OFFSET);     //可能要调参（除以一个参数）
                output.pitch = data.pitch + PIT_OFFSET;
            } else {
                output.yaw = YAW_PREDICT_K_RIGHT * (data.yaw + YAW_OFFSET);
                output.pitch = data.pitch + PIT_OFFSET;
            }

            if (Gimbal::gimbal.GMRealAngle(GMPitch) >= 16.2f) {
                if ((output.yaw_v/data.predict_dist_3d) > 0.1f) // 左行
                    output.pitch += PIT_SENTRY_OFFSET_LEFT;
                else if ((output.yaw_v/data.predict_dist_3d) < -0.1f)
                    output.pitch += PIT_SENTRY_OFFSET_RIGHT;
                else
                    output.pitch -= 0.60f;
            }
            output.yaw_v = data.yaw_v / 180.0 * 3.1415926;
            output.pitch_v = data.pitch_v / 180.0 * 3.1415926;
            infantryPara.gimbalPara.GMYPara.gmanglewatch = output.yaw;
            //同上
            if (autoaimMode == antiTop) {
                antitop_period = data.period;
                antitop_period_offset = data.period_offset;
            }

            speedCalc_yaw = true;
            speedCalc_pitch = true;
            autoControl = true;
            autoShoot = data.shoot_flag;

            INRANGE(output.yaw, -25.0f, 25.0f);
            INRANGE(output.pitch, -25.0f, 27.0f);
        } else {
            autoControl = false;
        }
    }   else {autoControl = false;}
}

void AutoAim::Control(){
    if (autoaimMode != manual && autoControl) {
        uint8_t interval = 25;
        uint8_t tmp = (posPtr >= interval) ? posPtr-interval : posPtr+80-interval;
//        Gimbal::gimbal.AutoAimSetPosition(data.pitch, data.yaw, data.pitch_v, data.yaw_v);
        Gimbal::gimbal.AutoAimSetPosition(PitchRealAngle[tmp], YawRealAngle[tmp],output.pitch, output.yaw, output.pitch_v, output.yaw_v);
    }
    autoControl  = false;
}

void AutoAim::AutoShoot() {
    if (autoaimMode == antiTop) {
        static uint16_t tmpPeriod = 0;
        if (tmpPeriod < data.period_offset) {
            tmpPeriod++;
            return;
        }
        tmpPeriod++;
        if (autoShoot && tmpPeriod >= data.period) {
            tmpPeriod = 0;
            Shoot::shoot.ShootOneBullet(0);
        }
        return;
    }

    if (autoShoot && autoaimMode == predict) {
        Shoot::shoot.ShootOneBullet(0);
    }
    autoShoot = false;
}

void AutoAim::UartTxInfo(){
    // 2ms发一次 每毫秒调用本函数所以增加一个 is2ms 来判断
    static uint8_t is2ms = 1;
    if (is2ms % 10 == 0) {
//        sendData.start = 's';  // 在AutoAimParaInit里面
        sendData.bulletSpeed = RefereeData.GameRobotState.shooter_id1_17mm_speed_limit - 1;
//        sendData.bulletSpeed = 13.0f;
        sendData.enemyColor = RefereeData.GameRobotState.robot_id < 10 ? 0 : 1;
//        sendData.enemyColor = 0;  // 0 - blue   1 - red
        if (autoaimMode == predict) { // 自瞄
            if (energyMode == 6) {    // antiTop
                sendData.program_mode = 0;
                sendData.energy_mode = 0;
            } else {                  // predict
                sendData.program_mode = 0;
                sendData.energy_mode = 0;
            }
        } else if (autoaimMode == energy) {
            sendData.program_mode = 1;
            sendData.energy_mode = energyMode;
        }

        sendData.crc = 0;
        for (uint8_t *ptr = (uint8_t *) &sendData.bulletSpeed; ptr < &sendData.crc; ++ptr) {
            sendData.crc += *ptr;
        }
        sendData.end = 'e';  // 在AutoAimParaInit里面
        HAL_UART_Transmit_IT(&AUTOAIM_UART, (uint8_t *) &sendData, sizeof(sendData));
        sendCnt++;
        is2ms = 0;
    }
    is2ms++;
}


void AutoAim::RestoreRealAngle() {
    PitchRealAngle[posPtr] = Gimbal::gimbal.GMRealAngle(GMPitch);
    YawRealAngle[posPtr] = Gimbal::gimbal.GMRealAngle(GMYaw);
    posPtr++;
    if (posPtr >= 80) posPtr = 0;
}

void AutoAim::Handle(){
    UartTxInfo();
    RestoreRealAngle();
    DataProcess();
    Control();
    AutoShoot();
}

AutoAim AutoAim::autoAim;