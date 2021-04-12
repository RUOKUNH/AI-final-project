/**
  ******************************************************************************
  * @FileName			    ChassisTask.cpp
  * @Description            Control the chassis motors and limit the power of them
  *                         Follow the gimbal or twist or spin
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

//#include <ChassisTask.h>

#include "includes.h"



void Chassis::Reset(const int16_t *_headDirection,const chassisPara_t chassisPara) {
    headDirection=_headDirection;
    followDirection=chassisPara.followDirection;
    lock=0;

#ifdef HERO
    followCenter=heroPara.gimbalPara.GMYPara.zero;
#endif

#ifdef INFANTRY
    followCenter=infantryPara.gimbalPara.GMYPara.zero;
#endif

    SetVelocity(0, 0, 0);
    CMFL.Reset(chassisPara.FLPara);
    CMFR.Reset(chassisPara.FRPara);
    CMBL.Reset(chassisPara.BLPara);
    CMBR.Reset(chassisPara.BRPara);



    rotate.Reset(chassisPara.Followkp,chassisPara.Followki,chassisPara.Followkd,chassisPara.FollowpMax,chassisPara.FollowiMax,
                 chassisPara.FollowdMax,chassisPara.FollowMax);

}

//使用变结构体PID
void Chassis::Reset(const int16_t *_headDirection, float _followDirection,
                    CanType_e FLcan, uint16_t _FLRxID, CanType_e FRcan, uint16_t _FRRxID,
                    CanType_e BLcan, uint16_t _BLRxID, CanType_e BRcan, uint16_t _BRRxID, float _reductionRate,
                    float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
                    float _kdA, float _kdB, float _kdC, float _pMax,float _iMax, float _dMax, float _max,

                    float _FollowkpA, float _FollowkpB, float _FollowkpC,
                    float _FollowkiA, float _FollowkiC, float _FollowkiK0,
                    float _FollowkdA, float _FollowkdB, float _FollowkdC,
                    float _FollowpMax,float _FollowiMax, float _FollowdMax, float _Followmax){
    headDirection = _headDirection;
    followDirection=_followDirection;
    lock = 0;
    followCenter = infantryPara.gimbalPara.GMYPara.zero;
    SetVelocity(0, 0, 0);
    CMFL.Reset(FLcan, _FLRxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
               _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);
    CMFR.Reset(FRcan, _FRRxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
               _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);
    CMBL.Reset(BLcan, _BLRxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
               _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);
    CMBR.Reset(BRcan, _BRRxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
               _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);
    rotate.ResetChange(_FollowkpA, _FollowkpB, _FollowkpC, _FollowkiA, _FollowkiC, _FollowkiK0,
                       _FollowkdA, _FollowkdB, _FollowkdC, _FollowpMax, _FollowiMax,  _FollowdMax, _Followmax);
}


//*********************功率限制算法***************************
//通过一定算法，使底盘4个电机的预期速度等比减小，来使功率小于阈值
void Chassis::PowerLimitation() {
    powerLimPara.kIntensityToMoment = 200000.0;
    powerLimPara.PowerMax = 0;  //powerLimPara.PowerMax=(double)Cap_Get_Aim_Power()*0.7;
    powerLimPara.rotateSpeedRate = 0.0;
    powerLimPara.CMFLRotateSpeed = (float) CMFL.GetRxMsgC6x0().rotateSpeed;
    powerLimPara.CMFRRotateSpeed = (float) CMFR.GetRxMsgC6x0().rotateSpeed;
    powerLimPara.CMBLRotateSpeed = (float) CMBL.GetRxMsgC6x0().rotateSpeed;
    powerLimPara.CMBRRotateSpeed = (float) CMBR.GetRxMsgC6x0().rotateSpeed;

    float PowerSum = CMFL.GetSpeedPID().getKp() *
                     ((CMFL.targetSpeed - powerLimPara.CMFLRotateSpeed) * powerLimPara.CMFLRotateSpeed +
                      (CMFR.targetSpeed - powerLimPara.CMFRRotateSpeed) * powerLimPara.CMFRRotateSpeed +
                      (CMBL.targetSpeed - powerLimPara.CMBLRotateSpeed) * powerLimPara.CMBLRotateSpeed +
                      (CMBR.targetSpeed - powerLimPara.CMBRRotateSpeed) * powerLimPara.CMBRRotateSpeed) /
                     powerLimPara.kIntensityToMoment;
    //************************底盘功率预期值设定**************************
    //powerLimPara.PowerMax就是预期功率
#ifdef _POWER_LIMITATION
#ifdef _CAP
    //    if(/*Cap::cap.GetCapState()==CAP_STATE_RELEASE && */Cap::cap.GetCapVoltage()>18.0){
    //        powerLimPara.PowerMax = 10000.0f;
    //    }else{
    //        powerLimPara.PowerMax = RefereeData.GameRobotState.max_chassis_power + 50 * (Cap::cap.GetCapVoltage() - 13);
    //    }


        if(RefereeData.GameRobotState.chassis_power_limit > 0) {
            fakeBuffer += ((Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) *0.3f) *
                           RefereeData.GameRobotState.chassis_power_limit -
                           Cap::cap.GetOutVoltage() * (0.9f + Cap::cap.GetOutCurr())) * 0.001f;
            INRANGE(fakeBuffer, 0, 60);
            powerLimPara.PowerMax =
                    (Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) *0.3f) *
                    RefereeData.GameRobotState.chassis_power_limit + 5.0f * (fakeBuffer - 10.0f);
        }else{
            fakeBuffer += ((Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) *0.3f) *
                           50 - Cap::cap.GetOutVoltage() * (0.9f + Cap::cap.GetOutCurr())) * 0.001f;
            INRANGE(fakeBuffer, 0, 60);
            powerLimPara.PowerMax =
                    (Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) *0.3f) *
                    50.0f + 5.0f * (fakeBuffer - 10.0f);
        }
#else
    powerLimPara.PowerMax =
            (RefereeData.GameRobotState.max_chassis_power == 0 ? 50 : RefereeData.GameRobotState.max_chassis_power) +
            5 * (RefereeData.PowerHeat.chassis_power_buffer == 0 ? 0 : RefereeData.PowerHeat.chassis_power_buffer - 10);
#endif
#else

#ifdef _CAP

    if (RefereeData.GameRobotState.chassis_power_limit > 0) {
        fakeBuffer += ((Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) * 0.3f) *
                       RefereeData.GameRobotState.chassis_power_limit -
                       Cap::cap.GetOutVoltage() * (0.9f + Cap::cap.GetOutCurr())) * 0.001f;
        INRANGE(fakeBuffer, 0, 60);
    } else {
        fakeBuffer += ((Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) * 0.3f) *
                       50 - Cap::cap.GetOutVoltage() * (0.9f + Cap::cap.GetOutCurr())) * 0.001f;
        INRANGE(fakeBuffer, 0, 60);
    }
/*
    if (powerLimPara.PowerMax < 0.0f) powerLimPara.PowerMax = 0.0f;
    if (underVoltage == 0 && Cap::cap.GetCapVoltage() < 15.0f) underVoltage = 1;
    else if (underVoltage == 1 && Cap::cap.GetCapVoltage() > 16.0f) underVoltage = 0;
*/

    if (Cap::cap.GetUnderVoltage() || dashFlag == 0 ) {
        if (RefereeData.GameRobotState.chassis_power_limit > 0) {
            powerLimPara.PowerMax =
                    (Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) * 0.3f) *
                    RefereeData.GameRobotState.chassis_power_limit + 5.0f * (fakeBuffer - 10.0f);
        } else {
            powerLimPara.PowerMax =
                    (Cap::cap.GetCapVoltage() > 13.0f ? 0.85f : (Cap::cap.GetCapVoltage() - 10.0f) * 0.3f) *
                    50.0f + 5.0f * (fakeBuffer - 10.0f);
        }
    } else {
        powerLimPara.PowerMax = 200.0f;
    }
#else
    powerLimPara.PowerMax = 10000.0f;
#endif
#endif
    if (powerLimPara.PowerMax < 0.0f) powerLimPara.PowerMax = 0.0f;
    //************************底盘功率预期值设定**************************
    //***********************通过蓝牙发送相关数据用于监视*******************
//    static uint8_t data[10];
//    data[0] = 0xAA;
//    data[1] = 0xBB;
//    data[2] = 0xCC;
//    data[3] = 0xDD;
//    *((float *) (data) + 1) = fakeBuffer;
//    *((float *) (data) + 2) = Cap::cap.GetCapVoltage();
//    *((float *) (data) + 3) = (float) powerLimPara.PowerMax;
//    HAL_UART_Transmit_DMA(&huart2, data, 4 + 4 * 3);
    //***********************通过蓝牙发送相关数据用于监视*******************

    if (PowerSum > powerLimPara.PowerMax) {
        powerLimPara.rotateSpeedRate =
                1.0f -
                (PowerSum - powerLimPara.PowerMax) /
                (CMFL.GetSpeedPID().getKp() * (CMFL.targetSpeed * powerLimPara.CMFLRotateSpeed +
                                               CMFR.targetSpeed * powerLimPara.CMFRRotateSpeed +
                                               CMBL.targetSpeed * powerLimPara.CMBLRotateSpeed +
                                               CMBR.targetSpeed * powerLimPara.CMBRRotateSpeed))
                * powerLimPara.kIntensityToMoment;

        if (powerLimPara.rotateSpeedRate > 1.0f)powerLimPara.rotateSpeedRate = 1.0f;
        if (powerLimPara.rotateSpeedRate < 0.0f)powerLimPara.rotateSpeedRate = 0.0f;
        //powerLimPara.rotateSpeedRate就是四轮角速度等比减小的比率
        CMFL.targetSpeed *= powerLimPara.rotateSpeedRate;
        CMFR.targetSpeed *= powerLimPara.rotateSpeedRate;
        CMBL.targetSpeed *= powerLimPara.rotateSpeedRate;
        CMBR.targetSpeed *= powerLimPara.rotateSpeedRate;
    }
}
//*********************功率限制算法***************************
void Chassis::Handle() {
    ChassisDataDecoding();
    PowerLimitation();

    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}
//******************底盘转动相关解算***********************
void Chassis::ChassisDataDecoding() {
    if (!lock) {
        ControlRotate();

        float cosPlusSin, cosMinusSin, GMYEncoderAngle;

#ifdef INFANTRY
        GMYEncoderAngle = (float) (*headDirection - infantryPara.gimbalPara.GMYPara.zero) * 6.28f / 8192.0f;
#endif

#ifdef HERO
        GMYEncoderAngle = (float) (*headDirection - heroPara.gimbalPara.GMYPara.zero) * 6.28f / 8192.0f*(-1);
#endif

        cosPlusSin = cos(GMYEncoderAngle) + sin(GMYEncoderAngle);
        cosMinusSin = cos(GMYEncoderAngle) - sin(GMYEncoderAngle);

#ifdef USE_CHASSIS_FOLLOW
        CMFL.targetSpeed =
                (forwardBackVelocity * cosMinusSin + leftRightVelocity * cosPlusSin + rotateVelocity) * 12;
        CMFR.targetSpeed =
                (-forwardBackVelocity * cosPlusSin + leftRightVelocity * cosMinusSin + rotateVelocity) * 12;
        CMBL.targetSpeed =
                (forwardBackVelocity * cosPlusSin - leftRightVelocity * cosMinusSin + rotateVelocity) * 12;
        CMBR.targetSpeed =
                (-forwardBackVelocity * cosMinusSin - leftRightVelocity * cosPlusSin + rotateVelocity) * 12;
#else
        CMFL.targetSpeed = (forwardBackVelocity + leftRightVelocity + rotateVelocity) * 12;
        CMFR.targetSpeed = (-forwardBackVelocity + leftRightVelocity + rotateVelocity) * 12;
        CMBL.targetSpeed = (forwardBackVelocity - leftRightVelocity + rotateVelocity) * 12;
        CMBR.targetSpeed = (-forwardBackVelocity - leftRightVelocity + rotateVelocity) * 12;
#endif
    } else {
        CMFL.targetSpeed = 0;
        CMFR.targetSpeed = 0;
        CMBL.targetSpeed = 0;
        CMBR.targetSpeed = 0;
    }
}

//三种底盘转动的状态
void Chassis::TwistSpin() {
    angleToCenter = fabs((float)(followCenter - *headDirection) * 360 / 8192.0f - (float) twistGapAngle);
    if (angleToCenter > 360) angleToCenter -= 360;
    switch (twistState) {
        case 1: {
            //正向陀螺

            rotateVelocity = 60;
            INRANGE(forwardBackVelocity, -360.0f, 360.0f);
            INRANGE(leftRightVelocity, -360.0f, 360.0f);
            break;
        }
        case 2: {
            //反向陀螺
            rotateVelocity = -60;
            INRANGE(forwardBackVelocity, -300.0f, 300.0f);
            INRANGE(leftRightVelocity, -300.0f, 300.0f);
            break;
        }
        case 3: {
            //扭腰（来回切换期望朝向)
            switch (twistGapAngle) {
                case 0: {
                    twistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;
                    break;
                }
                case CHASSIS_TWIST_ANGLE_LIMIT: {
                    if (angleToCenter < 10)
                        twistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;
                    break;
                }
                case -CHASSIS_TWIST_ANGLE_LIMIT: {
                    if (angleToCenter < 10)
                        twistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
                    break;
                }
            }
            rotateVelocity = (float) (followCenter - *headDirection) * 360 / 8192.0f - (float) twistGapAngle;
            break;
        }
        default: {
            //正常底盘跟随
            twistGapAngle = 0;
            rotateVelocity = (float) (followCenter - *headDirection) * 360 / 8192.0f - (float) twistGapAngle;
            break;
        }
    }
    NORMALIZE_ANGLE180(rotateVelocity);
}
//******************底盘转动相关解算***********************
void Chassis::ControlRotate() {
#ifdef USE_CHASSIS_FOLLOW
    TwistSpin();
    //*******底盘跟随正负反馈**************
    rotateVelocity*=followDirection;
    //*******底盘跟随正负反馈**************
#endif

#ifdef VARIABLE_STRUCTRUE_PID
    rotateVelocity = rotate.CalcChange(rotateVelocity,0);
#else
    //***********底盘跟随松紧的PID****************
    rotateVelocity = rotate.Calc(rotateVelocity, 0);
#endif
}

//设定底盘速度
void Chassis::SetVelocity(int16_t _forwardBackVelocity, int16_t _leftRightVelocity, int16_t _rotateVelocity) {
    forwardBackVelocity = (float) _forwardBackVelocity * CHASSIS_SPEED_K;
    leftRightVelocity = (float) _leftRightVelocity * CHASSIS_SPEED_K / 3 * 2;
    //有底盘跟随的情况下，会自动解算旋转速度，所以不能强行设置
#ifndef USE_CHASSIS_FOLLOW
    rotateVelocity = (float) _rotateVelocity * ROTATE_SPEED_K;
#endif
}
//用增量的方式设定底盘速度
void Chassis::AddVelocity(int16_t accFB, int16_t accLR) {
    forwardBackVelocity += (float) accFB;
    leftRightVelocity += (float) accLR;
    INRANGE(forwardBackVelocity, -1000.0f, 1000.0f);
    INRANGE(leftRightVelocity, -1000.0f, 1000.0f);
}

/*void SetBurstFlag(uint8_t state){
    Chassis::chassis.burstFlag = state;
}*/

Chassis Chassis::chassis;
