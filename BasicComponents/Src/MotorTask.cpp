/**
  ******************************************************************************
  * @FileName			    MotorTask.cpp
  * @Description            Control motors
  * @author                 Chang Liu
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "includes.h"

void ChassisMotor::Handle() {
#ifdef VARIABLE_STRUCTRUE_PID //变结构体PID运算
    intensity = speedPid.CalcChange(targetAngle,RxMsgC6x0.rotateSpeed);
#else
    //简单的单环PID运算。
    intensity = speedPid.Calc(targetSpeed,RxMsgC6x0.rotateSpeed);
#endif
}

void NormalMotor::Handle() {
    //获取反馈值
    thisAngle = (float) RxMsgC6x0.angle;
    thisSpeed = (float) RxMsgC6x0.rotateSpeed;
    if (firstEnter) {
        lastRead = thisAngle;
        firstEnter = 0;
        return;
    }
    //利用反馈值（单圈）计算多圈累计角度
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 4000)
            realAngle += (thisAngle + 8192 - lastRead) * 360.0f / 8192.0f / reductionRate;
        else
            realAngle -= (lastRead - thisAngle) * 360.0f / 8192.0f / reductionRate;
    } else {
        if (thisAngle - lastRead > 4000)
            realAngle -= (lastRead + 8192 - thisAngle) * 360.0f / 8192.0f / reductionRate;
        else
            realAngle += (thisAngle - lastRead) * 360.0f / 8192.0f / reductionRate;
    }

    //上面的代码主要用于处理realAngle(作为PID的反馈值fdb) 主要是处理映射关系的一些运算。
#ifdef VARIABLE_STRUCTRUE_PID
    intensity = speedPid.CalcChange(anglePid.CalcChange(targetAngle,realAngle),thisSpeed);
#else
    intensity = speedPid.Calc(anglePid.Calc(targetAngle, realAngle), thisSpeed);
    //双环PID运算
#endif

    lastRead = thisAngle;
}

/*注意：
 *云台反馈值采用的是陀螺仪(IMU)返回的数据(因为主控板是放在云台上的)。
 *控制逻辑就是双环PID
 */
void GMPitchMotor::Handle() {
    //获取反馈值
    thisSpeed = *speedFeedback;
    thisAngle = *angleFeedback;
    if (firstEnter) {
        lastRead = thisAngle;
        realAngle = -(float) (infantryPara.gimbalPara.GMPPara.zero - RxMsgC6x0.angle) * 360.0f / 8192.0f;
        NORMALIZE_ANGLE180(realAngle);
        firstEnter = 0;
        return;
    }


    //利用反馈值（单圈）计算多圈累计角度
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 180)
            realAngle += (thisAngle + 360 - lastRead);
        else
            realAngle -= (lastRead - thisAngle);
    } else {
        if (thisAngle - lastRead > 180)
            realAngle -= (lastRead + 360 - thisAngle);
        else
            realAngle += (thisAngle - lastRead);
    }

//    realAngle = thisAngle;

    /*
    if (fabs(realAngle - targetAngle) < 10)
        reseted = 1;
    if (reseted == 0) anglePid.PIDInfo.outputMax = 1.0f;
    else anglePid.PIDInfo.outputMax = 10.0f;
    */
    anglePid.setOutputMax(10.0f);
#ifdef HERO
    INRANGE(targetAngle,  -40.0f, 21.4f);
#endif
#ifdef INFANTRY
    INRANGE(targetAngle, -30.0f, 30.0f);
#endif
    lastRead = thisAngle;

    //Friction compensation
    int16_t tmp;
#ifdef INFANTRY
    tmp = speedPid.Calc(
            anglePid.Calc(targetAngle, realAngle) + autoAimPitch_v * infantryPara.gimbalPara.GMPPara.anglekp,
            thisSpeed);
    autoAimPitch_v = 0.0;
#endif
#ifdef HERO
    tmp = speedPid.Calc(anglePid.Calc(targetAngle,realAngle) + autoAimPitch_v * heroPara.gimbalPara.GMPPara.anglekp , thisSpeed);
#endif

/*#ifdef VARIABLE_STRUCTRUE_PID
    int16_t tmp = speedPid.CalcChange(anglePid.CalcChange(targetAngle, realAngle), thisSpeed);
#else
    int16_t tmp = speedPid.Calc(anglePid.Calc(targetAngle,realAngle),thisSpeed);
#endif*/
intensity=tmp;
//    if (tmp > 500) {
//        intensityThisTime += GMP_Friction_compensation;
//    } else if (tmp < -500) {
//        intensityThisTime -= GMP_Friction_compensation;
//    }
//
//    intensityThisTime = GM_PITCH_GRAVITY_COMPENSATION + tmp;
//    intensity = infantryPara.gimbalPara.GMPPara.presentPercentage * intensityThisTime +
//                (1 - infantryPara.gimbalPara.GMPPara.presentPercentage) * intensityLastTime;
//    intensityLastTime = intensityThisTime;
}

void GMYawMotor::Handle() {
    //获取反馈值
    thisSpeed = *speedFeedback;
    thisAngle = *angleFeedback;
//    infantryPara.gimbalPara.GMYPara.gmanglewatch = RxMsgC6x0.angle;
//    infantryPara.gimbalPara.GMYPara.gmanglewatch = thisAngle;

    if (firstEnter) {
        lastRead = thisAngle;
#ifdef INFANTRY
        realAngle = -(float) (infantryPara.gimbalPara.GMYPara.zero - RxMsgC6x0.angle) * 360.0f / 8192.0f;
#endif
#ifdef HERO
        realAngle = -(float) (heroPara.gimbalPara.GMYPara.zero - RxMsgC6x0.angle) * 360.0f / 8192.0f;
#endif
        NORMALIZE_ANGLE180(realAngle);
        firstEnter = 0;
        return;
    }

    //利用反馈值（单圈）计算多圈累计角度
    if (thisAngle <= lastRead) {
        if (lastRead - thisAngle > 180)
            realAngle += (thisAngle + 360 - lastRead);
        else
            realAngle -= (lastRead - thisAngle);
    } else {
        if (thisAngle - lastRead > 180)
            realAngle -= (lastRead + 360 - thisAngle);
        else
            realAngle += (thisAngle - lastRead);
    }

    if (fabs(realAngle - targetAngle) < 30)
        reseted = 1;

    if (reseted == 0) anglePid.setOutputMax(1.0f);
    else anglePid.setOutputMax(30.0f);

    lastRead = thisAngle;

    //Friction compensation

    int16_t tmp;
#ifdef INFANTRY
    tmp = speedPid.Calc(anglePid.Calc(targetAngle, realAngle) + autoAimYaw_v * infantryPara.gimbalPara.GMYPara.anglekp,
                        thisSpeed);
    autoAimYaw_v = 0.0;
#endif
#ifdef HERO
    tmp = speedPid.Calc(anglePid.Calc(targetAngle,realAngle) + autoAimYaw_v *heroPara.gimbalPara.GMYPara.anglekp, thisSpeed);
#endif
/*#ifdef VARIABLE_STRUCTRUE_PID
    int16_t tmp = speedPid.CalcChange(anglePid.CalcChange(targetAngle,realAngle),thisSpeed);
#else
    int16_t tmp = speedPid.Calc(anglePid.Calc(targetAngle,realAngle),thisSpeed);
#endif*/
intensity=tmp;
//    if (tmp > 500) {
//        intensityThisTime += GMY_Friction_compensation;
//    } else if (tmp < -500) {
//        intensityThisTime -= GMY_Friction_compensation;
//    }
//
//    INRANGE(tmp, -speedPid.getOutputMax() - GM_YAW_GRAVITY_COMPENSATION,
//            speedPid.getOutputMax() - GM_YAW_GRAVITY_COMPENSATION);
//    intensityThisTime = GM_YAW_GRAVITY_COMPENSATION + tmp;
//    intensity = infantryPara.gimbalPara.GMYPara.presentPercentage * intensityThisTime +
//                (1 - infantryPara.gimbalPara.GMYPara.presentPercentage) * intensityLastTime;
//    intensityLastTime = intensityThisTime;
#ifdef USE_CUBE_MONITOR
    refreshPara(monitorPara_GYMotor,realAngle,0,intensity);
#endif
}

void Motor::motorInit() {
    motorConliction = 0;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 8; j++) {
            motors[i][j] = nullptr;
        }
    }
}

void Motor::Reset(chassisMotorPara_t chassisMotorPara)
{
    everRx = 0;
    RxMsgC6x0.angle = 0;
    reductionRate=chassisMotorPara.reductionRate;
    RxMsgC6x0.rotateSpeed = 0;
    RxMsgC6x0.moment = 0;
    targetAngle = 0;
    targetSpeed = 0;
    reseted = 0;
    firstEnter = 1;
    lastRead = 0;
    realAngle = 0;
    intensity = 0;
    RxID=chassisMotorPara.RxID;
    if (chassisMotorPara.CAN == CAN_TYPE_1)
        Motor::motors[0][chassisMotorPara.RxID - 0x201] = this;
    if (chassisMotorPara.CAN == CAN_TYPE_2)
        Motor::motors[1][chassisMotorPara.RxID - 0x201] = this;
    speedPid.Reset(chassisMotorPara.kp,chassisMotorPara.ki,chassisMotorPara.kd,chassisMotorPara.pMax,
                   chassisMotorPara.iMax,chassisMotorPara.dMax,chassisMotorPara.max);
}

void Motor::Reset(normalMotorPara_t normalMotorPara)
{
    everRx = 0;
    RxMsgC6x0.angle = 0;
    reductionRate=normalMotorPara.reductionRate;
    RxMsgC6x0.rotateSpeed = 0;
    RxMsgC6x0.moment = 0;
    targetAngle = 0;
    targetSpeed = 0;
    reseted = 0;
    firstEnter = 1;
    lastRead = 0;
    realAngle = 0;
    intensity = 0;
    RxID=normalMotorPara.RxID;
    if (normalMotorPara.CAN == CAN_TYPE_1)
        Motor::motors[0][normalMotorPara.RxID - 0x201] = this;
    if (normalMotorPara.CAN == CAN_TYPE_2)
        Motor::motors[1][normalMotorPara.RxID - 0x201] = this;
    speedPid.Reset(normalMotorPara.kp,normalMotorPara.ki,normalMotorPara.kd,normalMotorPara.pMax,
                   normalMotorPara.iMax,normalMotorPara.dMax,normalMotorPara.max);
}



//void
//ChassisMotor::Reset_two(CanType_e can, uint16_t _RxID, float _reductionRate, float _kp, float _ki, float _kd, float _pMax,
//             float _iMax,
//             float _dMax, float _max) {
//    everRx = 0;
//    RxMsgC6x0.angle = 0;
//    reductionRate = _reductionRate;
//    RxMsgC6x0.rotateSpeed = 0;
//    RxMsgC6x0.moment = 0;
//    targetSpeed = 0;
//    reseted = 0;
//    firstEnter = 1;
//    lastRead = 0;
//    realAngle = 0;
//    intensity = 0;
//
//    RxID = _RxID;
//    TxID = 0x200;
//    if (can == CAN_TYPE_1) {
//        if(Motor::motors[0][_RxID - 0x201] == nullptr)
//            Motor::motors[0][_RxID - 0x201] = this;
//        else motorConliction = 1;
//    }
//    if (can == CAN_TYPE_2) {
//        if(Motor::motors[1][_RxID - 0x201] == nullptr)
//            Motor::motors[1][_RxID - 0x201] = this;
//        else motorConliction = 1;
//    }
//
//    speedPid.Reset(_kp, _ki, _kd, _pMax, _iMax, _dMax, _max);
//}

//使用变结构体PID
void Motor::Reset(CanType_e can, uint16_t _RxID, float _reductionRate,
                  float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
                  float _kdA, float _kdB, float _kdC, float _pMax,float _iMax, float _dMax, float _max) {
    everRx = 0;
    RxMsgC6x0.angle = 0;
    reductionRate = _reductionRate;
    RxMsgC6x0.rotateSpeed = 0;
    RxMsgC6x0.moment = 0;
    targetAngle = 0;
    targetSpeed = 0;
    reseted = 0;
    firstEnter = 1;
    lastRead = 0;
    realAngle = 0;
    intensity = 0;
    intensityThisTime = 0;
    intensityLastTime = 0;

    RxID = _RxID;
    if (can == CAN_TYPE_1)
        Motor::motors[0][_RxID - 0x201] = this;
    if (can == CAN_TYPE_2)
        Motor::motors[1][_RxID - 0x201] = this;

    speedPid.ResetChange(_kpA, _kpB, _kpC, _kiA, _kiC, _kiK0, _kdA, _kdB, _kdC,
                         _pMax, _iMax, _dMax, _max);
}

void Motor::StopReset() {
    everRx = 0;
    RxMsgC6x0.angle = 0;
    RxMsgC6x0.rotateSpeed = 0;
    RxMsgC6x0.moment = 0;
    targetAngle = 0;
    targetSpeed = 0;
    reseted = 0;
    firstEnter = 1;
    lastRead = 0;
    realAngle = 0;
    intensity = 0;
    speedPid.Reset();
    anglePid.Reset();
}

void ChassisMotor::Reset(chassisMotorPara_t chassisMotorPara) {
    Motor::Reset(chassisMotorPara);
}


void ChassisMotor::RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max) {
    this->speedPid.RefreshPara(_kp, _ki, _kd, _pMax, _iMax, _dMax, _max);
}

//使用变结构体PID
void ChassisMotor::Reset(CanType_e can, uint16_t _RxID, float _reductionRate,
                         float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
                         float _kdA, float _kdB, float _kdC, float _pMax,float _iMax, float _dMax, float _max) {
    Motor::Reset(can, _RxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
                 _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);
}

void NormalMotor::Reset(normalMotorPara_t normalMotorPara) {
    Motor::Reset(normalMotorPara);
    anglePid.Reset(normalMotorPara.anglekp, normalMotorPara.angleki, normalMotorPara.anglekd, normalMotorPara.anglepMax,
                   normalMotorPara.angleiMax, normalMotorPara.angledMax, normalMotorPara.anglemax);
}
void NormalMotor::RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                              float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                              float _angledMax, float _anglemax) {
    this->speedPid.RefreshPara(_kp, _ki, _kd, _pMax, _iMax, _dMax, _max);
    this->anglePid.RefreshPara(_anglekp, _angleki, _anglekd, _anglepMax, _angleiMax, _angledMax, _anglemax);
}
void GMPitchMotor::Reset(normalMotorPara_t gmpPara) {
    Motor::Reset(gmpPara);
    anglePid.Reset(gmpPara.anglekp, gmpPara.angleki, gmpPara.anglekd, gmpPara.anglepMax, gmpPara.angleiMax,
                   gmpPara.angledMax, gmpPara.anglemax);
    thisAngle = 0;
    thisSpeed = 0;
}


//void GMPitchMotor::Reset(const float *_speedFeedback, const float *_angleFeedback){
//    speedFeedback=_speedFeedback;
//    angleFeedback=_angleFeedback;
//    thisAngle=*angleFeedback;
//    thisSpeed=*speedFeedback;
//    lastRead=thisAngle;
//}

void GMPitchMotor::Reset(const float *_speedFeedback, const float *_angleFeedback,normalMotorPara_t gmpPara) {
    Motor::Reset(gmpPara);
    anglePid.Reset(gmpPara.anglekp, gmpPara.angleki, gmpPara.anglekd, gmpPara.anglepMax, gmpPara.angleiMax,
                   gmpPara.angledMax,
                   gmpPara.anglemax);
    thisAngle = 0;
    thisSpeed = 0;
    autoAimPitch_v = 0.0;
    speedFeedback = _speedFeedback;
    angleFeedback = _angleFeedback;
}

void GMYawMotor::Reset(normalMotorPara_t gmyPara) {
    Motor::Reset(gmyPara);
    anglePid.Reset(gmyPara.anglekp, gmyPara.angleki, gmyPara.anglekd, gmyPara.anglepMax, gmyPara.angleiMax,
                   gmyPara.angledMax,
                   gmyPara.anglemax);
    thisAngle = 0;
    thisSpeed = 0;
    autoAimYaw_v = 0.0;
}

void GMYawMotor::Reset(const float *_speedFeedback, const float *_angleFeedback,normalMotorPara_t gmyPara) {
#ifdef USE_CUBE_MONITOR
    this->GetSpeedPID().EnableMonitoringProcessQuantity(monitorPara.watches,monitorPara.watches+4
    ,monitorPara.watches+8,monitorPara.watches+12,monitorPara.watches+16);
#endif
    Motor::Reset(gmyPara);
    anglePid.Reset(gmyPara.anglekp, gmyPara.angleki, gmyPara.anglekd, gmyPara.anglepMax, gmyPara.angleiMax,
                   gmyPara.angledMax,
                   gmyPara.anglemax);
    thisAngle = 0;
    thisSpeed = 0;
    speedFeedback = _speedFeedback;
    angleFeedback = _angleFeedback;
}
//void GMYawMotor::Reset(const float *_speedFeedback, const float *_angleFeedback){
//    speedFeedback=_speedFeedback;
//    angleFeedback=_angleFeedback;
//    thisAngle=*angleFeedback;
//    thisSpeed=*speedFeedback;
//    lastRead=thisAngle;
//}

void GMYawMotor::RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                             float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                             float _angledMax, float _anglemax) {
    this->speedPid.RefreshPara(_kp, _ki, _kd, _pMax, _iMax, _dMax, _max);
    this->anglePid.RefreshPara(_anglekp, _angleki, _anglekd, _anglepMax, _angleiMax, _angledMax, _anglemax);
}




void GMPitchMotor::RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                               float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                               float _angledMax, float _anglemax) {
    this->speedPid.RefreshPara(_kp, _ki, _kd, _pMax, _iMax, _dMax, _max);
    this->anglePid.RefreshPara(_anglekp, _angleki, _anglekd, _anglepMax, _angleiMax, _angledMax, _anglemax);
}

//使用变结构体PID
void GMYawMotor::Reset(const float *_speedFeedback, const float *_angleFeedback, CanType_e can, uint16_t _RxID, float _reductionRate,
                       float _kpA, float _kpB, float _kpC,
                       float _kiA, float _kiC, float _kiK0,
                       float _kdA, float _kdB, float _kdC,
                       float _pMax,float _iMax, float _dMax,float _max,
                       float _anglekpA, float _anglekpB, float _anglekpC,
                       float _anglekiA, float _anglekiC, float _anglekiK0,
                       float _anglekdA, float _anglekdB, float _anglekdC,
                       float _anglepMax, float _angleiMax, float _angledMax,float _anglemax) {
    Motor::Reset(can, _RxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
                 _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);

    anglePid.ResetChange(_anglekpA, _anglekpB, _anglekpC, _anglekiA, _anglekiC, _anglekiK0,
                         _anglekdA, _anglekdB, _anglekdC, _anglepMax, _angleiMax, _angledMax, _anglemax);
    thisAngle = 0;
    thisSpeed = 0;
    speedFeedback = _speedFeedback;
    angleFeedback = _angleFeedback;
}
//使用变结构体PID
void GMPitchMotor::Reset(const float *_speedFeedback, const float *_angleFeedback, CanType_e can, uint16_t _RxID, float _reductionRate,
                         float _kpA, float _kpB, float _kpC,
                         float _kiA, float _kiC, float _kiK0,
                         float _kdA, float _kdB, float _kdC,
                         float _pMax,float _iMax, float _dMax,float _max,
                         float _anglekpA, float _anglekpB, float _anglekpC,
                         float _anglekiA, float _anglekiC, float _anglekiK0,
                         float _anglekdA, float _anglekdB, float _anglekdC,
                         float _anglepMax, float _angleiMax, float _angledMax,float _anglemax) {
    Motor::Reset(can, _RxID, _reductionRate, _kpA, _kpB, _kpC, _kiA, _kiC, _kiK0,
                 _kdA, _kdB, _kdC, _pMax, _iMax, _dMax, _max);

    anglePid.ResetChange(_anglekpA, _anglekpB, _anglekpC, _anglekiA, _anglekiC, _anglekiK0,
                         _anglekdA, _anglekdB, _anglekdC, _anglepMax, _angleiMax, _angledMax, _anglemax);
    thisAngle = 0;
    thisSpeed = 0;
    speedFeedback = _speedFeedback;
    angleFeedback = _angleFeedback;
}

Motor *Motor::motors[2][8];
uint8_t Motor::motorConliction;
