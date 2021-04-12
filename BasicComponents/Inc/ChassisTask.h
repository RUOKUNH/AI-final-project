/**
  ******************************************************************************
  * @FileName			    ChassisTask.h
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

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "includes.h"

#define CHASSIS_SPEED_K            0.85f
#define ROTATE_SPEED_K             0.07f





class Chassis {
public:
    static Chassis chassis;
#ifdef TEST_MOTOR
    ChassisMotor testmotor;
#endif
    ChassisMotor CMFL, CMFR, CMBL, CMBR;
    uint8_t twistState;
    uint8_t lock;
    uint8_t dashFlag = 0;

    void Reset(const int16_t *_headDirection, const chassisPara_t chassisPara);  //初始化

//    inline void Reset(const int16_t *_headDirection) { headDirection = _headDirection; } //重置底盘跟随角度反馈值的来源

    //初始化
//    void
//    Reset(const int16_t *_headDirection, float _followDirection, CanType_e FLcan, uint16_t _FLRxID, CanType_e FRcan,
//          uint16_t _FRRxID,
//          CanType_e BLcan, uint16_t _BLRxID,
//          CanType_e BRcan, uint16_t _BRRxID, float _reductionRate, float _kp, float _ki, float _kd,
//          float _pMax,
//          float _iMax, float _dMax, float _max, float _Followkp, float _Followki, float _Followkd,
//          float _FollowpMax, float _FollowiMax, float _FollowdMax, float _Followmax);


    //使用变结构体PID
    void Reset(const int16_t *_headDirection, float _followDirection,
               CanType_e FLcan, uint16_t _FLRxID, CanType_e FRcan, uint16_t _FRRxID,
               CanType_e BLcan, uint16_t _BLRxID, CanType_e BRcan, uint16_t _BRRxID, float _reductionRate,
               float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
               float _kdA, float _kdB, float _kdC, float _pMax, float _iMax, float _dMax, float _max,

               float _FollowkpA, float _FollowkpB, float _FollowkpC,
               float _FollowkiA, float _FollowkiC, float _FollowkiK0,
               float _FollowkdA, float _FollowkdB, float _FollowkdC,
               float _FollowpMax, float _FollowiMax, float _FollowdMax, float _Followmax);


    //**************底盘锁定相关函数**********************
    inline void LockChassis() { lock = 1; }

    inline void UnlockChassis() { lock = 0; }

    inline uint8_t IsLocked() const { return lock; }

    //**************底盘锁定相关函数**********************
    void Handle();

    //*******************设置底盘速度*************************
    void SetVelocity(int16_t _forwardBackVelocity, int16_t _leftRightVelocity, int16_t _rotateVelocity = 0);

    inline void SetLRVelocity(){leftRightVelocity=0;}

    inline void SetFBVelocity(){forwardBackVelocity=0;}

    inline void SetRotateVelocity(int16_t _rotateVelocity) { rotateVelocity = _rotateVelocity; }

    void AddVelocity(int16_t accFB, int16_t accLR);
    //*******************设置底盘速度*************************

    //inline void SetChassisLock(uint8_t value) { chassisLock = value; }

    //inline uint8_t GetChassisLock() { return chassisLock; }

    inline void SetDashFlag(uint8_t state) { Chassis::chassis.dashFlag = state; }

private:
    float forwardBackVelocity;
    float leftRightVelocity;
    float rotateVelocity;
    PowerLimitation_t powerLimPara;
    PID rotate;

    int8_t twistGapAngle;
    int16_t followCenter;
    const int16_t *headDirection;
    float angleToCenter;
    float followDirection;
    float fakeBuffer;

    void ControlRotate();

    void ChassisDataDecoding();

    void TwistSpin();

    void PowerLimitation();
};

#endif
