/**
  ******************************************************************************
  * @FileName			    MotorTask.h
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

#ifndef __MOTORTASK_H__
#define __MOTORTASK_H__

#include "includes.h"

#define GMP_Friction_compensation 0
#define GMY_Friction_compensation 0

#define GM_PITCH_GRAVITY_COMPENSATION 0    //英雄用



#define GM_YAW_GRAVITY_COMPENSATION 0


typedef struct {
    int16_t angle;//角度
    int16_t rotateSpeed;//转速
    int16_t moment;//扭矩
} ESCC6x0RxMsg_t;//电调解码器发送的数据 用户可以直接读出数据进行处理。

class Motor {
public:
    static Motor *motors[2][8];
    uint8_t everRx;
    float targetAngle;
    float targetSpeed;

    static void motorInit();

    void Reset(chassisMotorPara_t chassisMotorPara);

    void Reset(normalMotorPara_t normalMotorPara);

//    void
//    Reset(CanType_e can, uint16_t _RxID, float _reductionRate, float _kp, float _ki, float _kd, float _pMax,
//          float _iMax, float _dMax,
//          float _max);

    //使用变结构体PID
    void Reset(CanType_e can, uint16_t _RxID, float _reductionRate,
               float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
               float _kdA, float _kdB, float _kdC, float _pMax, float _iMax, float _dMax, float _max);

    void StopReset();

    inline ESCC6x0RxMsg_t GetRxMsgC6x0() const { return RxMsgC6x0; }

    inline const int16_t *GetpAngle() const { return &(RxMsgC6x0.angle); }

    inline const PID &GetSpeedPID() const { return speedPid; }

    inline int16_t GetIntensity() const { return intensity; }

    inline float GetRealAngle() const { return realAngle; }

    inline static uint8_t GetMotorConfliction() { return motorConliction; }

    inline void RxFromCan(const uint8_t *data) {
        RxMsgC6x0.angle = CanRxGetU16(data, 0);
        RxMsgC6x0.rotateSpeed = CanRxGetU16(data, 1);
        RxMsgC6x0.moment = CanRxGetU16(data, 2);
    }

    virtual void Handle() {}

protected:
    PID speedPid; //速度环PID，所有类型的电机都有。
    PID anglePid; //角度环PID，只有用双环PID的电机才有，例如GMYawMotor，NormalMotor等。
    uint16_t RxID; //can通讯相关参数，和电调ID(设其为i)之间满足以下映射关系：
    //RxID = 0x200 + i;
    int16_t intensity;//通过can通讯发送给电机的电流强度。也就是所谓PID输出的控制量。
    int16_t intensityThisTime;//此次电流的计算值(未滤波)
    int16_t intensityLastTime;//上一次电流的输出值
    ESCC6x0RxMsg_t RxMsgC6x0;//前文提过，不再赘述。
    uint8_t firstEnter;//是否第一次进入？电机是否做初始化工作的判据。
    uint8_t reseted;//用于判断云台电机是否回归零点。
    float reductionRate;//电机减速比
    float lastRead;//辅助计算realAngle，具体请看相关代码，此处不赘述。
    float realAngle;//真实角度
    static uint8_t motorConliction;
};

class ChassisMotor : public Motor {

public:
    void Reset(chassisMotorPara_t chassisMotorPara);

    //使用变结构体PID
    void Reset(CanType_e can, uint16_t _RxID, float _reductionRate,
               float _kpA, float _kpB, float _kpC, float _kiA, float _kiC, float _kiK0,
               float _kdA, float _kdB, float _kdC, float _pMax, float _iMax, float _dMax, float _max);

    void RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max);

    void Handle() override;
//    void
//    Reset_two(CanType_e can, uint16_t _RxID, float _reductionRate, float _kp, float _ki, float _kd, float _pMax,
//              float _iMax, float _dMax,
//              float _max);
    /*！
     * 用于cubemonitor修改PID
     */
};


class NormalMotor : public Motor {
public:

    void Reset(normalMotorPara_t normalMotorPara);
//
//    void
//    Reset(CanType_e can, uint16_t _RxID, float _reductionRate, float _kp, float _ki, float _kd, float _pMax,
//          float _iMax, float _dMax,
//          float _max,
//          float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax, float _angledMax,
//          float _anglemax) ;

    /*！
   * 用于cubemonitor修改PID
   */
    void RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                     float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                     float _angledMax,
                     float _anglemax);

    void Handle() override;

private:
    float thisSpeed;
    float thisAngle;
};

class GMPitchMotor : public Motor {
public:
    float autoAimPitch_v;

    void Reset(const float *_speedFeedback, const float *_angleFeedback, normalMotorPara_t gmpPara);

    void Reset(normalMotorPara_t gmpPara);

//    void Reset(const float *_speedFeedback, const float *_angleFeedback);

    /*！
     * 用于cubemonitor修改PID
     */
    void RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                     float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                     float _angledMax,
                     float _anglemax);

    //使用变结构体PID
    void
    Reset(const float *_speedFeedback, const float *_angleFeedback, CanType_e can, uint16_t _RxID, float _reductionRate,
          float _kpA, float _kpB, float _kpC,
          float _kiA, float _kiC, float _kiK0,
          float _kdA, float _kdB, float _kdC,
          float _pMax, float _iMax, float _dMax, float _max,
          float _anglekpA, float _anglekpB, float _anglekpC,
          float _anglekiA, float _anglekiC, float _anglekiK0,
          float _anglekdA, float _anglekdB, float _anglekdC,
          float _anglepMax, float _angleiMax, float _angledMax, float _anglemax);

    void Handle();

private:
    float thisSpeed;
    float thisAngle;
    const float *speedFeedback;
    const float *angleFeedback;
};

class GMYawMotor : public Motor {
public:
    float autoAimYaw_v;

    void Reset(const float *_speedFeedback, const float *_angleFeedback, normalMotorPara_t gmyPara);

    void Reset(normalMotorPara_t gmyPara);

//    void Reset(const float *_speedFeedback, const float *_angleFeedback);

//    void
//    Reset(const float *_speedFeedback, const float *_angleFeedback, CanType_e can, uint16_t _RxID,
//          float _reductionRate,
//          float _kp, float _ki, float _kd, float _pMax,
//          float _iMax, float _dMax,
//          float _max,
//          float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax, float _angledMax,
//          float _anglemax) ;

    /*！
     * 用于cubemonitor修改PID
     */
    void RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max,
                     float _anglekp, float _angleki, float _anglekd, float _anglepMax, float _angleiMax,
                     float _angledMax,
                     float _anglemax);


    //使用变结构体PID
    void
    Reset(const float *_speedFeedback, const float *_angleFeedback, CanType_e can, uint16_t _RxID, float _reductionRate,
          float _kpA, float _kpB, float _kpC,
          float _kiA, float _kiC, float _kiK0,
          float _kdA, float _kdB, float _kdC,
          float _pMax, float _iMax, float _dMax, float _max,
          float _anglekpA, float _anglekpB, float _anglekpC,
          float _anglekiA, float _anglekiC, float _anglekiK0,
          float _anglekdA, float _anglekdB, float _anglekdC,
          float _anglepMax, float _angleiMax, float _angledMax, float _anglemax);


    void Handle() override;

private:
    float thisSpeed;
    float thisAngle;
    const float *speedFeedback;
    const float *angleFeedback;
};

#endif
