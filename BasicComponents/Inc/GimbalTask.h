/**
  ******************************************************************************
  * @FileName			    GimbalTask.h
  * @Description            Control the gimbal motor
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "includes.h"

#ifdef INFANTRY
#define YAW_DIR -1
#define PITCH_DIR -1

#endif
#ifdef HERO
#define YAW_DIR 1
#define PITCH_DIR -1
#define GM_PITCH_ZERO  7020
#define GM_YAW_ZERO  2050
#endif

#define GIMBAL_SPEED_K                  0.000288f

enum GM_e {
    GMPitch = 0,
    GMYaw
};

class Gimbal {
private:
    GMPitchMotor GMP;
    GMYawMotor GMY;

public:
    static Gimbal gimbal;

    void Handle();

    void Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback,
               const float *_GMPAngleFeedback, gimbalPara_t gimbalPara);

//    void Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback,
//          const float *_GMPAngleFeedback,
//          CanType_e GMPcan, uint16_t _GMPRxID, float _GMPreductionRate,
//          float _GMPkp, float _GMPki, float _GMPkd, float _GMPpMax, float _GMPiMax, float _GMPdMax,
//          float _GMPmax, float _GMPanglekp, float _GMPangleki,
//          float _GMPanglekd, float _GMPanglepMax, float _GMPangleiMax, float _GMPangledMax, float _GMPanglemax,
//
//          CanType_e GMYcan, uint16_t _GMYRxID, float _GMYreductionRate,
//          float _GMYkp, float _GMYki, float _GMYkd, float _GMYpMax, float _GMYiMax, float _GMYdMax,
//          float _GMYmax, float _GMYanglekp, float _GMYangleki,
//          float _GMYanglekd, float _GMYanglepMax, float _GMYangleiMax, float _GMYangledMax, float _GMYanglemax);
//    //设置云台反馈来源
//    void Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback,
//          const float *_GMPAngleFeedback);


    //使用变结构体PID
//    void Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback,
//               const float *_GMPAngleFeedback,
//               CanType_e GMYcan, uint16_t _GMYRxID, float _GMYreductionRate,
//               float _GMYkpA, float _GMYkpB, float _GMYkpC,
//               float _GMYkiA, float _GMYkiC, float _GMYkiK0,
//               float _GMYkdA, float _GMYkdB, float _GMYkdC,
//               float _GMYpMax,float _GMYiMax, float _GMYdMax,float _GMYmax,
//               float _GMYanglekpA, float _GMYanglekpB, float _GMYanglekpC,
//               float _GMYanglekiA, float _GMYanglekiC, float _GMYanglekiK0,
//               float _GMYanglekdA, float _GMYanglekdB, float _GMYanglekdC,
//               float _GMYanglepMax, float _GMYangleiMax, float _GMYangledMax,float _GMYanglemax,
//
//               CanType_e GMPcan, uint16_t _GMPRxID, float _GMPreductionRate,
//               float _GMPkpA, float _GMPkpB, float _GMPkpC,
//               float _GMPkiA, float _GMPkiC, float _GMPkiK0,
//               float _GMPkdA, float _GMPkdB, float _GMPkdC,
//               float _GMPpMax,float _GMPiMax, float _GMPdMax,float _GMPmax,
//               float _GMPanglekpA, float _GMPanglekpB, float _GMPanglekpC,
//               float _GMPanglekiA, float _GMPanglekiC, float _GMPanglekiK0,
//               float _GMPanglekdA, float _GMPanglekdB, float _GMPanglekdC,
//               float _GMPanglepMax, float _GMPangleiMax, float _GMPangledMax,float _GMPanglemax);

//**************************云台角度参数读写接口************************
    //设置云台位置
    inline void SetPosition(float pitchPos, float yawPos, float pitchK, float yawK) {
        GMP.targetAngle += PITCH_DIR * pitchPos * pitchK;
        GMY.targetAngle += YAW_DIR * yawPos * yawK;
    }

    //增量式设置云台位置，自瞄用
    inline void AutoAimSetPosition(float pitchPos, float yawPos, float pitch_v, float yaw_v) {
        GMP.targetAngle = GMP.GetRealAngle() + pitchPos;
        GMY.targetAngle = GMY.GetRealAngle() + yawPos;
        GMP.autoAimPitch_v = pitch_v;
        GMY.autoAimYaw_v = yaw_v;
    }

    //增量式设置云台角度，云台角度使用xms前的数据
    inline void
    AutoAimSetPosition(float GMPLastAngle, float GMYawLastAngle, float pitchPos, float yawPos, float pitch_v,
                       float yaw_v) {
        GMP.targetAngle = GMPLastAngle + pitchPos;
        GMY.targetAngle = GMYawLastAngle + yawPos;
        GMP.autoAimPitch_v = pitch_v;
        GMY.autoAimYaw_v = yaw_v;
    }

    //读取云台角度
    inline float GMRealAngle(GM_e GM) {
        if (GM == GMPitch) return GMP.GetRealAngle();
        else return GMY.GetRealAngle();
    }

    //设定单个云台电机角度
    inline void SetPosition(float pitchPos) { GMP.targetAngle += PITCH_DIR * pitchPos * GIMBAL_SPEED_K; }

    //获取云台电机角度所在的地址
    inline const int16_t *GetpMotorAngle(GM_e GM) {
        if (GM == GMPitch) return GMP.GetpAngle();
        else return GMY.GetpAngle();
    }
    //**************************云台角度参数读写接口************************
};

#endif
