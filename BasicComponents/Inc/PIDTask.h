/**
  ******************************************************************************
  * @FileName			    PIDTask.h
  * @Description            PID
  * @author                 Chang Liu
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __PID__H__
#define __PID__H__

#include "includes.h"

typedef struct PID_Regulator_t {
    float ref;
    float fdb;
    float err[4];
    float errSum;
    float kp;
    float ki;
    float kd;
    float componentKp;
    float componentKi;
    float componentKd;
    float componentKpMax;
    float componentKiMax;
    float componentKdMax;
    float output;
    float outputMax;

    float kpA;
    float kpB;
    float kpC;
    float kiA;
    float kiC;
    float kiK0;
    float kiK1;
    float ki0;
    float kiErrLimit = 10;
    float kdA;
    float kdB;
    float kdC;
} PID_Regulator_t;

typedef struct PID_Monitor_t {
    float* ref;
    float* fdb;

    float* componentKp;
    float* componentKi;
    float* componentKd;
} PID_Monitor_t;

class PID {
    uint8_t flagForMonitoringProcessQuantity;
    PID_Regulator_t PIDInfo;
    PID_Monitor_t PIDMoni;
public:

    float Calc(float target, float feedback);

    float CalcChange(float target, float feedback);

    void Reset(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max);

    void Reset();

    void ResetChange(float _kpA, float _kpB, float _kpC,
                     float _kiA, float _kiC, float _kiK0, float _kdA, float _kdB, float _kdC,
                     float _pMax, float _iMax, float _dMax, float _max);

    /*!
     * 用于PID参数热修改
     * @param _kp
     * @param _ki
     * @param _kd
     * @param _pMax
     * @param _iMax
     * @param _dMax
     * @param _max
     */
    void RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max);

    /*!
     * 调用后置位相应标志位，持续将过程量复制到传入的地址处
     * @param _ref
     * @param _fed
     * @param _componentKp
     * @param _componentKi
     * @param _componentKd
     */
    void EnableMonitoringProcessQuantity(float *_ref, float *_fdb, float *_componentKp, float *_componentKi,
                                         float *_componentKd);

    inline void setOutputMax(float outputMax) {
        PIDInfo.outputMax = outputMax;
    }

    inline float getOutputMax() const {
        return PIDInfo.outputMax;
    }

    inline float getKp() const {
        return PIDInfo.kp;
    }

};

#endif
