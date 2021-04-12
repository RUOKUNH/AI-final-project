/**
  ******************************************************************************
  * @FileName			    PIDTask.cpp
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

#include "includes.h"

float PID::Calc(float target, float feedback) {
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    PIDInfo.componentKp = PIDInfo.err[3] * PIDInfo.kp;
    PIDInfo.errSum = PIDInfo.err[0]+PIDInfo.err[1]+PIDInfo.err[2]+PIDInfo.err[3];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);

    PIDInfo.err[0] = PIDInfo.err[1];
    PIDInfo.err[1] = PIDInfo.err[2];
    PIDInfo.err[2] = PIDInfo.err[3];
#ifdef USE_CUBE_MONITOR
    if (flagForMonitoringProcessQuantity == 1){
        *PIDMoni.ref=PIDInfo.ref;
        *PIDMoni.fdb=PIDInfo.fdb;
        *PIDMoni.componentKp=PIDInfo.componentKp;
        *PIDMoni.componentKi=PIDInfo.componentKi;
        *PIDMoni.componentKd=PIDInfo.componentKd;
    }
#endif
    return PIDInfo.output;
}

float PID::CalcChange(float target, float feedback){
    PIDInfo.fdb = feedback;
    PIDInfo.ref = target;
    PIDInfo.err[3] = PIDInfo.ref - PIDInfo.fdb;
    //计算P控制量: Kp (e) =ap+bp (1-exp (-cp|e|)
    PIDInfo.kp = PIDInfo.kpA + PIDInfo.kpB * (1 - exp(-PIDInfo.kpC * fabs(PIDInfo.err[1])));
    PIDInfo.componentKp = PIDInfo.err[1] * PIDInfo.kp;
    //计算I控制量: Ki (e) =Ki0 * aiexp (-ci|e|)
    PIDInfo.kiK1 = exp(-PIDInfo.kiK0 * PIDInfo.kiErrLimit);
    if (fabs(PIDInfo.err[1]) <= PIDInfo.kiErrLimit){
        PIDInfo.ki0 = PIDInfo.kiK1 * exp(-PIDInfo.kiK0 * fabs(PIDInfo.err[1]));
    }else{
        PIDInfo.ki0 = 1;
    }
    PIDInfo.ki =PIDInfo.ki0 * PIDInfo.kiA * exp(-PIDInfo.kiC * fabs(PIDInfo.err[1]));
    PIDInfo.errSum = PIDInfo.err[0]+PIDInfo.err[1]+PIDInfo.err[2]+PIDInfo.err[3];
    INRANGE(PIDInfo.errSum, -1 * PIDInfo.componentKiMax / PIDInfo.ki, PIDInfo.componentKiMax / PIDInfo.ki);
    PIDInfo.componentKi = PIDInfo.errSum * PIDInfo.ki;
    //计算D控制量：Kd (e) =ad-bd (1-exp (-cd|e|) )
    PIDInfo.kd = PIDInfo.kdA - PIDInfo.kdB * (1 - exp(-PIDInfo.kdC * fabs(PIDInfo.err[1])));
    PIDInfo.componentKd = (PIDInfo.err[3] - PIDInfo.err[2]) * PIDInfo.kd;
    INRANGE(PIDInfo.componentKp, -1 * PIDInfo.componentKpMax, PIDInfo.componentKpMax);
    INRANGE(PIDInfo.componentKi, -1 * PIDInfo.componentKiMax, PIDInfo.componentKiMax);
    INRANGE(PIDInfo.componentKd, -1 * PIDInfo.componentKdMax, PIDInfo.componentKdMax);
    PIDInfo.output = PIDInfo.componentKp + PIDInfo.componentKi + PIDInfo.componentKd;
    INRANGE(PIDInfo.output, -1 * PIDInfo.outputMax, PIDInfo.outputMax);

    PIDInfo.err[0] = PIDInfo.err[1];
    PIDInfo.err[1] = PIDInfo.err[2];
    PIDInfo.err[2] = PIDInfo.err[3];
    return PIDInfo.output;
}

void PID::Reset(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max) {
    PIDInfo.ref = 0;
    PIDInfo.fdb = 0;
    PIDInfo.err[0] = 0;
    PIDInfo.err[1] = 0;
    PIDInfo.err[2] = 0;
    PIDInfo.err[3] = 0;
    PIDInfo.componentKp = 0;
    PIDInfo.componentKi = 0;
    PIDInfo.componentKd = 0;
    PIDInfo.output = 0;
    PIDInfo.errSum = 0;
    PIDInfo.kp = _kp;
    PIDInfo.ki = _ki;
    PIDInfo.kd = _kd;
    PIDInfo.componentKpMax = _pMax;
    PIDInfo.componentKiMax = _iMax;
    PIDInfo.componentKdMax = _dMax;
    PIDInfo.outputMax = _max;
    flagForMonitoringProcessQuantity=0;
}

void PID::ResetChange(float _kpA, float _kpB, float _kpC,
                      float _kiA, float _kiC, float _kiK0, float _kdA, float _kdB, float _kdC,
                      float _pMax, float _iMax, float _dMax, float _max){
    PIDInfo.ref = 0;
    PIDInfo.fdb = 0;
    PIDInfo.err[0] = 0;
    PIDInfo.err[1] = 0;
    PIDInfo.componentKp = 0;
    PIDInfo.componentKi = 0;
    PIDInfo.componentKd = 0;
    PIDInfo.output = 0;
    PIDInfo.errSum = 0;
    PIDInfo.kpA = _kpA;
    PIDInfo.kpB = _kpB;
    PIDInfo.kpC = _kpC;
    PIDInfo.kiA = _kiA;
    PIDInfo.kiC = _kiC;
    PIDInfo.kiK0 = _kiK0;
    PIDInfo.kdA = _kdA;
    PIDInfo.kdB = _kdB;
    PIDInfo.kdC = _kdC;
    PIDInfo.componentKpMax = _pMax;
    PIDInfo.componentKiMax = _iMax;
    PIDInfo.componentKdMax = _dMax;
    PIDInfo.outputMax = _max;
}

void PID::Reset(){
    PIDInfo.ref = 0;
    PIDInfo.fdb = 0;
    PIDInfo.err[0] = 0;
    PIDInfo.err[1] = 0;
    PIDInfo.componentKp = 0;
    PIDInfo.componentKi = 0;
    PIDInfo.componentKd = 0;
    PIDInfo.output = 0;
    PIDInfo.errSum = 0;
    flagForMonitoringProcessQuantity=0;
}

void PID::RefreshPara(float _kp, float _ki, float _kd, float _pMax, float _iMax, float _dMax, float _max) {
    PIDInfo.kp = _kp;
    PIDInfo.ki = _ki;
    PIDInfo.kd = _kd;
    PIDInfo.componentKpMax = _pMax;
    PIDInfo.componentKiMax = _iMax;
    PIDInfo.componentKdMax = _dMax;
    PIDInfo.outputMax = _max;
}

void PID::EnableMonitoringProcessQuantity(float *_ref, float *_fdb, float *_componentKp, float *_componentKi,
                                          float *_componentKd) {
    flagForMonitoringProcessQuantity=1;
    PIDMoni.ref=_ref;
    PIDMoni.fdb=_fdb;
    PIDMoni.componentKp=_componentKp;
    PIDMoni.componentKi=_componentKi;
    PIDMoni.componentKd=_componentKd;
}
