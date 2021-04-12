/**
  ***************************************************************************
  * ***
  * @FileName			    GimbalTask.cpp
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

#include "includes.h"

void Gimbal::Handle() {
#ifdef USE_CUBE_MONITOR
    GMY.RefreshPara( infantryPara.gimbalPara.GMYPara.kp, infantryPara.gimbalPara.GMYPara.ki, infantryPara.gimbalPara.GMYPara.kd, infantryPara.gimbalPara.GMYPara.pMax, infantryPara.gimbalPara.GMYPara.iMax, infantryPara.gimbalPara.GMYPara.dMax, infantryPara.gimbalPara.GMYPara.max,
                     infantryPara.gimbalPara.GMYPara.anglekp, infantryPara.gimbalPara.GMYPara.angleki, infantryPara.gimbalPara.GMYPara.anglekd, infantryPara.gimbalPara.GMYPara.anglepMax, infantryPara.gimbalPara.GMYPara.angleiMax, infantryPara.gimbalPara.GMYPara.angledMax, infantryPara.gimbalPara.GMYPara.anglemax);
    GMY.RefreshPara( infantryPara.gimbalPara.GMYPara.kp, infantryPara.gimbalPara.GMYPara.ki, infantryPara.gimbalPara.GMYPara.kd, infantryPara.gimbalPara.GMYPara.pMax, infantryPara.gimbalPara.GMYPara.iMax, infantryPara.gimbalPara.GMYPara.dMax, infantryPara.gimbalPara.GMYPara.max,
                     infantryPara.gimbalPara.GMYPara.anglekp, infantryPara.gimbalPara.GMYPara.angleki, infantryPara.gimbalPara.GMYPara.anglekd, infantryPara.gimbalPara.GMYPara.anglepMax, infantryPara.gimbalPara.GMYPara.angleiMax, infantryPara.gimbalPara.GMYPara.angledMax, infantryPara.gimbalPara.GMYPara.anglemax);
#endif//当此部分启用时，修改infantryPara.gimbalPara的数据可以直接起作用
    GMY.Handle();
    GMP.Handle();
}

void Gimbal::Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback,
                   const float *_GMPAngleFeedback,gimbalPara_t gimbalPara)
{
    GMP.Reset(_GMPSpeedFeedback,_GMPAngleFeedback,gimbalPara.GMPPara);
    GMY.Reset(_GMYSpeedFeedback,_GMYAngleFeedback,gimbalPara.GMYPara);
}



//void Gimbal::Reset(const float *_GMYSpeedFeedback, const float *_GMYAngleFeedback, const float *_GMPSpeedFeedback, const float *_GMPAngleFeedback){
//    GMP.Reset(_GMPSpeedFeedback,_GMPAngleFeedback);
//    GMY.Reset(_GMYSpeedFeedback,_GMYAngleFeedback);
//}

Gimbal Gimbal::gimbal;
