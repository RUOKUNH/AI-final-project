/**
  ******************************************************************************
  * @FileName			    includes.h
  * @Description            include all the .h file we need
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#define RC_UART                 huart1
#define JUDGE_UART              huart6
#define AUTOAIM_UART            huart3
#define SERVO_UART				huart7

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
#define NORMALIZE_ANGLE180(angle) {\
    angle=angle>180?angle-360:(angle<-180?angle+360:angle);\
}

#define freq_div(func, times)\
{\
    static uint16_t cnt = 0;\
    if(cnt == 0)\
    {\
        func;\
    }\
    if(cnt++ >= times)\
    {\
        cnt = 0;\
    }\
}
                                     //如果button由0变为1，则执行button，由1变为0则没用
#define OnePush(button, execution)\
{\
    static uint8_t cache;\
    static uint8_t cnt=0;\
    if(cache != (button)){\
        cache = (button);\
        cnt = 0;\
    }\
    else if(cnt == 5){\
        if(cache) execution;\
        cnt=11;\
    }\
    else if(cnt < 5) cnt++;\
}
#define RECV_LEN(UART) (UART.RxXferSize - UART.hdmarx->Instance->NDTR)

#define ENABLE_DC24V_2()    HAL_GPIO_WritePin(DC24V_2_GPIO_Port, DC24V_2_Pin, GPIO_PIN_SET)
#define ENABLE_DC24V_3()    HAL_GPIO_WritePin(DC24V_3_GPIO_Port, DC24V_3_Pin, GPIO_PIN_SET)
#define ENABLE_DC24V_4()    HAL_GPIO_WritePin(DC24V_4_GPIO_Port, DC24V_4_Pin, GPIO_PIN_SET)
#define ENABLE_DC24V_5()    HAL_GPIO_WritePin(DC24V_5_GPIO_Port, DC24V_5_Pin, GPIO_PIN_SET)


#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "adc.h"
#include "math.h"
#include "core_cm4.h"
#include "dac.h"

#include "BoardConfiguration.h"
#include "RobotConfiguration.h"

#include "CallbackAdministration.h"

#include "Parameter.h"
#include "AutoAimTask.h"
#include "JudgeTask.h"
#include "RemoteTask.h"
#include "CANTask.h"
#include "PIDTask.h"
#include "MotorTask.h"
#include "GimbalTask.h"
#include "IMUTask.h"
#include "CarTask.h"
#include "ChassisTask.h"
#include "ShootTask.h"
#include "GateTask.h"

#include "drivers_ramp.h"
#include "pid_regulator.h"

#include "CapTask.h"
#include "CANDevice.h"

#include "LED.h"
#include "KeyMonitor.h"

#include "SoftTimer.h"

#include "Lib_songs.h"
#include "DanceWithBGM.h"

#include "ClimbTask.h"

#endif


