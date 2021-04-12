/**
  ******************************************************************************
  * @FileName			    RobotConfiguration.h
  * @Description            Contains robot's basic information
  * @author                 Xi Wang
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __ROBOTCONFIGURATION_H
#define __ROBOTCONFIGURATION_H

#define INFANTRY
//#define HERO
//#define ENGINEER
//#define GUARD
//#define AIRCRAFT
//#define DART

#define BOARD_MASTER
//#define BOARD_SLAVE
//#define BOARD_SLAVE2

#ifdef INFANTRY
#ifdef BOARD_MASTER
    #define _CHASSIS
    #define _GIMBAL
    #define _FRIC
    #define _GATE_SERVO
    #define _IMU
    //#define _AHRS
    #define _AUTOAIM
    //#define _POWER_LIMITATION

    #define _CAP
//    #define _NCOMPLETE_TEST
    #define USE_CHASSIS_FOLLOW
    //#define TEST_MOTOR
    //#define USE_CUBE_MONITOR//使用后可以监视部分变量，同时启用部分参数热修正，实现动态调参
    //#define VARIABLE_STRUCTRUE_PID
    //#define SHOOT_SPEED_TEST
    //#define DARTS_SHOOT //飞镖发射架


//    #define INFANTRY_3
    #define INFANTRY_4
#endif
#endif

#ifdef HERO
    #define _CHASSIS
    #define _GIMBAL
    #define _FRIC

    #define _IMU
    //#define _AHRS
    #define _AUTOAIM
#define USE_CHASSIS_FOLLOW
#define _POWER_LIMITATION
#define _CAP
//#define _NCOMPLETE_TEST
#endif



#endif //__ROBOTCONFIGURATION_H
