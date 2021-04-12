/**
  ******************************************************************************
  * @FileName			    CarTask.cpp
  * @Description            This is a robot, not only a car
  * @author                 Chang Liu & Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include <CarTask.h>

#include "includes.h"


void CarInit(void) {
    Car::car.Reset();
}

void MainControlLoop(void) {
    refreshJudgeState();
    Car::car.WorkStateFSM();
#ifdef _IMU
    IMU::BSP_IMU.Handle();
#endif
#ifdef _AUTOAIM
    AutoAim::autoAim.Handle();
#endif
#ifdef _CAP
    Cap::cap.Handle();
#endif
    if (Car::car.GetWorkState() > 0) {
#ifdef _CHASSIS
        Chassis::chassis.Handle();
#endif
#ifdef _GIMBAL
        Gimbal::gimbal.Handle();
#endif
#ifdef _FRIC
        Shoot::shoot.Handle();
#endif
#ifdef DARTS_SHOOT
        Shoot::shoot.Handle();
#endif
    }

    CAN::can1.TxHandle();
    CAN::can2.TxHandle();

    soft_timer_schedule(&timer_song_1ms);
#ifdef TEST_MOTOR
#ifdef USE_CUBE_MONITOR
    Chassis::chassis.testmotor.RefreshPara(testMotorPara.kp,testMotorPara.ki,testMotorPara.kd,testMotorPara.pMax,\
        testMotorPara.iMax,testMotorPara.dMax,testMotorPara.max);
    refreshPara(monitorPara_testMotor,&Chassis::chassis.testmotor.targetAngle,Chassis::chassis.testmotor.GetRealAngle(),0,Chassis::chassis.testmotor.GetIntensity());
#endif

Chassis::chassis.testmotor.Handle();
monitorPara.watch1[0] = Chassis::chassis.testmotor.targetAngle;
monitorPara.watch1[1] = Chassis::chassis.testmotor.GetRealAngle();

testMotorPara.kpA = monitorPara.watch1[2];
testMotorPara.kpB = monitorPara.watch1[3];
testMotorPara.kpC = monitorPara.watch1[4];
#endif
    Referee_Transmit();
}

void Count() {
    #ifdef _FRIC
    Shoot::shoot.Count();
    #endif
}

void Car::Reset(void) {
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
    Motor::motorInit();
    setPara();
    #ifdef _CHASSIS
    #ifdef INFANTRY
    Chassis::chassis.Reset(Gimbal::gimbal.GetpMotorAngle(GMYaw),infantryPara.chassisPara);
    #endif
    #ifdef HERO
    Chassis::chassis.Reset(Gimbal::gimbal.GetpMotorAngle(GMYaw),heroPara.chassisPara);
    #endif
    #endif

    //使用变结构体PID
    /*Chassis::chassis.Reset(Gimbal::gimbal.GetpMotorAngle(GMYaw), chassisPara.followDirection,
                           chassisPara.FL.CAN, chassisPara.FL.RxID, chassisPara.FR.CAN, chassisPara.FR.RxID,
                           chassisPara.BL.CAN, chassisPara.BL.RxID, chassisPara.BR.CAN, chassisPara.BR.RxID, chassisPara.reductionRate,
                           chassisPara.kpA, chassisPara.kpB, chassisPara.kpC,
                           chassisPara.kiA, chassisPara.kiC, chassisPara.kiK0,
                           chassisPara.kdA, chassisPara.kdB, chassisPara.kdC,
                           chassisPara.pMax, chassisPara.iMax, chassisPara.dMax, chassisPara.max,

                           chassisPara.FollowkpA, chassisPara.FollowkpB, chassisPara.FollowkpC,
                           chassisPara.FollowkiA, chassisPara.FollowkiC, chassisPara.FollowkiK0,
                           chassisPara.FollowkdA, chassisPara.FollowkdB, chassisPara.FollowkdC,
                           chassisPara.FollowpMax, chassisPara.FollowiMax, chassisPara.FollowdMax, chassisPara.FollowMax);*/

    #ifdef _GIMBAL
    #ifdef HERO
    Gimbal::gimbal.Reset(IMU::BSP_IMU.GetpData(negative_wz),IMU::BSP_IMU.GetpData(negative_yaw),
                         IMU::BSP_IMU.GetpData(wx),IMU::BSP_IMU.GetpData(rol),
                         imu,heroPara.gimbalPara);
    #endif
    #ifdef INFANTRY
    Gimbal::gimbal.Reset(IMU::BSP_IMU.GetpData(wz),IMU::BSP_IMU.GetpData(yaw),
                         IMU::BSP_IMU.GetpData(negative_wy),IMU::BSP_IMU.GetpData(negative_pit),
                         infantryPara.gimbalPara);

    /*Gimbal::gimbal.Reset( IMU::BSP_IMU.GetpData(wz),IMU::BSP_IMU.GetpData(yaw),IMU::BSP_IMU.GetpData(negative_wy),IMU::BSP_IMU.GetpData(negative_pit),
                          gimbalPara.GMP.CAN, gimbalPara.GMP.RxID, gimbalPara.GMP.reductionRate,
                          gimbalPara.GMP.kp, gimbalPara.GMP.ki, gimbalPara.GMP.kd, gimbalPara.GMP.pMax, gimbalPara.GMP.iMax, gimbalPara.GMP.dMax, gimbalPara.GMP.max,
                          gimbalPara.GMP.anglekp, gimbalPara.GMP.angleki, gimbalPara.GMP.anglekd, gimbalPara.GMP.anglepMax, gimbalPara.GMP.angleiMax, gimbalPara.GMP.angledMax, gimbalPara.GMP.anglemax,
                          gimbalPara.GMY.CAN, gimbalPara.GMY.RxID, gimbalPara.GMY.reductionRate,
                          gimbalPara.GMY.kp, gimbalPara.GMY.ki, gimbalPara.GMY.kd, gimbalPara.GMY.pMax, gimbalPara.GMY.iMax, gimbalPara.GMY.dMax, gimbalPara.GMY.max,
                          gimbalPara.GMY.anglekp, gimbalPara.GMY.angleki, gimbalPara.GMY.anglekd, gimbalPara.GMY.anglepMax, gimbalPara.GMY.angleiMax, gimbalPara.GMY.angledMax, gimbalPara.GMY.anglemax);
*/
    //使用变结构体PID
    /*Gimbal::gimbal.Reset( IMU::BSP_IMU.GetpData(wz),IMU::BSP_IMU.GetpData(yaw),
                          IMU::BSP_IMU.GetpData(negative_wy),IMU::BSP_IMU.GetpData(negative_pit),
                          gimbalPara.GMY.CAN, gimbalPara.GMY.RxID, gimbalPara.GMY.reductionRate,
                          gimbalPara.GMY.kpA,gimbalPara.GMY.kpB,gimbalPara.GMY.kpC,
                          gimbalPara.GMY.kiA,gimbalPara.GMY.kiC,gimbalPara.GMY.kiK0,
                          gimbalPara.GMY.kdA, gimbalPara.GMY.kdB,gimbalPara.GMY.kdC,
                          gimbalPara.GMY.pMax, gimbalPara.GMY.iMax, gimbalPara.GMY.dMax, gimbalPara.GMY.max,
                          gimbalPara.GMY.anglekpA,gimbalPara.GMY.anglekpB, gimbalPara.GMY.anglekpC,
                          gimbalPara.GMY.anglekiA,gimbalPara.GMY.anglekiC, gimbalPara.GMY.anglekiK0,
                          gimbalPara.GMY.anglekdA,gimbalPara.GMY.anglekdB, gimbalPara.GMY.anglekdC,
                          gimbalPara.GMY.anglepMax, gimbalPara.GMY.angleiMax, gimbalPara.GMY.angledMax, gimbalPara.GMY.anglemax,

                          gimbalPara.GMP.CAN, gimbalPara.GMP.RxID, gimbalPara.GMP.reductionRate,
                          gimbalPara.GMP.kpA,gimbalPara.GMP.kpB,gimbalPara.GMP.kpC,
                          gimbalPara.GMP.kiA,gimbalPara.GMP.kiC,gimbalPara.GMP.kiK0,
                          gimbalPara.GMP.kdA, gimbalPara.GMP.kdB,gimbalPara.GMP.kdC,
                          gimbalPara.GMP.pMax, gimbalPara.GMP.iMax, gimbalPara.GMP.dMax, gimbalPara.GMP.max,
                          gimbalPara.GMP.anglekpA,gimbalPara.GMP.anglekpB, gimbalPara.GMP.anglekpC,
                          gimbalPara.GMP.anglekiA,gimbalPara.GMP.anglekiC, gimbalPara.GMP.anglekiK0,
                          gimbalPara.GMP.anglekdA,gimbalPara.GMP.anglekdB, gimbalPara.GMP.anglekdC,
                          gimbalPara.GMP.anglepMax, gimbalPara.GMP.angleiMax, gimbalPara.GMP.angledMax, gimbalPara.GMP.anglemax);*/

    #endif
    #endif

    #ifdef _FRIC
        #ifdef HERO
            Shoot::shoot.Reset(heroPara.shootPara);
        #endif
        #ifdef INFANTRY
            Shoot::shoot.Reset(infantryPara.shootPara);
        #endif
    #endif
    #ifdef DARTS_SHOOT
        Shoot::shoot.ResetDart();
    #endif
    #ifdef BOARD_MASTER
        CAN::can2.Reset(CAN_TYPE_2, 1, 1);
        CAN::can1.Reset(CAN_TYPE_1, 1, 1);
        Remote::remote.Reset();
    #endif
    #ifdef BOARD_SLAVE
        CAN::can2.Reset(CAN_TYPE_2, 0, 0);
        CAN::can1.Reset(CAN_TYPE_1, 1,1);
        Remote::remote.Reset();
    #endif
    CANDevice::CANdevice.Reset();
    #ifdef _IMU
    IMU::BSP_IMU.Reset();
    #endif
    #ifdef _AUTOAIM
    AutoAim::autoAim.Reset();
    #endif
    #ifdef BOARD_MASTER
    #ifndef _NCOMPLETE_TEST
            MX_IWDG_Init();
    #endif
        InitJudgeUart();
    #endif
    #ifdef _CAP
       Cap::cap.Reset();
    #endif
#ifdef TEST_MOTOR
       Chassis::chassis.testmotor.Reset(testMotorPara.CAN,testMotorPara.RxID,

    #ifdef VARIABLE_STRUCTRUE_PID
       Chassis::chassis.testmotor.Reset(testMotorPara.testMotor.CAN,testMotorPara.testMotor.RxID,testMotorPara.reductionRate,
                                        testMotorPara.kpA,testMotorPara.kpB,testMotorPara.kpC,
                                        testMotorPara.kiA,testMotorPara.kiC,testMotorPara.kiK0,
                                        testMotorPara.kdA,testMotorPara.kdB,testMotorPara.kdC,
                                        testMotorPara.pMax,testMotorPara.iMax,testMotorPara.dMax,testMotorPara.max);
    #else
       Chassis::chassis.testmotor.Reset(testMotorPara.testMotor.CAN,testMotorPara.testMotor.RxID,
                                        testMotorPara.reductionRate,testMotorPara.kp,testMotorPara.ki,
                                        testMotorPara.kd,testMotorPara.pMax,testMotorPara.iMax,
                                        testMotorPara.dMax,testMotorPara.max);
    #endif
    Chassis::chassis.testmotor.targetAngle = 100;
#endif
    workState = PREPARE_STATE;
    Gate::gate.Reset();
    //Music_init(&timer_song_1ms, &Pureland, (task_s**)0);
}

void Car::WorkStateFSM() {
#ifdef BOARD_MASTER
    static uint16_t prepareTime = 0;
    static uint16_t normalTimeUn = 0;
    static uint16_t normalTimeDeux = 0;
    static uint16_t stateOneTimeUn = 0;
    static uint16_t stateOneTimeDeux = 0;
    static uint16_t stateTwoTimeUn = 0;
    static uint16_t stateTwoTimeDeux = 0;
    static uint8_t allMotorsRx = 0;
    static uint16_t GPIO_PINs[8] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4,
                                    GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8};
#ifndef _NCOMPLETE_TEST
    if (allMotorsRx == 0) {
        allMotorsRx = 1;
        for (uint8_t j = 0; j < 8; j++) {
            HAL_GPIO_WritePin(GPIOG, GPIO_PINs[j], GPIO_PIN_RESET);
        }
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
        //电机离线过不了自检
        for (uint8_t i = 0; i < 2; i++) {
            for (uint8_t j = 0; j < 8; j++) {
                if (Motor::motors[i][j] != nullptr && Motor::motors[i][j]->everRx != 1) {
                    allMotorsRx = 0;
                    HAL_GPIO_WritePin(GPIOG, GPIO_PINs[j], GPIO_PIN_SET);//灭开发板左边对应数字的灯
                    if (i == 0) {
                        //开发板右红灯灭 can1出现问题
                        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
                    } else if (i == 1) {
                        //开发板右绿灯灭 can2出现问题
                        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
                    }
                }
            }
        }
    }
#else
    allMotorsRx = 1;
#endif
    switch (workState) {
        static uint32_t counterForStopMode=0;
        case PREPARE_STATE: {
            counterForStopMode = 0;//每次退出stop模式必经prepare，在此重置
            if (Remote::remote.GetInputMode() == STOP) workState = STOP_STATE;
            //进入normal state的条件
            if (prepareTime < 500 && IMU::BSP_IMU.GetData().InitFinish == 1 && allMotorsRx == 1 &&
                Motor::GetMotorConfliction() == 0)
                prepareTime++;//&& allMotorsRx == 1
            if (prepareTime >= 500) {
                workState = NORMAL_STATE;
                prepareTime = 0;
            }
        }
            break;
        case NORMAL_STATE: {
            if (Remote::remote.GetInputMode() == STOP) workState = STOP_STATE;

            /*if (Remote::remote.GetFunctionMode() == MIDDLE_POS)workState = ADDITIONAL_STATE_ONE;
            if (Remote::remote.GetFunctionMode() == LOWER_POS)workState = ADDITIONAL_STATE_ONE;*/
            if (Remote::remote.GetFunctionMode() == MIDDLE_POS) normalTimeUn++;
            if (normalTimeUn >= 50) {
                workState = ADDITIONAL_STATE_ONE;
                normalTimeUn = 0;
                normalTimeDeux = 0;
            }

            if (Remote::remote.GetFunctionMode() == LOWER_POS) normalTimeDeux++;
            if (normalTimeDeux >= 50) {
                workState = ADDITIONAL_STATE_TWO;
                normalTimeUn = 0;
                normalTimeDeux = 0;
            }
        }
            break;
        case ADDITIONAL_STATE_ONE: {
            if (Remote::remote.GetInputMode() == STOP) workState = STOP_STATE;

            /*if (Remote::remote.GetFunctionMode() == UPPER_POS)workState = NORMAL_STATE;
            if (Remote::remote.GetFunctionMode() == LOWER_POS)workState = ADDITIONAL_STATE_TWO;*/
            if (Remote::remote.GetFunctionMode() == UPPER_POS) stateOneTimeUn++;
            if (stateOneTimeUn >= 50) {
                workState = NORMAL_STATE;
                stateOneTimeUn = 0;
                stateOneTimeDeux = 0;
            }

            if (Remote::remote.GetFunctionMode() == LOWER_POS) stateOneTimeDeux++;
            if (stateOneTimeDeux >= 50) {
                workState = ADDITIONAL_STATE_TWO;
                stateOneTimeUn = 0;
                stateOneTimeDeux = 0;
            }
        }
            break;
        case ADDITIONAL_STATE_TWO: {
            if (Remote::remote.GetInputMode() == STOP) workState = STOP_STATE;

            /*if (Remote::remote.GetFunctionMode() == UPPER_POS)workState = NORMAL_STATE;
            if (Remote::remote.GetFunctionMode() == MIDDLE_POS)workState = ADDITIONAL_STATE_ONE;*/
            if (Remote::remote.GetFunctionMode() == UPPER_POS) stateTwoTimeUn++;
            if (stateTwoTimeUn >= 50) {
                workState = NORMAL_STATE;
                stateTwoTimeUn = 0;
                stateTwoTimeDeux = 0;
            }

            if (Remote::remote.GetFunctionMode() == MIDDLE_POS) stateTwoTimeDeux++;
            if (stateTwoTimeDeux >= 50) {
                workState = ADDITIONAL_STATE_ONE;
                stateTwoTimeUn = 0;
                stateTwoTimeDeux = 0;
            }
        }
            break;
        case STOP_STATE: {
            if (Remote::remote.GetInputMode() == REMOTE_INPUT || Remote::remote.GetInputMode() == KEY_MOUSE_INPUT) {
                workState = PREPARE_STATE;
            }
            Shoot::shoot.RefreshAngleGap();

            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 8; j++) {
                    if (Motor::motors[i][j] != nullptr) {
                   //     if (counterForStopMode++ > 1000) {
                            Motor::motors[i][j]->StopReset();
//                        } else if (counterForStopMode <= 1000) {
//                            Motor::motors[i][j]->targetSpeed = 0;
//                            Motor::motors[i][j]->Handle();
//                        }
                    }
                }
            }
            break;
        default:
            break;
    }
#else

#endif
}
}

Car Car::car;