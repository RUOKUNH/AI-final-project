/**
  ******************************************************************************
  * @FileName			    Parameter.cpp
  * @Description            set parameters frequently mofifyied
  *
  * @author                 David Hong
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/
#include "includes.h"

infantryPara_t infantryPara;
heroPara_t heroPara;
monitorPara_Motor_t monitorPara_testMotor;
monitorPara_Motor_t monitorPara_GYMotor;
monitorPara_imu_t monitorPara_imu;
monitorPara_t monitorPara;
chassisMotorPara_t testMotorPara;




float bullet_timer(0);
float rotate_timer(0);
#ifdef HERO

void setPara(){

    heroPara.chassisPara.followDirection=-2.0f;
    heroPara.chassisPara.FLPara.RxID =0x201;
    heroPara.chassisPara.FRPara.RxID =0x202;
    heroPara.chassisPara.BLPara.RxID =0x203;
    heroPara.chassisPara.BRPara.RxID =0x204;
    heroPara.chassisPara.FLPara.CAN=CAN_TYPE_2;
    heroPara.chassisPara.FRPara.CAN=CAN_TYPE_2;
    heroPara.chassisPara.BLPara.CAN=CAN_TYPE_2;
    heroPara.chassisPara.BRPara.CAN=CAN_TYPE_2;

    heroPara.chassisPara.FLPara.kp=16;
    heroPara.chassisPara.FLPara.ki=0;
    heroPara.chassisPara.FLPara.kd=2;
    heroPara.chassisPara.FLPara.pMax=20000;
    heroPara.chassisPara.FLPara.iMax=20000;
    heroPara.chassisPara.FLPara.dMax=20000;
    heroPara.chassisPara.FLPara.max=16384;
    heroPara.chassisPara.FRPara.kp=16;
    heroPara.chassisPara.FRPara.ki=0;
    heroPara.chassisPara.FRPara.kd=2;
    heroPara.chassisPara.FRPara.pMax=20000;
    heroPara.chassisPara.FRPara.iMax=20000;
    heroPara.chassisPara.FRPara.dMax=20000;
    heroPara.chassisPara.FRPara.max=16384;
    heroPara.chassisPara.BLPara.kp=16;
    heroPara.chassisPara.BLPara.ki=0;
    heroPara.chassisPara.BLPara.kd=2;
    heroPara.chassisPara.BLPara.pMax=20000;
    heroPara.chassisPara.BLPara.iMax=20000;
    heroPara.chassisPara.BLPara.dMax=20000;
    heroPara.chassisPara.BLPara.max=16384;
    heroPara.chassisPara.BRPara.kp=16;
    heroPara.chassisPara.BRPara.ki=0;
    heroPara.chassisPara.BRPara.kd=2;
    heroPara.chassisPara.BRPara.pMax=20000;
    heroPara.chassisPara.BRPara.iMax=20000;
    heroPara.chassisPara.BRPara.dMax=20000;
    heroPara.chassisPara.BRPara.max=16384;

    heroPara.chassisPara.reductionRate=1;
    heroPara.chassisPara.kp=16;
    heroPara.chassisPara.ki=0;
    heroPara.chassisPara.kd=2;
    heroPara.chassisPara.pMax=20000;
    heroPara.chassisPara.iMax=20000;
    heroPara.chassisPara.dMax=20000;
    heroPara.chassisPara.max=16384;
    heroPara.chassisPara.Followkp=9.0f;
    heroPara.chassisPara.Followki=0.0005f;
    heroPara.chassisPara.Followkd=3.0f;
    heroPara.chassisPara.FollowpMax=3000.0;
    heroPara.chassisPara.FollowiMax=300.0;
    heroPara.chassisPara.FollowdMax=3000.0;
    heroPara.chassisPara.FollowMax=1500.0;

    heroPara.gimbalPara.GMPPara.CAN=CAN_TYPE_1;
    heroPara.gimbalPara.GMPPara.RxID=0x206;
    heroPara.gimbalPara.GMPPara.reductionRate=1;
    heroPara.gimbalPara.GMPPara.kp=28000;
    heroPara.gimbalPara.GMPPara.ki=0.0;
    heroPara.gimbalPara.GMPPara.kd=1000.0;
    heroPara.gimbalPara.GMPPara.pMax=50000.0;
    heroPara.gimbalPara.GMPPara.iMax=500.0;
    heroPara.gimbalPara.GMPPara.dMax=50000.0;
    heroPara.gimbalPara.GMPPara.max=30000.0;
    heroPara.gimbalPara.GMPPara.anglekp=0.32;
    heroPara.gimbalPara.GMPPara.angleki=0;
    heroPara.gimbalPara.GMPPara.anglekd=0;
    heroPara.gimbalPara.GMPPara.anglepMax=100.0;
    heroPara.gimbalPara.GMPPara.angleiMax=5.0;
    heroPara.gimbalPara.GMPPara.angledMax=100.0;
    heroPara.gimbalPara.GMPPara.anglemax=15.0;
    heroPara.gimbalPara.GMPPara.zero=610;

    heroPara.gimbalPara.GMYPara.CAN=CAN_TYPE_2;
    heroPara.gimbalPara.GMYPara.RxID=0x207;
    heroPara.gimbalPara.GMYPara.reductionRate=1;
    heroPara.gimbalPara.GMYPara.kp=25000.0;
    heroPara.gimbalPara.GMYPara.ki=500.0;
    heroPara.gimbalPara.GMYPara.kd=0;
    heroPara.gimbalPara.GMYPara.pMax=50000.0;
    heroPara.gimbalPara.GMYPara.iMax=500.0;
    heroPara.gimbalPara.GMYPara.dMax=50000.0;
    heroPara.gimbalPara.GMYPara.max=30000.0;
    heroPara.gimbalPara.GMYPara.anglekp=0.25;
    heroPara.gimbalPara.GMYPara.angleki=0;
    heroPara.gimbalPara.GMYPara.anglekd=0;
    heroPara.gimbalPara.GMYPara.anglepMax=100.0;
    heroPara.gimbalPara.GMYPara.angleiMax=5.0;
    heroPara.gimbalPara.GMYPara.angledMax=100.0;
    heroPara.gimbalPara.GMYPara.anglemax=10.0;
    heroPara.gimbalPara.GMYPara.zero=705;


    heroPara.shootPara.stirPara.RxID=0x208;
    heroPara.shootPara.stirPara.reductionRate=36;
    heroPara.shootPara.stirPara.kp=10.0;
    heroPara.shootPara.stirPara.ki=0;
    heroPara.shootPara.stirPara.kd=3;
    heroPara.shootPara.stirPara.pMax=10000;
    heroPara.shootPara.stirPara.iMax=0.0;
    heroPara.shootPara.stirPara.dMax=10000;
    heroPara.shootPara.stirPara.max=10000.0;
    heroPara.shootPara.stirPara.anglekp=200;
    heroPara.shootPara.stirPara.angleki=0;
    heroPara.shootPara.stirPara.anglekd=20;
    heroPara.shootPara.stirPara.anglepMax=15000.0;
    heroPara.shootPara.stirPara.angleiMax=10000.0;
    heroPara.shootPara.stirPara.angledMax=15000.0;
    heroPara.shootPara.stirPara.anglemax=15000.0;
    heroPara.shootPara.stirPara.CAN=CAN_TYPE_2;


    heroPara.shootPara.rodPara.RxID=0x205;
    heroPara.shootPara.rodPara.reductionRate=36;
    heroPara.shootPara.rodPara.kp=35.0;
    heroPara.shootPara.rodPara.ki=0;
    heroPara.shootPara.rodPara.kd=5;
    heroPara.shootPara.rodPara.pMax=10000;
    heroPara.shootPara.rodPara.iMax=0.0;
    heroPara.shootPara.rodPara.dMax=10000;
    heroPara.shootPara.rodPara.max=3000.0;
    heroPara.shootPara.rodPara.anglekp=200;
    heroPara.shootPara.rodPara.angleki=0;
    heroPara.shootPara.rodPara.anglekd=0;
    heroPara.shootPara.rodPara.anglepMax=15000.0;
    heroPara.shootPara.rodPara.angleiMax=3000.0;
    heroPara.shootPara.rodPara.angledMax=15000.0;
    heroPara.shootPara.rodPara.anglemax=15000.0;
    heroPara.shootPara.rodPara.CAN=CAN_TYPE_1;

    heroPara.shootPara.fricLPara.RxID=0x201;
    heroPara.shootPara.fricLPara.reductionRate=1;
    heroPara.shootPara.fricLPara.kp=24.0;
    heroPara.shootPara.fricLPara.ki=0.0;
    heroPara.shootPara.fricLPara.kd=7.05;
    heroPara.shootPara.fricLPara.pMax=15000;
    heroPara.shootPara.fricLPara.iMax=15000;
    heroPara.shootPara.fricLPara.dMax=15000;
    heroPara.shootPara.fricLPara.max=10000;
    heroPara.shootPara.fricLPara.CAN=CAN_TYPE_1;

    heroPara.shootPara.fricRPara.RxID=0x202;
    heroPara.shootPara.fricRPara.reductionRate=1;
    heroPara.shootPara.fricRPara.kp=24.0;
    heroPara.shootPara.fricRPara.ki=0.0;
    heroPara.shootPara.fricRPara.kd=7.05;
    heroPara.shootPara.fricRPara.pMax=15000;
    heroPara.shootPara.fricRPara.iMax=15000;
    heroPara.shootPara.fricRPara.dMax=15000;
    heroPara.shootPara.fricRPara.max=10000;
    heroPara.shootPara.fricRPara.CAN=CAN_TYPE_1;

    heroPara.chassisPara.FLPara.reductionRate=heroPara.chassisPara.reductionRate;
    heroPara.chassisPara.FLPara.kp=heroPara.chassisPara.kp;
    heroPara.chassisPara.FLPara.ki=heroPara.chassisPara.ki;
    heroPara.chassisPara.FLPara.kd=heroPara.chassisPara.kd;
    heroPara.chassisPara.FLPara.pMax=heroPara.chassisPara.pMax;
    heroPara.chassisPara.FLPara.iMax=heroPara.chassisPara.iMax;
    heroPara.chassisPara.FLPara.dMax=heroPara.chassisPara.dMax;
    heroPara.chassisPara.FLPara.max=heroPara.chassisPara.max;

    heroPara.chassisPara.FRPara.reductionRate=heroPara.chassisPara.reductionRate;
    heroPara.chassisPara.FRPara.kp=heroPara.chassisPara.kp;
    heroPara.chassisPara.FRPara.ki=heroPara.chassisPara.ki;
    heroPara.chassisPara.FRPara.kd=heroPara.chassisPara.kd;
    heroPara.chassisPara.FRPara.pMax=heroPara.chassisPara.pMax;
    heroPara.chassisPara.FRPara.iMax=heroPara.chassisPara.iMax;
    heroPara.chassisPara.FRPara.dMax=heroPara.chassisPara.dMax;
    heroPara.chassisPara.FRPara.max=heroPara.chassisPara.max;

    heroPara.chassisPara.BLPara.reductionRate=heroPara.chassisPara.reductionRate;
    heroPara.chassisPara.BLPara.kp=heroPara.chassisPara.kp;
    heroPara.chassisPara.BLPara.ki=heroPara.chassisPara.ki;
    heroPara.chassisPara.BLPara.kd=heroPara.chassisPara.kd;
    heroPara.chassisPara.BLPara.pMax=heroPara.chassisPara.pMax;
    heroPara.chassisPara.BLPara.iMax=heroPara.chassisPara.iMax;
    heroPara.chassisPara.BLPara.dMax=heroPara.chassisPara.dMax;
    heroPara.chassisPara.BLPara.max=heroPara.chassisPara.max;

    heroPara.chassisPara.BRPara.reductionRate=heroPara.chassisPara.reductionRate;
    heroPara.chassisPara.BRPara.kp=heroPara.chassisPara.kp;
    heroPara.chassisPara.BRPara.ki=heroPara.chassisPara.ki;
    heroPara.chassisPara.BRPara.kd=heroPara.chassisPara.kd;
    heroPara.chassisPara.BRPara.pMax=heroPara.chassisPara.pMax;
    heroPara.chassisPara.BRPara.iMax=heroPara.chassisPara.iMax;
    heroPara.chassisPara.BRPara.dMax=heroPara.chassisPara.dMax;
    heroPara.chassisPara.BRPara.max=heroPara.chassisPara.max;
    testMotorPara.RxID=0x203;
    testMotorPara.CAN=CAN_TYPE_1;
    testMotorPara.reductionRate=1;
    testMotorPara.kp=16;
    testMotorPara.ki=0;
    testMotorPara.kd=2;
    testMotorPara.pMax=20000;
    testMotorPara.iMax=20000;
    testMotorPara.dMax=20000;
    testMotorPara.max=32767;
}
#endif

#ifdef INFANTRY

void setPara()
{

    infantryPara.chassisPara.followDirection=2.0;
    infantryPara.chassisPara.FLPara.RxID =0x201;
    infantryPara.chassisPara.FRPara.RxID =0x202;
    infantryPara.chassisPara.BLPara.RxID =0x203;
    infantryPara.chassisPara.BRPara.RxID =0x204;
    infantryPara.chassisPara.FLPara.CAN=CAN_TYPE_2;
    infantryPara.chassisPara.FRPara.CAN=CAN_TYPE_2;
    infantryPara.chassisPara.BLPara.CAN=CAN_TYPE_2;
    infantryPara.chassisPara.BRPara.CAN=CAN_TYPE_2;

    infantryPara.chassisPara.reductionRate=1;
    infantryPara.chassisPara.kp=16;
    infantryPara.chassisPara.ki=0;
    infantryPara.chassisPara.kd=2;
    infantryPara.chassisPara.pMax=30000;
    infantryPara.chassisPara.iMax=20000;
    infantryPara.chassisPara.dMax=20000;
    infantryPara.chassisPara.max=32767;
    infantryPara.chassisPara.Followkp=5.0f;
    infantryPara.chassisPara.Followki=0.13f;
    infantryPara.chassisPara.Followkd=2.0f;
    infantryPara.chassisPara.FollowpMax=6000.0;
    infantryPara.chassisPara.FollowiMax=300.0;
    infantryPara.chassisPara.FollowdMax=3000.0;
    infantryPara.chassisPara.FollowMax=6000.0;

    infantryPara.gimbalPara.GMPPara.CAN=CAN_TYPE_1;
    infantryPara.gimbalPara.GMPPara.RxID=0x206;
    infantryPara.gimbalPara.GMPPara.reductionRate=1;
    #ifdef INFANTRY_3
        infantryPara.gimbalPara.GMPPara.kp=11000;
    #endif
    #ifdef INFANTRY_4
        infantryPara.gimbalPara.GMPPara.kp=12500;
    #endif
    infantryPara.gimbalPara.GMPPara.ki=400.0;
    infantryPara.gimbalPara.GMPPara.kd=1000.0;
    infantryPara.gimbalPara.GMPPara.pMax=30000.0;
    infantryPara.gimbalPara.GMPPara.iMax=20000.0;
    infantryPara.gimbalPara.GMPPara.dMax=20000.0;
    infantryPara.gimbalPara.GMPPara.max=32767.0;
    infantryPara.gimbalPara.GMPPara.anglekp=0.30;
    infantryPara.gimbalPara.GMPPara.angleki=0;
    infantryPara.gimbalPara.GMPPara.anglekd=0.05;
    infantryPara.gimbalPara.GMPPara.anglepMax=5000.0;
    infantryPara.gimbalPara.GMPPara.angleiMax=500.0;
    infantryPara.gimbalPara.GMPPara.angledMax=5000.0;
    infantryPara.gimbalPara.GMPPara.anglemax=5000.0;
    #ifdef INFANTRY_3
        infantryPara.gimbalPara.GMPPara.zero=6300;
    #endif
    #ifdef INFANTRY_4
        infantryPara.gimbalPara.GMPPara.zero=7178;
    #endif
    infantryPara.gimbalPara.GMPPara.presentPercentage=0.8;

    infantryPara.gimbalPara.GMYPara.CAN=CAN_TYPE_2;
    infantryPara.gimbalPara.GMYPara.RxID=0x205;
    infantryPara.gimbalPara.GMYPara.reductionRate=1;
    #ifdef INFANTRY_3
        infantryPara.gimbalPara.GMYPara.kp=15000.0;
    #endif
    #ifdef INFANTRY_4
        infantryPara.gimbalPara.GMYPara.kp=14000.0;
    #endif
    infantryPara.gimbalPara.GMYPara.ki=500.0;
    infantryPara.gimbalPara.GMYPara.kd=1500.0;
    infantryPara.gimbalPara.GMYPara.pMax=20000.0;
    infantryPara.gimbalPara.GMYPara.iMax=20000.0;
    infantryPara.gimbalPara.GMYPara.dMax=20000.0;
    infantryPara.gimbalPara.GMYPara.max=32767.0;
    infantryPara.gimbalPara.GMYPara.anglekp=0.30;
    infantryPara.gimbalPara.GMYPara.angleki=0;
    infantryPara.gimbalPara.GMYPara.anglekd=0;
    infantryPara.gimbalPara.GMYPara.anglepMax=5000.0;
    infantryPara.gimbalPara.GMYPara.angleiMax=5000.0;
    infantryPara.gimbalPara.GMYPara.angledMax=5000.0;
    infantryPara.gimbalPara.GMYPara.anglemax=5000.0;
    #ifdef INFANTRY_3
        infantryPara.gimbalPara.GMYPara.zero=5122;
    #endif
    #ifdef INFANTRY_4
        infantryPara.gimbalPara.GMYPara.zero=6477;
    #endif
    infantryPara.gimbalPara.GMYPara.presentPercentage=0.8;

    infantryPara.gimbalPara.GMYPara.gmanglewatch=0.0;


    infantryPara.shootPara.stirPara.RxID=0x205;
    infantryPara.shootPara.stirPara.reductionRate=36;
    infantryPara.shootPara.stirPara.kp=6.0;
    infantryPara.shootPara.stirPara.ki=0;
    infantryPara.shootPara.stirPara.kd=0;
    infantryPara.shootPara.stirPara.pMax=15000;
    infantryPara.shootPara.stirPara.iMax=5000.0;
    infantryPara.shootPara.stirPara.dMax=15000;
    infantryPara.shootPara.stirPara.max=10000.0;
    infantryPara.shootPara.stirPara.anglekp=200;
    infantryPara.shootPara.stirPara.angleki=0;
    infantryPara.shootPara.stirPara.anglekd=0;
    infantryPara.shootPara.stirPara.anglepMax=9000.0;
    infantryPara.shootPara.stirPara.angleiMax=3000.0;
    infantryPara.shootPara.stirPara.angledMax=15000.0;
    infantryPara.shootPara.stirPara.anglemax=9000.0;
    infantryPara.shootPara.stirPara.CAN=CAN_TYPE_1;

    infantryPara.shootPara.stirPara.strintensitywatch=0.0;
    infantryPara.shootPara.stirPara.watch=0.0;

    infantryPara.shootPara.fricLPara.RxID=0x201;
    infantryPara.shootPara.fricLPara.reductionRate=1;
    infantryPara.shootPara.fricLPara.kp=30.0;
    infantryPara.shootPara.fricLPara.ki=0;
    infantryPara.shootPara.fricLPara.kd=9.0;
    infantryPara.shootPara.fricLPara.pMax=15000;
    infantryPara.shootPara.fricLPara.iMax=15000;
    infantryPara.shootPara.fricLPara.dMax=15000;
    infantryPara.shootPara.fricLPara.max=10000;
    infantryPara.shootPara.fricLPara.CAN=CAN_TYPE_1;

    infantryPara.shootPara.fricRPara.RxID=0x202;
    infantryPara.shootPara.fricRPara.reductionRate=1;
    infantryPara.shootPara.fricRPara.kp=30.0;
    infantryPara.shootPara.fricRPara.ki=0;
    infantryPara.shootPara.fricRPara.kd=9.0;
    infantryPara.shootPara.fricRPara.pMax=15000;
    infantryPara.shootPara.fricRPara.iMax=15000;
    infantryPara.shootPara.fricRPara.dMax=15000;
    infantryPara.shootPara.fricRPara.max=10000;
    infantryPara.shootPara.fricRPara.CAN=CAN_TYPE_1;

    infantryPara.shootPara.fricRPara.Watch=0.0;

    infantryPara.shootPara.shootHeat=0.0;
    infantryPara.shootPara.speed=0.0;

    infantryPara.chassisPara.FLPara.reductionRate=infantryPara.chassisPara.reductionRate;
    infantryPara.chassisPara.FLPara.kp=infantryPara.chassisPara.kp;
    infantryPara.chassisPara.FLPara.ki=infantryPara.chassisPara.ki;
    infantryPara.chassisPara.FLPara.kd=infantryPara.chassisPara.kd;
    infantryPara.chassisPara.FLPara.pMax=infantryPara.chassisPara.pMax;
    infantryPara.chassisPara.FLPara.iMax=infantryPara.chassisPara.iMax;
    infantryPara.chassisPara.FLPara.dMax=infantryPara.chassisPara.dMax;
    infantryPara.chassisPara.FLPara.max=infantryPara.chassisPara.max;

    infantryPara.chassisPara.FRPara.reductionRate=infantryPara.chassisPara.reductionRate;
    infantryPara.chassisPara.FRPara.kp=infantryPara.chassisPara.kp;
    infantryPara.chassisPara.FRPara.ki=infantryPara.chassisPara.ki;
    infantryPara.chassisPara.FRPara.kd=infantryPara.chassisPara.kd;
    infantryPara.chassisPara.FRPara.pMax=infantryPara.chassisPara.pMax;
    infantryPara.chassisPara.FRPara.iMax=infantryPara.chassisPara.iMax;
    infantryPara.chassisPara.FRPara.dMax=infantryPara.chassisPara.dMax;
    infantryPara.chassisPara.FRPara.max=infantryPara.chassisPara.max;

    infantryPara.chassisPara.BLPara.reductionRate=infantryPara.chassisPara.reductionRate;
    infantryPara.chassisPara.BLPara.kp=infantryPara.chassisPara.kp;
    infantryPara.chassisPara.BLPara.ki=infantryPara.chassisPara.ki;
    infantryPara.chassisPara.BLPara.kd=infantryPara.chassisPara.kd;
    infantryPara.chassisPara.BLPara.pMax=infantryPara.chassisPara.pMax;
    infantryPara.chassisPara.BLPara.iMax=infantryPara.chassisPara.iMax;
    infantryPara.chassisPara.BLPara.dMax=infantryPara.chassisPara.dMax;
    infantryPara.chassisPara.BLPara.max=infantryPara.chassisPara.max;

    infantryPara.chassisPara.BRPara.reductionRate=infantryPara.chassisPara.reductionRate;
    infantryPara.chassisPara.BRPara.kp=infantryPara.chassisPara.kp;
    infantryPara.chassisPara.BRPara.ki=infantryPara.chassisPara.ki;
    infantryPara.chassisPara.BRPara.kd=infantryPara.chassisPara.kd;
    infantryPara.chassisPara.BRPara.pMax=infantryPara.chassisPara.pMax;
    infantryPara.chassisPara.BRPara.iMax=infantryPara.chassisPara.iMax;
    infantryPara.chassisPara.BRPara.dMax=infantryPara.chassisPara.dMax;
    infantryPara.chassisPara.BRPara.max=infantryPara.chassisPara.max;
    testMotorPara.RxID=0x203;
    testMotorPara.CAN=CAN_TYPE_1;
    testMotorPara.reductionRate=1;
    testMotorPara.kp=16;
    testMotorPara.ki=0;
    testMotorPara.kd=2;
    testMotorPara.pMax=20000;
    testMotorPara.iMax=20000;
    testMotorPara.dMax=20000;
    testMotorPara.max=32767;
}

#endif
