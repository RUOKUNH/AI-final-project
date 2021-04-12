/**
  ******************************************************************************
  * @FileName			    Parameter.h
  * @Description            Store parameters frequently mofifyied
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

#ifndef RM_FRAME2_PARAMETER_H
#define RM_FRAME2_PARAMETER_H

#include "CANTask.h"

struct PowerLimitation_t {           //功率限制参数
    double kIntensityToMoment;
    double PowerMax;
    double rotateSpeedRate;
    double CMFLRotateSpeed;
    double CMFRRotateSpeed;
    double CMBLRotateSpeed;
    double CMBRRotateSpeed;
};





struct normalMotorPara_t{                //   双环pid电机
    uint16_t RxID;
    CanType_e CAN;
    float reductionRate;
    float kp;
    float ki;
    float kd;
    float pMax;
    float iMax;
    float dMax;
    float max;
    float anglekp;
    float angleki;
    float anglekd;
    float anglepMax;
    float angleiMax;
    float angledMax;
    float anglemax;
    float zero;

    float gmanglewatch;
    float strintensitywatch;
    float watch;

    //低通滤波参数
    float presentPercentage;
    /*//变结构体PID参数
    float kpA;
    float kpB;
    float kpC;
    float kiA;
    float kiC;
    float kiK0;
    float kdA;
    float kdB;
    float kdC;

    float anglekpA;
    float anglekpB;
    float anglekpC;
    float anglekiA;
    float anglekiC;
    float anglekiK0;
    float anglekdA;
    float anglekdB;
    float anglekdC;*/

};

struct chassisMotorPara_t{                      //单环pid电机
    uint16_t RxID;
    CanType_e CAN;
    float reductionRate;
    float kp;
    float ki;
    float kd;
    float pMax;
    float iMax;
    float dMax;
    float max;

    float Watch;


};

struct chassisPara_t{
    chassisMotorPara_t FLPara;
    chassisMotorPara_t FRPara;
    chassisMotorPara_t BLPara;
    chassisMotorPara_t BRPara;


    float reductionRate;
    float kp;
    float ki;
    float kd;
    float pMax;
    float iMax;
    float dMax;
    float max;

    float followDirection;
    float Followkp;
    float Followki;
    float Followkd;
    float FollowpMax;
    float FollowiMax;
    float FollowdMax;
    float FollowMax;

    //变结构体PID参数
    float kpA;
    float kpB;
    float kpC;
    float kiA;
    float kiC;
    float kiK0;
    float kdA;
    float kdB;
    float kdC;

    //变结构体PID参数
    float FollowkpA;
    float FollowkpB;
    float FollowkpC;
    float FollowkiA;
    float FollowkiC;
    float FollowkiK0;
    float FollowkdA;
    float FollowkdB;
    float FollowkdC;

};
struct gimbalPara_t{
    normalMotorPara_t GMPPara;
    normalMotorPara_t GMYPara;
};

struct shootPara_t{
    chassisMotorPara_t fricLPara;
    chassisMotorPara_t fricRPara;
    normalMotorPara_t stirPara;

    uint16_t shootHeat;
    uint8_t speed;
#ifdef HERO
    normalMotorPara_t rodPara;
#endif

};

struct heroPara_t {
    gimbalPara_t gimbalPara;
    chassisPara_t chassisPara;
    shootPara_t shootPara;

};

struct infantryPara_t {
    gimbalPara_t gimbalPara;
    chassisPara_t chassisPara;
    shootPara_t shootPara;
};

struct monitorPara_Motor_t{
    float_t targetAngle,realAngle,
    angle_output,speed_output;


    float reductionRate;
    float kp;
    float ki;
    float kd;
    float pMax;
    float iMax;
    float dMax;
    float max;

    float followDirection;
    float Followkp;
    float Followki;
    float Followkd;
    float FollowpMax;
    float FollowiMax;
    float FollowdMax;
    float FollowMax;

    //变结构体PID参数
    float kpA;
    float kpB;
    float kpC;
    float kiA;
    float kiC;
    float kiK0;
    float kdA;
    float kdB;
    float kdC;

    //变结构体PID参数
    float FollowkpA;
    float FollowkpB;
    float FollowkpC;
    float FollowkiA;
    float FollowkiC;
    float FollowkiK0;
    float FollowkdA;
    float FollowkdB;
    float FollowkdC;
};
struct monitorPara_imu_t{
    float_t rol_m,pit_m,yaw_m;
};
struct monitorPara_t{
    float_t watches[5];
    float_t watch1[20];
    float_t dataBuffer1[4];
    float_t targetAngle[10];
};
void setPara();


extern infantryPara_t infantryPara;
extern heroPara_t heroPara;


extern float bullet_timer;
extern float rotate_timer;


inline void refreshPara(monitorPara_Motor_t _monitorPara,\
float_t _realAngle,float_t _angle_output,float_t _speed_output){
        _monitorPara.realAngle=_realAngle;
        _monitorPara.angle_output=_angle_output;
        _monitorPara.speed_output=_speed_output;
        };

extern gimbalPara_t gimbalPara;
extern chassisPara_t chassisPara;
extern normalMotorPara_t stirPara;
extern chassisMotorPara_t fricLPara;
extern chassisMotorPara_t fricRPara;


extern monitorPara_Motor_t monitorPara_testMotor;
extern monitorPara_Motor_t monitorPara_GYMotor;
extern monitorPara_imu_t monitorPara_imu;
extern monitorPara_t monitorPara;

extern chassisMotorPara_t testMotorPara;

extern monitorPara_t monitorPara;

extern chassisMotorPara_t dartFLPara;
extern chassisMotorPara_t dartFRPara;
extern chassisMotorPara_t dartMLPara;
extern chassisMotorPara_t dartMRPara;
extern chassisMotorPara_t dartBLPara;
extern chassisMotorPara_t dartBRPara;

#endif //RM_FRAME2_PARAMETER_H
