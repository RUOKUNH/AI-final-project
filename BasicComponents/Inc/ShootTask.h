/**
  ******************************************************************************
  * @FileName			    ShootTask.h
  * @Description            Shoot bullets and heat limitation
  * @author                 Steve Young
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2021 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
**/

#ifndef __SHOOTTASK_H__
#define __SHOOTTASK_H__

#include "includes.h"

#define ROD_ANGLE    75
#define MAXHEAT01    120
#define MAXHEAT02    240
#define MAXHEAT03    300
#define COOLDOWN01    20.0f
#define COOLDOWN02    40.0f
#define COOLDOWN03    50.0f

#define MAXHEAT11    200
#define MAXHEAT12    300
#define MAXHEAT13    400
#define COOLDOWN11    20.0f
#define COOLDOWN12    40.0f
#define COOLDOWN13    60.0f

#define MAXSPEED1    15
#define MAXSPEED2    18
#define MAXSPEED3    22
#define MAXSPEED4    30

#ifdef HERO
#define FRIC_SPEED_1    1050
#define FRIC_SPEED_2    4750
#define FRIC_SPEED_3    4750
#define FRIC_SPEED_4    4750
#endif

#ifdef INFANTRY
    #ifdef INFANTRY_3


7#define FRIC_SPEED_1    4250
#define FRIC_SPEED_2    4750
#define FRIC_SPEED_3    5550
#define FRIC_SPEED_4    7300


    #endif
    #ifdef INFANTRY_4

        #define FRIC_SPEED_1    4350
        #define FRIC_SPEED_2    4850
        #define FRIC_SPEED_3    5650
        #define FRIC_SPEED_4    7350
    #endif
#endif

#ifdef HERO
#define STIR_STEP_ANGLE 60/2*60/56
#endif

#ifdef INFANTRY
#define STIR_STEP_ANGLE 45
#endif
#define    LONG_CD            200
#define    SHORT_CD        50
//#define KEEP_ON_SHOOT
#define KEEP_ON_COOLDOWN_TIME   1
//#define KEEP_ON_SHOOT_FIERCELY

#define HANDLE_BLOCK_STEP_ANGLE     30

class Shoot {
private:

    float fakeHeat0{};
    float fakeHeat1{};
    float coolDown0{};
    float coolDown1{};
    uint16_t maxHeat0{MAXHEAT03};
    uint16_t maxHeat1{MAXHEAT13};
    uint16_t maxSpeed0{MAXSPEED1};
    uint16_t maxSpeed1{MAXSPEED1};

    uint16_t heat0Count{};
    uint16_t heat1Count{};
    uint16_t stirCount{};
    uint8_t burst{};
    uint8_t ShootState;
    bool fric_ON{false};
    bool blockFlag{false};
    int16_t shootCD{};
    int16_t currentCD{SHORT_CD};


   

    NormalMotor STIR;
    ChassisMotor FRICL;
    ChassisMotor FRICR;

#ifdef HERO
 //   NormalMotor ROD;
#endif

public:

    static Shoot shoot;

    static int16_t shootcount;
    static int16_t mcount;
    static int16_t refeshcount;
    static bool flag1;

    uint16_t fricSpeed{FRIC_SPEED_1};

    Shoot();
    //一些cnt的更新
    void Count();

    void SwitchBulletSpeed();

    void ShootOneBullet(uint8_t state);

    void ShootNoStop(uint8_t state);

    void BulletBlockHandler();

    void BulletBlockHandler2();

    void BulletBlockHandler3();

    void BulletBlockHandler4();

    void Fric(bool ON);

    void FakeHeatCalc();

    void Handle();

    void Reset(shootPara_t shootPara);

    void SetBurst(uint8_t _burst);    //设置burst模式（不考虑热量限制）

#ifdef HERO
    void Extend();

    void Contrast();

    void SlowContrast();
#endif

    void RefreshAngleGap();

    void RefreshExtraStep();

    void SetBulletSpeed(uint16_t _bulletSpeed);

    float GetSTIRtargetAngle();

    ESCC6x0RxMsg_t GetSTIRRxMsgC6x0();

    ESCC6x0RxMsg_t GetFricRRxMsgC6x0();

    ESCC6x0RxMsg_t GetFricLRxMsgC6x0();

};



extern Shoot shoot;

#endif
