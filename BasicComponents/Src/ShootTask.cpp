/**
  ******************************************************************************
  * @FileName			    ShootTask.cpp
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

#include <ShootTask.h>

#include "includes.h"
#define USE_HEAT_LIMIT

int16_t  Shoot::shootcount=0;
int16_t Shoot::refeshcount=0;
int16_t Shoot::mcount=0;
bool Shoot::flag1=false;


Shoot::Shoot() {

}

void Shoot::Reset(shootPara_t shootPara)
{
    STIR.Reset(shootPara.stirPara);
#ifdef HERO
  //  ROD.Reset(shootPara.rodPara);
#endif
    FRICL.Reset(shootPara.fricLPara);
    FRICR.Reset(shootPara.fricRPara);
    ShootState=0;
}

void Shoot::Fric(bool ON) {
    fric_ON = ON;
    if (fric_ON) {
#ifdef  INFANTRY
        FRICL.targetSpeed = -fricSpeed;
        FRICR.targetSpeed = fricSpeed;
#endif
#ifdef  HERO
        FRICL.targetSpeed = fricSpeed;
        FRICR.targetSpeed = -fricSpeed;
#endif
        if(blockFlag==0)
        {ShootState=1;}
        else
        {ShootState=0;}
    } else {
        FRICL.targetSpeed = 0;
        FRICR.targetSpeed = 0;
        ShootState=0;
    }
}

void Shoot::RefreshAngleGap(){
    STIR.targetAngle = STIR.GetRealAngle();
}

void Shoot::RefreshExtraStep(){
    static float gapAngle;
    static float gapAngleUse;
    gapAngle = STIR.targetAngle - STIR.GetRealAngle();
    gapAngleUse = ceil(fabs(gapAngle) / fabs(float(STIR_STEP_ANGLE))) * STIR_STEP_ANGLE;
    STIR.targetAngle = STIR.targetAngle + gapAngleUse;
}

void Shoot::Count() {
    if (shootCD > 0) shootCD--;
    if (stirCount > 0) stirCount--;
    if (heat0Count > 0) heat0Count--;
    if (heat1Count > 0) heat1Count--;
}

void Shoot::FakeHeatCalc() {
    //if (JUDGE_State == ONLINE) {
    maxHeat1 = RefereeData.GameRobotState.shooter_id1_42mm_cooling_limit;
    coolDown1 = RefereeData.GameRobotState.shooter_id1_42mm_cooling_rate;
    if (heat1Count == 0)fakeHeat1 = RefereeData.PowerHeat.shooter_id1_42mm_cooling_heat;
    maxHeat0 = RefereeData.GameRobotState.shooter_id1_17mm_cooling_limit;
    coolDown0 = RefereeData.GameRobotState.shooter_id1_17mm_cooling_rate;
    if (heat0Count == 0) fakeHeat0 = RefereeData.PowerHeat.shooter_id1_17mm_cooling_heat;
    if (fakeHeat0 >= coolDown0 / 1000) fakeHeat0 -= coolDown0 / 1000;
    else fakeHeat0 = 0;
    if (fakeHeat1 >= coolDown1 / 1000) fakeHeat1 -= coolDown1 / 1000;
    else fakeHeat1 = 0;
}



void Shoot::SwitchBulletSpeed() {
    maxSpeed0 = RefereeData.GameRobotState.shooter_id1_17mm_speed_limit;
    maxSpeed1 = RefereeData.GameRobotState.shooter_id1_42mm_speed_limit;
    switch (maxSpeed0) {
        case MAXSPEED1:
            fricSpeed = FRIC_SPEED_1;
            break;
        case MAXSPEED2:
            fricSpeed = FRIC_SPEED_2;
            break;
        case MAXSPEED3:
            fricSpeed = FRIC_SPEED_3;
            break;
        case MAXSPEED4:
            fricSpeed = FRIC_SPEED_4;
            break;
    }
    //LastFricSpeed = RefereeData.GameRobotState.shooter_heat0_speed_limit;
}

//发射一发弹丸
void Shoot::ShootOneBullet(uint8_t state) {
#ifndef USE_HEAT_LIMIT
    STIR.targetAngle -= STIR_STEP_ANGLE;
#else
    if (state == 0) {
        if ((maxHeat0 - fakeHeat0 >= 15 || burst) && fric_ON && !blockFlag && shootCD == 0) {
            STIR.targetAngle -= STIR_STEP_ANGLE;
            fakeHeat0 += 10;
            heat0Count = 200;
            shootCD = currentCD;
        }
    }
    if (state == 1) {
        if ((maxHeat1 - fakeHeat1 >= 20 || burst) && fric_ON && !blockFlag && shootCD == 0) {
            STIR.targetAngle -= STIR_STEP_ANGLE;
            fakeHeat1 += 100;
            heat1Count = 200;
            shootCD = currentCD;
        }
    }
#endif
}

void Shoot::ShootNoStop(uint8_t state){
    static uint8_t shoot_little_count = KEEP_ON_COOLDOWN_TIME;
    if(shoot_little_count == 0){
        Shoot::shoot.ShootOneBullet(state);
        shoot_little_count = KEEP_ON_COOLDOWN_TIME;
    }
    shoot_little_count--;
}

#ifdef HERO

void Shoot::Extend() {
  //  ROD.targetAngle-= ROD_ANGLE;
}

void Shoot::Contrast() {
//    ROD.targetAngle+= ROD_ANGLE;
}

void Shoot::SlowContrast() {
  //  if (ROD.targetAngle < 0) ROD.targetAngle += 3;
   // if (ROD.targetAngle > 0) ROD.targetAngle -= 3;
}

#endif
//卡弹处理
#ifdef HERO
void Shoot::BulletBlockHandler() {

    OnePush(abs(STIR.GetRxMsgC6x0().moment) > 9800,
            {

                STIR.targetAngle += 0.25 * STIR_STEP_ANGLE;
                stirCount = 100;
                blockFlag = 1;
                ShootState=0;
            });
    if (blockFlag == 1) {
        OnePush(stirCount == 0,
                {
                    if (abs(STIR.GetRxMsgC6x0().moment) < 500) {
                        STIR.targetAngle -= 0.25 * STIR_STEP_ANGLE;
                        blockFlag = 0;
                        ShootState=1;
                    } else {
              //          STIR.targetAngle += STIR_STEP_ANGLE;
                        stirCount = 100;
                    }
                });
    }
}
#endif
//卡弹处理
#ifdef INFANTRY
void Shoot::BulletBlockHandler() {
    /*OnePush(STIR.GetRxMsgC6x0().moment < -6000,
            {
                STIR.targetAngle += 1.5 * STIR_STEP_ANGLE;
                stirCount = 100;
                blockFlag = 1;
            });
    if (blockFlag == 1) {
        OnePush(stirCount == 0,
                {
                    if (STIR.GetRxMsgC6x0().moment > -500) {
                        STIR.targetAngle -= 0.5 * STIR_STEP_ANGLE;
                        blockFlag = 0;
                    } else {
                        STIR.targetAngle += STIR_STEP_ANGLE;
                        stirCount = 100;
                    }
                });
    }*/
    static uint8_t countIn = 1;
    static uint8_t countOut = 10;
    if ( STIR.GetRxMsgC6x0().moment < -9500 ) {
        countIn--;
        if (countIn == 0) {
            STIR.targetAngle += 1.5 * STIR_STEP_ANGLE;
            stirCount = 100;
            blockFlag = 1;
            countIn = 1;
        }
    }else{
        countIn = 1;
    }
    if (blockFlag == 1 && stirCount == 0){
        countOut--;
        if (countOut==0){
            countOut = 10;
            if (STIR.GetRxMsgC6x0().moment > -5000) {
                STIR.targetAngle -= 0.5 * STIR_STEP_ANGLE;
                blockFlag = 0;
            }else{
                STIR.targetAngle += STIR_STEP_ANGLE;
                stirCount = 100;
            }
        }
    }
}

void Shoot::BulletBlockHandler2(){
    static uint8_t countIn = 1;
    static uint8_t countOut = 5;
    static uint8_t rotateFlag = 0;
    if ( STIR.GetRxMsgC6x0().moment < -9500 ) {
        countIn--;
        if (countIn == 0) {
            //Shoot::RefreshAngleGap();
            Shoot::RefreshExtraStep();
            STIR.targetAngle += HANDLE_BLOCK_STEP_ANGLE;
            stirCount = 50;
            blockFlag = 1;
            countIn = 1;
        }
    }else{
        countIn = 1;
    }
    if (blockFlag == 1 && stirCount == 0){
        countOut--;
        if (countOut==0){
            countOut = 5;
            if (STIR.GetRxMsgC6x0().moment > -6000) {
                if (rotateFlag % 2 == 0){
                    STIR.targetAngle -= HANDLE_BLOCK_STEP_ANGLE;
                }else if (rotateFlag % 2 == 1){
                    STIR.targetAngle += HANDLE_BLOCK_STEP_ANGLE;
                }
                blockFlag = 0;
                rotateFlag = 0;
            }else{
                if (rotateFlag % 2 == 0){
                    STIR.targetAngle -= 2 * HANDLE_BLOCK_STEP_ANGLE;
                    rotateFlag++;
                }else if (rotateFlag % 2 == 1){
                    STIR.targetAngle += 2 * HANDLE_BLOCK_STEP_ANGLE;
                    rotateFlag++;
                }
                stirCount = 50;
            }
        }
    }
}

void Shoot::BulletBlockHandler4() {
   if(fabs(STIR.GetRealAngle() - STIR.targetAngle) > 5.0f ){
       shootcount++;
       if(STIR.GetRealAngle()-STIR.targetAngle > 0)  flag1=false;
       else if(STIR.GetRealAngle()-STIR.targetAngle < 0)  flag1=true;
   }
   else{
       shootcount=0;
       blockFlag=false;
   }

   if(shootcount >= 150 && !flag1 && blockFlag) {
       shootcount=0;
       blockFlag=true;
       STIR.targetAngle += 2 * STIR_STEP_ANGLE;
       flag1 = true;
   }

   if(shootcount >= 150 && flag1 && blockFlag){
       shootcount=0;
       blockFlag=true;
       STIR.targetAngle -= 2 * STIR_STEP_ANGLE;
       flag1 = false;
   }

    //停止发弹
    if(fabs(STIR.GetRxMsgC6x0().moment)>9500) {
        mcount++;
        if (mcount > 13) {
            blockFlag = true;
            mcount = 0;
        }
    }


    //防爆弹
   if(fabs(STIR.GetRxMsgC6x0().moment) < 10) {
       refeshcount++;
       if (refeshcount > 30) {
           RefreshAngleGap();
           refeshcount = 0;
       }
   }


   infantryPara.shootPara.stirPara.watch=STIR.targetAngle;
   monitorPara.watches[1]=blockFlag;
}

#endif
void Shoot::SetBurst(uint8_t _burst) {
    burst = _burst;
}

void Shoot::SetBulletSpeed(uint16_t _bulletSpeed){
    fricSpeed = _bulletSpeed;
}

ESCC6x0RxMsg_t Shoot::GetSTIRRxMsgC6x0() {
    return STIR.GetRxMsgC6x0();
}

ESCC6x0RxMsg_t Shoot::GetFricRRxMsgC6x0() {
    return FRICR.GetRxMsgC6x0();
}

ESCC6x0RxMsg_t Shoot::GetFricLRxMsgC6x0() {
    return FRICL.GetRxMsgC6x0();
}

float Shoot::GetSTIRtargetAngle() {
    return STIR.targetAngle;
}



void Shoot::Handle() {
    SwitchBulletSpeed();
    BulletBlockHandler4();
    FakeHeatCalc();
    FRICL.Handle();
    FRICR.Handle();
    STIR.Handle();

#ifdef HERO
//    ROD.Handle();
#endif

}

void Shoot::BulletBlockHandler3() {
    static int16_t blockcount=0;
    if(fabs(STIR.GetRxMsgC6x0().moment)>8000) blockcount++;
    else blockcount=0;

    if(blockcount>50) {
        RefreshExtraStep();
        blockcount = 0;
    }

}


Shoot Shoot::shoot;