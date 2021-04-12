/**
  ******************************************************************************
  * @FileName			    KeyMouseControlTask.cpp
  * @Description            Control the robot by the key and mouse on PC
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

int16_t speed = 0;

#ifdef INFANTRY

void Remote::KeyMouseControl() {

    //Chassis::chassis.LockChassis();

    static WorkState_e lastWorkState = NORMAL_STATE;
    if (Car::car.GetWorkState() <= 0) return;
    if (Car::car.GetWorkState() == NORMAL_STATE) {
        Chassis::chassis.twistState = 0;
        Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, 0);
        Gimbal::gimbal.SetPosition(-channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
    }

    if (Car::car.GetWorkState() == ADDITIONAL_STATE_ONE) {
        Chassis::chassis.twistState = 1;
        Chassis::chassis.SetVelocity(channel.rcol, channel.rrow, 0);
        Gimbal::gimbal.SetPosition(-channel.lcol, channel.lrow, GIMBAL_SPEED_K, GIMBAL_SPEED_K);
        //Shoot::shoot.ShootOneBullet(0);
    }
    if (Car::car.GetWorkState() == ADDITIONAL_STATE_TWO) {
//        INRANGE(mouse.x, -150, 150);
//        INRANGE(mouse.y, -150, 150);

#ifdef USE_CHASSIS_FOLLOW
        Gimbal::gimbal.SetPosition(mouse.y, mouse.x, MOUSE_TO_PITCH_ANGLE_INC_FACT, MOUSE_TO_YAW_ANGLE_INC_FACT);
#else
        Chassis::chassis.SetRotateVelocity(-mouse.x);
#endif
        Shoot::shoot.Fric(true);
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

        MouseModeFSM();

        switch (MouseRMode) {
            case SHORT_CLICK: {
                if (mouse.press_r) {
                    AutoAim::autoAim.autoaimMode = predict;
                    AutoAim::autoAim.energyMode=0.0;
                }
            }
                break;
            case LONG_CLICK: {
                if (mouse.press_r) {
                    AutoAim::autoAim.autoaimMode = predict;
                    AutoAim::autoAim.energyMode=0.0;
                    AutoAim::autoAim.AutoShoot();
                }
            }
                break;
            default:
                AutoAim::autoAim.autoaimMode = manual;
                break;
        }

        switch (MouseLMode) {
            case SHORT_CLICK: {
            #ifdef SHOOT_SPEED_TEST
                //Shoot::shoot.Shoot(0);
            #endif
            }
                break;
            case LONG_CLICK: {
                if (!mouse.press_r) {
            #ifdef SHOOT_SPEED_TEST
                Shoot::shoot.ShootNoStop(0);
            #else
                Shoot::shoot.ShootOneBullet(0);
            #endif
                }
            }
            default:
                break;
        }

        KeyboardModeFSM();

        switch (KeyboardMode) {
            case SHIFT_CTRL: {
                Shoot::shoot.SetBurst(1);
                Chassis::chassis.SetDashFlag((uint8_t)0);
                break;
            }
            case CTRL: {
                Shoot::shoot.SetBurst(0);
                Chassis::chassis.SetDashFlag((uint8_t)0);
                if (key.v & KEY_R) {
                    Gate::gate.SetGateState(open);
                    //HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,GPIO_PIN_14);
                }
                if (key.v & KEY_G) {
                    //if (chassis_lock) GMP.TargetAngle = -20;
                }
                break;
            }
            case SHIFT: {
                Shoot::shoot.SetBurst(0);
                Chassis::chassis.SetDashFlag((uint8_t)1);
                Chassis::chassis.UnlockChassis();
                break;
            }
            case NO_CHANGE: {
                Shoot::shoot.SetBurst(0);
                Chassis::chassis.SetDashFlag((uint8_t)0);
                if (key.v & KEY_G) Chassis::chassis.LockChassis();
                break;
            }
        }

        //云台微调
        if (Chassis::chassis.IsLocked()) {

        }

        if (key.v & KEY_W) {
            Chassis::chassis.AddVelocity(10, 0);
        } else if (key.v & KEY_S) {
            Chassis::chassis.AddVelocity(-10, 0);
        }

        if (key.v & KEY_A) {
            Chassis::chassis.AddVelocity(0, -10);
        } else if (key.v & KEY_D) {
            Chassis::chassis.AddVelocity(0, 10);
        }

//        if (!(key.v & (KEY_W | KEY_S | KEY_A | KEY_D))) {
//            Chassis::chassis.SetVelocity(0, 0);
//        }

        if (!(key.v & (KEY_W | KEY_S ))) {
            Chassis::chassis.SetFBVelocity();
        }

        if (!(key.v & (KEY_A | KEY_D))) {
           Chassis::chassis.SetLRVelocity();
        }
        if (key.v & KEY_Q) {
            Chassis::chassis.twistState = 1;
            Chassis::chassis.UnlockChassis();
        }
        if (key.v & KEY_E) {

        }
        if (key.v & KEY_F) {
            Chassis::chassis.twistState = 0;
            Chassis::chassis.UnlockChassis();
            Gate::gate.SetGateState(close);
            //HAL_GPIO_TogglePin(LED_RED_GPIO_Port,GPIO_PIN_11);
        }

#ifdef SHOOT_SPEED_TEST //改变射速
        if(key.v & KEY_Z){
            Shoot::shoot.SetBulletSpeed(FRIC_SPEED_1);
        }
        if(key.v & KEY_X){
            Shoot::shoot.SetBulletSpeed(FRIC_SPEED_2);
        }
        if(key.v & KEY_C){
            Shoot::shoot.SetBulletSpeed(FRIC_SPEED_3);
        }
        if(key.v & KEY_V){
            Shoot::shoot.SetBulletSpeed(FRIC_SPEED_4);
        }
#endif

        /*
        if (KeyboardMode == SHIFT_CTRL || KeyboardMode == CTRL ||
            (mouse.press_r && mouse.press_l && autoShoot_flag == 1)) {
            cur_cd = SHORT_CD;
            if (shoot_cd > cur_cd)
                shoot_cd = cur_cd;
        } else {
            cur_cd = LONG_CD;
            if (shoot_cd > cur_cd)
                shoot_cd = cur_cd;
        }
        */

        //LastBulletSpeed = BulletSpeed;


        //UI test
        if (KeyboardMode == CTRL && key.v & KEY_B) {
            Client_Graph_Start();
        } else if (key.v & KEY_B) {
            Client_Graph_Clear();
        }

        if (lastWorkState != Car::car.GetWorkState()) {
            //ChassisTwistState = 0;
        }
    }
    lastWorkState = Car::car.GetWorkState();
}

#endif

#ifdef HERO
void Remote::KeyMouseControl() {
    if (Car::car.GetWorkState() <= 0) return;
    if (Car::car.GetWorkState() == ADDITIONAL_STATE_TWO) {

              static uint8_t IfShoot = 0;
              static uint8_t Key_R_state = 0;
                static uint8_t Key_Z_state = 0;

        INRANGE(mouse.x, -150, 150);
        INRANGE(mouse.y, -150, 150);

#ifdef USE_CHASSIS_FOLLOW
                if((mouse.x > 100) || (mouse.x < -100))
                {
                    Chassis::chassis.lock = 0;
                }
                if(Chassis::chassis.lock == 0)
                {
                    Gimbal::gimbal.SetPosition(mouse.y, mouse.x, -MOUSE_TO_PITCH_ANGLE_INC_FACT, MOUSE_TO_YAW_ANGLE_INC_FACT);
                }
#else
        Chassis::chassis.SetRotateVelocity(-mouse.x);
#endif

        Shoot::shoot.Fric(true);
        HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

        MouseModeFSM();

        switch (MouseRMode) {
            case SHORT_CLICK: {
                if (mouse.press_r) {

                }
            }
                break;
            case LONG_CLICK: {
                if (mouse.press_r) {

                }
            }
                break;
            default:

                break;
        }

        switch (MouseLMode) {
            case SHORT_CLICK: {
                            if(!IfShoot)
                            {
                                Shoot::shoot.ShootOneBullet(1);
                                IfShoot = 1;
                            }
            }
                break;
            case LONG_CLICK: {

            }
            default:
                        {
                            IfShoot = 0;
                        }
                break;
        }

        KeyboardModeFSM();

        switch (KeyboardMode) {
            case SHIFT_CTRL: {
                Shoot::shoot.SetBurst(1);
                break;
            }
            case CTRL: {
                Shoot::shoot.SetBurst(0);
                              Chassis::chassis.lock = 0;

                              //上台阶自动
                                if(key.v & KEY_Q)
                                {
                                    Climb::climb.autoClimb();
                                }

                                //上台阶快
                                if(key.v & KEY_X)
                                {
                                    Climb::climb.frontDown();
                                }
                                if(key.v & KEY_Z)
                                {
                                    Climb::climb.frontUp();
                                }
                                if(key.v & KEY_V)
                                {
                                    Climb::climb.backDown();
                                }
                                if(key.v & KEY_C)
                                {
                                    Climb::climb.backUp();
                                }

                break;
            }
            case SHIFT: {
                Shoot::shoot.SetBurst(0);
                Chassis::chassis.lock = 0;
                break;
            }
            case NO_CHANGE: {
                Shoot::shoot.SetBurst(0);
                              if(key.v & KEY_R && Key_R_state == 0)
                                {
                                    Chassis::chassis.lock = 1;
                                    Key_R_state = 1;
                                }
                                if((!(key.v & KEY_R)) && Key_R_state == 1)
                                {
                                    Key_R_state = 0;
                                }

                                //打开弹舱门
                                if(key.v &KEY_X)
                                {

                                }

                                //吊射模式云台微调
                                if(Chassis::chassis.lock)
                                {
                                    if(key.v & KEY_W)  			//key: w
                                        Gimbal::gimbal.SetPosition(1.0, 0, SHOOTMODE_GM_ADJUST_ANGLE, 0);
                                    else if(key.v & KEY_S) 	//key: s
                                        Gimbal::gimbal.SetPosition(-1.0, 0, SHOOTMODE_GM_ADJUST_ANGLE, 0);
                                    if(key.v & KEY_D)  			//key: d
                                        Gimbal::gimbal.SetPosition(0, 1.0, 0, SHOOTMODE_GM_ADJUST_ANGLE);
                                    else if(key.v & KEY_A) 	//key: a
                                        Gimbal::gimbal.SetPosition(0, -1.0, 0, SHOOTMODE_GM_ADJUST_ANGLE);
                                }

                                //底盘运动控制
                                if (key.v & KEY_W) {
                                        Chassis::chassis.AddVelocity(10, 0);
                                } else if (key.v & KEY_S) {
                                        Chassis::chassis.AddVelocity(-10, 0);
                                }

                                if (key.v & KEY_A) {
                                        Chassis::chassis.AddVelocity(0, -10);
                                } else if (key.v & KEY_D) {
                                        Chassis::chassis.AddVelocity(0, 10);
                                }

                                if (!(key.v & (KEY_W | KEY_S | KEY_A | KEY_D))) {
                                        Chassis::chassis.SetVelocity(0, 0);
                                }


                                //扭腰、陀螺模式选择
                                if(key.v & KEY_Q) { Chassis::chassis.twistState = 1;
                                        Chassis::chassis.lock = 0;}
                                if(key.v & KEY_E) { Chassis::chassis.twistState = 3;
                                        Chassis::chassis.lock = 0;}

                                //回头取弹
                                if(key.v & KEY_Z && Key_Z_state == 0)
                                {

                                    //Chassis::chassis.TurnBack();
                                    Key_Z_state = 1;
                                }
                                if((!(key.v & KEY_Z)) && Key_Z_state == 1)
                                {
                                    Key_Z_state = 0;
                                }

                                //自瞄模式选择
                                if(key.v & KEY_C)
                                {
                                    AutoAim::autoAim.autoaimMode = predict;
                                    if(mouse.press_r) AutoAim::autoAim.AutoShoot();
                                }
                                else if(key.v & KEY_V)
                                {
                                    AutoAim::autoAim.autoaimMode = antiTop;
                                    if(mouse.press_r) AutoAim::autoAim.AutoShoot();
                                }
                                else if(mouse.press_r)
                                {
                                    if(Gimbal::gimbal.GMRealAngle(GMPitch) < -10)
                                    {
                                        AutoAim::autoAim.autoaimMode = predict;
                                        if(MouseRMode == LONG_CLICK) AutoAim::autoAim.AutoShoot();
                                    }
                                    else
                                    {
                                        AutoAim::autoAim.autoaimMode = antiTop;
                                        AutoAim::autoAim.AutoShoot();
                                    }
                                }
                                else
                                {
                                    AutoAim::autoAim.autoaimMode = manual;
                                }

                break;
            }
        }

        if (key.v & KEY_F) {
            Chassis::chassis.twistState = 0;
            Chassis::chassis.lock = 0;
#ifdef INFANTRY
            Gate::gate.SetGateState(close);
#endif
#ifdef HERO

#endif
                      //Chassis::chassis.TurnFront();
        }

        //UI test
        if (KeyboardMode == CTRL && key.v & KEY_B) {
            Client_Graph_Start();
        } else if (key.v & KEY_B) {
            Client_Graph_Clear();
        }
    }
}
#endif

void Remote::KeyboardModeFSM() {
    if ((key.v & 0x30) == 0x30) {
        KeyboardMode = SHIFT_CTRL;
    } else if (key.v & KEY_SHIFT) {
        KeyboardMode = SHIFT;
    } else if (key.v & KEY_CTRL) {
        KeyboardMode = CTRL;
    } else {
        KeyboardMode = NO_CHANGE;
    }

    if (KeyboardMode != LastKeyboardMode && KeyboardMode == SHIFT_CTRL) {}
    if (KeyboardMode == SHIFT) {} else {}
    if (KeyboardMode == CTRL) {}
    LastKeyboardMode = KeyboardMode;
}

void Remote::MouseModeFSM() {
    static int16_t counterl = 0;
    static int16_t counterr = 0;
    switch (MouseLMode) {
        case SHORT_CLICK: {
            counterl++;
            if (mouse.press_l == 0) {
                MouseLMode = NO_CLICK;
                counterl = 0;
            } else if (counterl >= 500) {
                MouseLMode = LONG_CLICK;
                counterl = 0;
            } else {
                MouseLMode = SHORT_CLICK;
            }
        }
            break;
        case LONG_CLICK: {
            if (mouse.press_l == 0) {
                MouseLMode = NO_CLICK;
            } else {
                MouseLMode = LONG_CLICK;
            }
        }
            break;
        case NO_CLICK: {
            if (mouse.press_l) {
                Shoot::shoot.ShootOneBullet(0);
                MouseLMode = SHORT_CLICK;
            }

        }
            break;
    }

    switch (MouseRMode) {
        case SHORT_CLICK:{
            counterr++;
            if (mouse.press_r == 0) {
                MouseRMode = NO_CLICK;
                counterr = 0;
            } else if (counterr >= 500) {
                MouseRMode = LONG_CLICK;
                counterr = 0;
            } else {
                MouseRMode = SHORT_CLICK;
            }
        }
            break;
        case LONG_CLICK: {
            if (mouse.press_r == 0) {
                MouseRMode = NO_CLICK;
            } else {
                MouseRMode = LONG_CLICK;
            }
        }
            break;
        case NO_CLICK: {
            if (mouse.press_r) {
                MouseRMode = SHORT_CLICK;
            }
        }
            break;
    }
}
